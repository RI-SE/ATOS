#define _GNU_SOURCE

#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/limits.h>
#include <semaphore.h>
#include <vector>
#include <cstring>
#include <string>
#include <algorithm>

#include "shmem.h"

//#define SHMEM_DEBUG /*!< Comment this line in order to suppress debug output */
#ifdef SHMEM_DEBUG
#define debug_print(fmt, ...) \
	do { fprintf(stderr, "%d:%s:%d:%s():" fmt,getpid(),__FILE__,\
	__LINE__,__func__,__VA_ARGS__);} while(0)
#else
#define debug_print(fmt, ...) \
	do {} while(0)
#endif

using namespace std;

#define LOCK_FILE_ENDING ".lck"
#define MEMORY_HEADER_ENDING ".hdr"
#define MEMORY_FILE_ENDING ".mem"
#define SHM_NAME_MAX (NAME_MAX - 4 - sizeof (LOCK_FILE_ENDING))

#define MEMORY_MINSIZE 1

/*!
 * \brief MemoryInformation Stores size and element type data for a section of anonymous memory
 */
typedef struct {
	unsigned int numberOfElements;	//!< Number of elements in the memory
	size_t elementSize;				//!< Size of an individual element
} MemoryInformation;

/*!
 * \brief Memory Stores all information for a shared memory block
 */
class Memory {
public:
	string name = "";								//!< ASCII name of the memory
	void* address = nullptr;						//!< Address of the shared memory
	typedef enum {
		CREATED_MEMORY,
		FOUND_EXISTING_MEMORY,
		FAILED,
		SUCCEEDED
	} MemoryReturnCode;

	Memory(string memoryName) : name(memoryName) {
		// Calls to shm assume the file name starts with a forward slash; ensure this matches
		if (name.front() != '/') {
			name.insert(0, "/");
		}
	}

	Memory(Memory &&other) noexcept {
		debug_print("Move constructor called for memory %s\n", name.c_str());
		name = other.name;
		address = other.address;
		mutex = other.mutex;
		fdHeader = other.fdHeader;
		fdMemory = other.fdMemory;
		virtualMemoryInfo = other.virtualMemoryInfo;
		info = other.info;

		other.name = "";
		other.address = nullptr;
		other.mutex = nullptr;
		other.info = nullptr;
		other.fdHeader = -1;
		other.fdMemory = -1;
		other.virtualMemoryInfo = {0, 0};
	}

	Memory& operator=(Memory&& other) noexcept {
		debug_print("Move assignment operator called for memory %s\n", name.c_str());
		if (this != &other) {
			name = other.name;
			address = other.address;
			mutex = other.mutex;
			fdHeader = other.fdHeader;
			fdMemory = other.fdMemory;
			virtualMemoryInfo = other.virtualMemoryInfo;
			info = other.info;

			other.name = "";
			other.address = nullptr;
			other.mutex = nullptr;
			other.info = nullptr;
			other.fdHeader = -1;
			other.fdMemory = -1;
			other.virtualMemoryInfo = {0, 0};
		}
		return *this;
	}

	~Memory() {
		debug_print("Destructor called for memory %s\n", name.c_str());

		// Unmap memory
		if (address != nullptr) {
			if (munmap(address, virtualMemoryInfo.elementSize == 0 || virtualMemoryInfo.numberOfElements == 0 ?
					   MEMORY_MINSIZE : virtualMemoryInfo.elementSize * virtualMemoryInfo.numberOfElements) == -1) {
				perror("munmap");
			}
		}

		if (info != nullptr) {
			if (munmap(info, sizeof (*info)) == -1) {
				perror("munmap");
			}
		}

		// Close open shared memory files
		if (name != "") {
			if (close(fdHeader) == -1) {
				perror("close");
			}
			if (close(fdMemory) == -1)  {
				perror("close");
			}
		}
		// Close semaphore
		if (mutex != nullptr) {
			if (sem_close(mutex) == -1) {
				perror("sem_close");
			}
		}
		return;
	}

	int claim() { return sem_wait(mutex); }
	int release() { return sem_post(mutex); }

	string getHeaderFileName(void) const { return name + MEMORY_HEADER_ENDING; }
	string getMemoryFileName(void) const { return name + MEMORY_FILE_ENDING; }
	string getLockFileName(void) const { return name + LOCK_FILE_ENDING; }
	ssize_t getTotalSize(void) const { return info == nullptr ? -1 : static_cast<ssize_t>(info->elementSize*info->numberOfElements); }
	size_t getMapSize(void) const { return virtualMemoryInfo.elementSize * virtualMemoryInfo.numberOfElements; }
	int getNumberOfElements(void) const { return info == nullptr ? -1 : static_cast<int>(info->numberOfElements); }
	ssize_t getElementSize(void) const { return info == nullptr ? -1 : static_cast<ssize_t>(info->elementSize); }

	MemoryReturnCode setUp(unsigned int numberOfElements, size_t elementSize) {

		MemoryReturnCode retval = FAILED;

		mutex = sem_open(getLockFileName().c_str(), O_CREAT, S_IRUSR | S_IWUSR, 1);
		if (mutex == SEM_FAILED) {
			perror("sem_open");
			return FAILED;
		}

		claim();

		// First try to open the shared memory header file without creating it
		fdHeader = shm_open(getHeaderFileName().c_str(), O_RDWR, S_IRUSR | S_IWUSR);
		if (fdHeader == -1 && errno == ENOENT) {
			// File did not exist: initialize header and memory file
			debug_print("Memory %s not found - creating\n", name.c_str());
			if (initialize(numberOfElements, elementSize) == FAILED) {
				release();
				return FAILED;
			}

			retval = CREATED_MEMORY;
		}
		else if (fdHeader != -1) {
			// File existed
			debug_print("Memory %s found - using existing\n", name.c_str());
			if ((fdMemory = shm_open(getMemoryFileName().c_str(), O_RDWR, S_IRUSR | S_IWUSR)) == -1) {
				release();
				perror("shm_open");
				return FAILED;
			}
			retval = FOUND_EXISTING_MEMORY;
		}
		else {
			// Failed to open for other reason
			debug_print("Memory %s failed to open\n", name.c_str());
			release();
			perror("shm_open");
			return FAILED;
		}

		debug_print("Mapping memory\n", nullptr);
		// Map header file into virtual memory space
		if ((info = static_cast<MemoryInformation*>(mmap(nullptr, sizeof (MemoryInformation), PROT_READ | PROT_WRITE, MAP_SHARED, fdHeader, 0))) == MAP_FAILED) {
			release();
			perror("mmap");
			return FAILED;
		}

		// If memory was created, after mapping it must be initialized with correct values
		if (retval == CREATED_MEMORY) {
			info->elementSize = elementSize;
			info->numberOfElements = numberOfElements;
		}
		else {
			elementSize = info->elementSize;
			numberOfElements = info->numberOfElements;
		}

		// Map memory file into virtual memory space
		if ((address = mmap(nullptr, elementSize == 0 || numberOfElements == 0 ?
							MEMORY_MINSIZE : elementSize * numberOfElements,
							PROT_READ | PROT_WRITE, MAP_SHARED,
							fdMemory, 0)) == MAP_FAILED) {
			release();
			perror("mmap");
			return FAILED;
		}

		// Store information on the memory map as well
		virtualMemoryInfo = *info;
		release();
		return retval;
	}

	MemoryReturnCode resize(unsigned int newNumberOfElements) {
		// Resize the system memory file


		if (ftruncate(fdMemory, newNumberOfElements == 0 ? MEMORY_MINSIZE : newNumberOfElements * getElementSize()) == -1) {
			perror("ftruncate");
			return FAILED;
		}
		else {
			info->numberOfElements = newNumberOfElements;
			// Remap the virtual memory space in case it is needed
			return refresh();
		}
	}

	MemoryReturnCode refresh() {
		void * newMemoryPointer = nullptr;
		newMemoryPointer = mremap(address, virtualMemoryInfo.elementSize == 0 || virtualMemoryInfo.numberOfElements == 0 ?
									  MEMORY_MINSIZE : virtualMemoryInfo.elementSize * virtualMemoryInfo.numberOfElements,
								  info->elementSize == 0 || info->numberOfElements == 0 ?
									  MEMORY_MINSIZE : info->elementSize * info->numberOfElements, MREMAP_MAYMOVE);

		if (newMemoryPointer == MAP_FAILED) {
			perror("mremap");
			return FAILED;
		}
		else {
			if (newMemoryPointer != address) {
				debug_print("Remapped memory %s\n", name.c_str());
				address = newMemoryPointer;
			}
			virtualMemoryInfo = *info;
			return SUCCEEDED;
		}
	}

private:
	sem_t* mutex = nullptr;							//!< Mutex synchronizing access to the memory
	int fdHeader = -1;								//!< File descriptor for the memory header
	int fdMemory = -1;								//!< File descriptor for the memory file
	MemoryInformation virtualMemoryInfo = {0, 0};	//!< Information on the virtual memory map
	MemoryInformation* info = nullptr;				//!< Information on the shared memory

	/*!
	 * \brief Creates the memory files necessary on the file system and ensures their size.
	 * \param initialElements Initial number of elements in the memory to be initialized.
	 * \param elementSize Size of the elements in the memory to be initialized.
	 * \return value according to ::MemoryReturnCode
	 */
	MemoryReturnCode initialize(const unsigned int initialElements, size_t elementSize) {
		if ((fdHeader = shm_open(getHeaderFileName().c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) == -1
				|| (fdMemory = shm_open(getMemoryFileName().c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) == -1){
			perror("shm_open");
			return FAILED;
		}
		if (ftruncate(fdHeader, sizeof (MemoryInformation)) == -1
				|| ftruncate(fdMemory, static_cast<ssize_t>(initialElements == 0 ? MEMORY_MINSIZE : initialElements * elementSize))) {
			perror("ftruncate");
			return FAILED;
		}
		return CREATED_MEMORY;
	}
};

/*********************************** STATIC VARIABLES ********************************************************************/
static vector<Memory> sharedMemoryBlocks;

/*********************************** STATIC FUNCTION DECLARATIONS ********************************************************/
static vector<Memory>::iterator getMemoryByAddress(volatile void* addr);

/*********************************** FUNCTION DEFINITIONS ****************************************************************/
/*!
 * \brief createSharedMemory Creates a file-backed memory space on the system and sets up virtual memory mappings in the
 *			calling process' address space. If a memory with the specified name already exists, the memory is not modified.
 *			Thus, the resulting memory may not actually be the size specified by the input arguments.
 * \param memoryName Name of the memory. Files will be created with endings ".mem", ".lck" and ".hdr" appended to this name
 * \param initialElements Desired initial number of elements if preexisting memory not found.
 * \param elementSize Size of the individual elements.
 * \param wasCreated Boolean value indicating if the memory was created (as opposed to a preexisting memory was found)
 * \return Pointer to the created memory.
 */
volatile void* createSharedMemory(const char* memoryName, const unsigned int initialElements,
								  const size_t elementSize, int *wasCreated) {

	size_t nameLength;

	// Initial sanity checks
	if (memoryName == nullptr) {
		errno = EINVAL;
		perror(__FUNCTION__);
		return nullptr;
	}

	nameLength = strlen(memoryName) + 1;
	if (nameLength > SHM_NAME_MAX) {
		errno = ENAMETOOLONG;
		perror(__FUNCTION__);
		return nullptr;
	}
	if (memoryName[0] == '/' && nameLength == 1) {
		errno = EINVAL;
		perror(__FUNCTION__);
		return nullptr;
	}

	// Check if specified memory has already been opened
	if (any_of(sharedMemoryBlocks.begin(), sharedMemoryBlocks.end(),
			   [memoryName](const Memory &mem){ return mem.name.find(memoryName) != string::npos; })) {
		errno = EEXIST;
		return nullptr;
	}

	debug_print("Emplacing new element in memory list\n", nullptr);
	sharedMemoryBlocks.emplace_back(memoryName);
	debug_print("Emplaced new element in memory list\n", nullptr);
	auto memory = sharedMemoryBlocks.end() - 1;

	debug_print("Setting up memory for %s\n", memory->name.c_str());
	switch (memory->setUp(initialElements, elementSize)) {
	case Memory::CREATED_MEMORY:
		if (wasCreated != nullptr) {
			*wasCreated = 1;
		}
		break;
	case Memory::FOUND_EXISTING_MEMORY:
		if (wasCreated != nullptr) {
			*wasCreated = 0;
		}
		break;
	default:
		if (wasCreated != nullptr) {
			*wasCreated = 0;
		}
		sharedMemoryBlocks.erase(memory);
		return nullptr;
	}

	return memory->address;
}

/*!
 * \brief getMemoryByAddress Fetches a pointer to an element in the shared memory vector
 *			which matches the specified address.
 * \param addr Address of the memory (in virtual memory space)
 * \return Pointer to a memory element, or end if none was found matching
 */
vector<Memory>::iterator getMemoryByAddress(volatile void* addr) {
	return find_if(sharedMemoryBlocks.begin(), sharedMemoryBlocks.end(),
				   [&](const Memory &mem){ return mem.address == addr; });
}

/*!
 * \brief destroySharedMemory Unlinks and unmaps all shared memory related to ::addr. The memory is
 *			destroyed and other processes using the same memory can no longer use it.
 * \param addr Address of the memory to be deallocated.
 */
void destroySharedMemory(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return;
	}

	// Save the names for unlinking after resource deallocation
	string lockFileName = mem->getLockFileName();
	string headerFileName = mem->getHeaderFileName();
	string memoryFileName = mem->getMemoryFileName();
	closeSharedMemory(addr);

	debug_print("Destroying memory %s\n", mem->name.c_str());
	if (shm_unlink(headerFileName.c_str()) == -1) {
		perror("shm_unlink");
	}
	if (shm_unlink(memoryFileName.c_str()) == -1) {
		perror("shm_unlink");
	}
	if (sem_unlink(lockFileName.c_str()) == -1) {
		perror("sem_unlink");
	}
}

/*!
 * \brief closeSharedMemory Unmaps all shared memory related to ::addr but leaves the resources on
 *			on the system.
 * \param addr Address of the memory to be deallocated.
 */
void closeSharedMemory(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return;
	}

	debug_print("Closing memory %s\n", mem->name.c_str());
	sharedMemoryBlocks.erase(mem);
}


/*!
 * \brief claimSharedMemory Claims access to the memory, restricting other processes from accessing the memory.
 *			If another process has already claimed the memory, this function blocks until access is available.
 * \param addr Address of the memory to be claimed.
 * \return Pointer to the memory, possibly updated.
 */
volatile void* claimSharedMemory(volatile void* addr) {

	auto mem = getMemoryByAddress(addr);

	if (mem != sharedMemoryBlocks.end()) {
		debug_print("Claiming memory %s\n", mem->name.c_str());
		mem->claim();
		debug_print("Claimed memory %s\n", mem->name.c_str());
		mem->refresh();
	}
	else {
		errno = ENOENT;
		perror(__FUNCTION__);
		return nullptr;
	}

	return mem->address;
}


/*!
 * \brief releaseSharedMemory Releases access to the memory, allowing other processes to access the memory
 * \param addr Address of the memory to be released.
 * \return Pointer to the memory.
 */
volatile void* releaseSharedMemory(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem != sharedMemoryBlocks.end()) {
		mem->release();
		debug_print("Released memory %s\n", mem->name.c_str());
	}
	else {
		errno = ENOENT;
		perror(__FUNCTION__);
		return nullptr;
	}
	return mem->address;
}


/*!
 * \brief resizeSharedMemory Modifies the size of shared memory. If the memory is expanded the new
 *			area is initialized with 0. If the memory is reduced in size the end is truncated.
 *			The memory should be claimed before this operation is performed.
 * \param addr Address of the memory to be modified.
 * \param newNumberOfElements Desired number of memory elements after update.
 * \return Pointer to the memory, possibly updated. Returns NULL on failure.
 */
volatile void* resizeSharedMemory(volatile void* addr, const unsigned int newNumberOfElements) {

	auto mem = getMemoryByAddress(addr);

	// Sanity checks
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return nullptr;
	}

	if (mem->resize(newNumberOfElements) == Memory::FAILED) {
		return nullptr;
	}
	return mem->address;
}


/*!
 * \brief getNumberOfMemoryElements Returns the number of memory elements for a shared memory. The memory
 *			should be claimed before this operation to ensure relevance of the return value.
 * \param addr Address of the memory
 * \return Number of elements in memory, or -1 if address invalid
 */
int getNumberOfMemoryElements(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return -1;
	}
	return mem->getNumberOfElements();
}


/*!
 * \brief getMemorySize Returns the total size of the shared memory.
 * \param addr Address of the shared memory.
 * \return Size of the memory, or -1 on error
 */
ssize_t getMemorySize(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return -1;
	}
	return mem->getTotalSize();
}


/*!
 * \brief getElementSize Returns the number of elements in the shared memory.
 * \param addr Address of the shared memory.
 * \return Number of elements in the memory, or -1 on error
 */
ssize_t getElementSize(volatile void* addr) {
	auto mem = getMemoryByAddress(addr);
	if (mem == sharedMemoryBlocks.end()) {
		errno = ENOENT;
		perror(__FUNCTION__);
		return -1;
	}
	return mem->getElementSize();
}
