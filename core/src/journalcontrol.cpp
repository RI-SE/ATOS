/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2020 AstaZero
  ------------------------------------------------------------------------------
  -- File        : journalcontrol.cpp
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#include <signal.h>
#include <vector>
#include <unordered_set>
#include <set>
#include <fstream>
#include <iostream>
#include <iterator>
#include <chrono>
#include <algorithm>
// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include "journalcontrol.h"
#include "journal.h"
#include "datadictionary.h"

#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
namespace fs = std::filesystem;
#else
namespace fs = std::experimental::filesystem;
#endif

// Add a C++20 type
namespace std::chrono {
	typedef duration<int64_t, ratio<60*60*24>> days;
}

#define DATE_STRING_MAX_LEN 20

#define MODULE_NAME "JournalControl"
class Journal {
public:
	class Bookmark {
	private:
		fs::path filePath;
		std::streampos filePosition;
	public:
		bool valid = false;
		void place(const fs::path &path, const bool placeAtBeginning=false) {
			std::ifstream istrm(path, placeAtBeginning ? std::ios_base::in
													   : std::ios_base::ate);
			if (istrm.is_open()) {
				LogMessage(LOG_LEVEL_DEBUG, "Storing bookmark to file %s", path.c_str());
				filePosition = istrm.tellg();
				filePath = path;
				valid = true;
				istrm.close();
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Unable to open file %s", path.c_str());
			}
		}
		const std::streampos& getPosition() const { return filePosition; }
		const fs::path& getFilePath() const { return filePath; }
	};

	Journal() {}
	std::string moduleName;
	mutable Bookmark startReference;
	mutable Bookmark stopReference;
	mutable std::set<fs::path> containedFiles;

	bool operator==(const Journal &other) const {
		return this->moduleName == other.moduleName;
	}
};



//! Template specialization of std::hash for Journal
namespace std {
	template <> struct hash<Journal> {
		size_t operator() (const Journal &journal) const {
			return std::hash<std::string>{}(journal.moduleName);
		}
	};
}

class JournalCollection : public std::unordered_set<Journal> {
public:
	void placeStartBookmarks();
	void placeStopBookmarks();
	void insertNonBookmarked();
	int dumpToFile();
private:
	std::chrono::time_point<std::chrono::system_clock, std::chrono::days> startDay;
	std::chrono::time_point<std::chrono::system_clock, std::chrono::days> stopDay;
};

/*------------------------------------------------------------
  -- Static variables.
  ------------------------------------------------------------*/
static volatile bool quit = false;

/*------------------------------------------------------------
  -- Static function declarations.
  ------------------------------------------------------------*/
static void signalHandler(int signo);
static int initializeModule(const LOG_LEVEL logLevel);
static std::vector<fs::path> getJournalFilesFrom(const std::chrono::system_clock::time_point &date);
static std::vector<fs::path> getJournalFilesFromToday(void);
static std::string getCurrentDateAsString(void);
static std::string getDateAsString(const std::chrono::system_clock::time_point &date);
static int printFilesTo(const fs::path &inputDirectory, std::ostream &outputFile);
static int printJournalHeaderTo(std::ofstream &ostrm);

/*------------------------------------------------------------
  -- Main task.
  ------------------------------------------------------------*/
void journalcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	std::vector<char> mqReceiveBuffer(MBUS_MAX_DATALEN, 0);
	std::vector<char> mqSendBuffer(MBUS_MAX_DATALEN, 0);
	enum COMMAND command = COMM_INV;
	ssize_t receivedBytes = 0;
	struct timeval recvTime;

	JournalCollection journals;

	// Initialize
	if (initializeModule(logLevel) < 0) {
		util_error("Failed to initialize module");
	}


	while (!quit) {
		std::fill(mqReceiveBuffer.begin(), mqReceiveBuffer.end(), 0);
		receivedBytes = iCommRecv(&command, mqReceiveBuffer.data(), mqReceiveBuffer.size(), &recvTime);

		switch (command) {
		case COMM_ARM:
			// Save start references
			journals.placeStartBookmarks();
			break;
		case COMM_STOP:
		case COMM_ABORT:
			// Temporary: Treat ABORT as stop signal
			// Save stop references
			journals.placeStopBookmarks();
			// If any additional journals were created in the start-stop interval,
			// insert them
			journals.insertNonBookmarked();
			// Merge journals into named output
			journals.dumpToFile();
			break;

        case COMM_GETSTATUS: {
			std::fill(mqSendBuffer.begin(), mqSendBuffer.end(), 0);
            unsigned long startTime = UtilGetPIDUptime(getpid()).tv_sec;
            snprintf(mqSendBuffer.data(), mqSendBuffer.size(), "%s:%lu", MODULE_NAME, startTime);
			mqSendBuffer.back() = '\0';
			if (iCommSend(COMM_GETSTATUS_OK, mqSendBuffer.data(), mqSendBuffer.size()) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending status reply");
			}
			break;
        }
		case COMM_REPLAY:
			LogMessage(LOG_LEVEL_WARNING, "Replay function out of date");
			break;
		case COMM_EXIT:
			quit = true;
			break;
		// Do nothing with these messages
		case COMM_GETSTATUS_OK:
			break;
		}
	}

	iCommClose();

	LogMessage(LOG_LEVEL_INFO, MODULE_NAME " exiting");
}


/*!
 * \brief placeStartBookmarks Stores references to the current end of file of all journals.
 */
void JournalCollection::placeStartBookmarks() {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	this->clear();

	for (const auto &journalFile : journalFilesFromToday) {
		LogMessage(LOG_LEVEL_DEBUG, "Storing start bookmark in file %s", journalFile.c_str());
		Journal journal;
		auto datePosition = journalFile.stem().string().find("-" + currentDate);
		journal.moduleName = journalFile.stem().string().substr(0, datePosition);
		journal.startReference.place(journalFile);
		journal.containedFiles.insert(journalFile);
		this->insert(journal);
	}
	using namespace std::chrono;
	this->startDay = floor<days>(system_clock::now());
}


/*!
 * \brief placeStopBookmarks Stores references to the current end of file of all journals.
 */
void JournalCollection::placeStopBookmarks() {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	for (const auto &journalFile : journalFilesFromToday) {
		LogMessage(LOG_LEVEL_DEBUG, "Storing stop bookmark in file %s", journalFile.c_str());
		// Find existing journal matching module name from file
		Journal soughtJournal;
		auto datePosition = journalFile.stem().string().find("-" + currentDate);
		soughtJournal.moduleName = journalFile.stem().string().substr(0, datePosition);
		soughtJournal.startReference.place(journalFile, true); // In case we are emplacing, startReference at beginning
		// Insert checks if an equivalent element (same module) already exists
		auto matchingJournal = this->insert(soughtJournal).first;
		matchingJournal->stopReference.place(journalFile);
		matchingJournal->containedFiles.insert(journalFile);
	}
	using namespace std::chrono;
	this->stopDay = floor<days>(system_clock::now());
}


/*!
 * \brief JournalCollection::insertNonBookmarked Inserts journal files that were not included
 *			among the start/stop bookmarks for some reason, e.g. that the start/stop bookmarks
 *			are separated by more than a day, or that the journal file did not extend to the
 *			stop bookmark day.
 */
void JournalCollection::insertNonBookmarked() {

	if (startDay.time_since_epoch().count() == 0) {
		LogMessage(LOG_LEVEL_ERROR, "Start time invalid, unable to insert non-bookmarked journals");
		return;
	}
	if (stopDay.time_since_epoch().count() == 0) {
		LogMessage(LOG_LEVEL_ERROR, "Stop time invalid, unable to insert non-bookmarked journals");
		return;
	}
	if (stopDay.time_since_epoch().count() < startDay.time_since_epoch().count()) {
		LogMessage(LOG_LEVEL_ERROR, "Stop time is before start time, unable to insert non-bookmarked journals");
		return;
	}

	// For each spanned day, check if any journal files exist that are not
	// known to this object. If so, insert them, and if the start reference
	// is not specified, place it at beginning of the first file found.
	for (auto day = startDay; day <= stopDay; day += std::chrono::days(1)) {
		auto journalFilesFromDay = getJournalFilesFrom(day);
		auto date = getDateAsString(day);

		for (const auto &journalFile : journalFilesFromDay) {
			// Find existing journal matching module name from file
			Journal soughtJournal;
			auto datePosition = journalFile.stem().string().find("-" + date);
			soughtJournal.moduleName = journalFile.stem().string().substr(0, datePosition);
			// Insert checks if an equivalent element (same module) already exists
			auto matchingJournal = insert(soughtJournal).first;
			if (matchingJournal->containedFiles.insert(journalFile).second) {
				LogMessage(LOG_LEVEL_DEBUG, "Inserted non-bookmarked file %s",
						   journalFile.c_str());
			}
			if (!matchingJournal->startReference.valid) {
				LogMessage(LOG_LEVEL_DEBUG, "Storing start bookmark at beginning of file %s",
						   journalFile.c_str());
				matchingJournal->startReference.place(journalFile, true);
			}
		}
	}

	// For each journal, loop backward over the contained files to insert a
	// stop reference at the end of the file with the highest date.
	for (auto journal : *this) {
		if (!journal.stopReference.valid) {
			for (auto day = stopDay; day >= startDay; day -= std::chrono::days(1)) {
				auto date = getDateAsString(day);
				auto isFromDate = [date](const fs::path& filePath) {
					return filePath.filename().string().find(date) != std::string::npos;
				};
				auto matchingFile = std::find_if(journal.containedFiles.begin(),
												 journal.containedFiles.end(), isFromDate);
				if (matchingFile != journal.containedFiles.end()) {
					LogMessage(LOG_LEVEL_DEBUG, "Storing end bookmark at end of file %s",
							   matchingFile->c_str());
					journal.stopReference.place(*matchingFile);
					break;
				}
			}
		}
	}
}

/*!
 * \brief dumpToFile Generates a merged file based on input journals
 *			with ascending timestamps. The output journal file is created by
 *			interleaving relevant entries from all contained journals.
 * \return 0 on success, -1 otherwise
 */
int JournalCollection::dumpToFile() {

	int retval = 0;
	char scenarioName[PATH_MAX] = {'\0'};
	char journalDir[PATH_MAX] = {'\0'};

	// Construct output file name and path
	if (DataDictionaryGetScenarioName(scenarioName, sizeof (scenarioName)) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to get scenario name parameter to generate output file");
		return -1;
	}
	UtilGetJournalDirectoryPath(journalDir, sizeof (journalDir));
	fs::path journalDirPath(std::string(journalDir) + std::string(scenarioName) + JOURNAL_FILE_ENDING);

	std::ofstream ostrm(journalDirPath);
	if (!ostrm.is_open()) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open %s for writing", journalDirPath.c_str());
		return -1;
	}

	if (printJournalHeaderTo(ostrm) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Error writing journal header");
		ostrm.close();
		return -1;
	}

	/*!
	 * \brief The JournalFileSection struct is used to keep track of reading one
	 *			file section which is part of a journal.
	 */
	struct JournalFileSection {
		fs::path path;			//!< Path to the file referred to
		std::ifstream istrm;	//!< Input stream for accessing the file
		std::streampos beg;		//!< First character index of relevant section
		std::streampos end;		//!< Last character index of relevant section
		std::string lastRead;	//!< Last read string from ::istrm member
		//! The < operator tells which of two is oldest at its last read line
		bool operator< (const JournalFileSection &other) const {
			std::istringstream strThis(lastRead), strOther(other.lastRead);
			double timeThis = 0.0, timeOther = 0.0;
			strThis >> timeThis;
			strOther >> timeOther;
			return timeThis < timeOther;
		}
	};

	// Fill a vector with all files pertaining to recorded data,
	// along with the correct start and stop references.
	// After this, the vector contains opened streams positioned
	// for reading at the line closest to the start time.
	std::vector<JournalFileSection> inputFiles;
	for (const auto &journal : *this) {
		for (const fs::path &file : journal.containedFiles) {
			JournalFileSection &section = inputFiles.emplace_back();
			section.path = file;
			section.istrm.open(file);
			if (section.istrm.is_open()) {
				section.beg = file == journal.startReference.filePath ?
							journal.startReference.filePosition
						  : section.istrm.tellg();
				section.istrm.seekg(std::ios_base::end);
				section.end = file == journal.stopReference.filePath ?
							journal.stopReference.filePosition
						  : section.istrm.tellg();
				section.istrm.seekg(section.beg);
				LogMessage(LOG_LEVEL_DEBUG, "File %s contained %d characters of journal data from period of interest",
						section.path.filename().c_str(), section.end-section.beg);
				if (section.beg == section.end || !std::getline(section.istrm, section.lastRead)) {
					section.istrm.close();
					inputFiles.pop_back();
				}
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Unable to open %s for reading", file.c_str());
				inputFiles.pop_back();
				retval = -1;
			}
		}
	}

	// Each iteration, transfer the line starting with the oldest timestamp to the output file and read the next
	while(!inputFiles.empty()) {
		std::vector<JournalFileSection>::iterator oldestFile = std::min_element(inputFiles.begin(), inputFiles.end());
		ostrm << oldestFile->lastRead << std::endl;
		if (!std::getline(oldestFile->istrm, oldestFile->lastRead)
				|| oldestFile->istrm.tellg() > oldestFile->end) {
			LogMessage(LOG_LEVEL_DEBUG, "Reached end of journal file %s", oldestFile->path.c_str());
			oldestFile->istrm.close();
			inputFiles.erase(oldestFile);
		}
	}

	ostrm.close();
	LogMessage(LOG_LEVEL_INFO, "Generated output journal %s", journalDirPath.stem().c_str());
	return retval;
}

/*!
 * \brief getCurrentDateAsString Creates a string on the format YYYY-MM-DD of the current date.
 * \return A std::string containing the current date
 */
std::string getCurrentDateAsString() {
	return getDateAsString(std::chrono::system_clock::now());
}

/*!
 * \brief getDateAsString Creates a string on the format YYYY-MM-DD of the specified date.
 * \param date Timestamp for which date string is to be extracted.
 * \return A std::string containing the date representation
 */
std::string getDateAsString(const std::chrono::system_clock::time_point &date) {
	using Clock = std::chrono::system_clock;
	char dateString[DATE_STRING_MAX_LEN] = {'\0'};
	auto dateRaw = Clock::to_time_t(date);
	auto dateStructPtr = std::localtime(&dateRaw);
	std::strftime(dateString, DATE_STRING_MAX_LEN, "%Y-%m-%d", dateStructPtr);
	return dateString;
}

int printJournalHeaderTo(std::ofstream &ostrm) {
	std::vector<char> trajectoryDirectory(PATH_MAX, '\0');
	std::vector<char> configurationDirectory(PATH_MAX, '\0');
	std::vector<char> objectDirectory(PATH_MAX, '\0');
	std::ifstream istrm;
	fs::path fileDirectory;

	ostrm << "------------------------------------------" << std::endl;
	ostrm << "Whole object files" << std::endl;
	ostrm << "------------------------------------------" << std::endl;

	UtilGetObjectDirectoryPath(objectDirectory.data(), objectDirectory.size());
	objectDirectory.erase(std::find(objectDirectory.begin(), objectDirectory.end(), '\0'),
							  objectDirectory.end());
	std::remove(std::find(objectDirectory.begin(), objectDirectory.end(), '\0'),
				objectDirectory.end(), '\0');
	fileDirectory.assign(objectDirectory.begin(), objectDirectory.end());

	if (printFilesTo(fileDirectory, ostrm) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to print object files to log - terminating log generation");
		return -1;
	}

	ostrm << "------------------------------------------" << std::endl;
	ostrm << "Whole trajectory files" << std::endl;
	ostrm << "------------------------------------------" << std::endl;

	UtilGetTrajDirectoryPath(trajectoryDirectory.data(), trajectoryDirectory.size());
	trajectoryDirectory.erase(std::find(trajectoryDirectory.begin(), trajectoryDirectory.end(), '\0'),
							  trajectoryDirectory.end());
	std::remove(std::find(trajectoryDirectory.begin(), trajectoryDirectory.end(), '\0'),
				trajectoryDirectory.end(), '\0');
	fileDirectory.assign(trajectoryDirectory.begin(), trajectoryDirectory.end());

	if (printFilesTo(fileDirectory, ostrm) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to print trajectory files to log - terminating log generation");
		return -1;
	}

	ostrm << "------------------------------------------" << std::endl;
	ostrm << "Whole configuration files" << std::endl;
	ostrm << "------------------------------------------" << std::endl;

	UtilGetConfDirectoryPath(configurationDirectory.data(), configurationDirectory.size());
	configurationDirectory.erase(std::find(configurationDirectory.begin(), configurationDirectory.end(), '\0'),
								 configurationDirectory.end());
	std::remove(std::find(configurationDirectory.begin(), configurationDirectory.end(), '\0'),
				configurationDirectory.end(), '\0');
	fileDirectory.assign(configurationDirectory.begin(), configurationDirectory.end());

	if (printFilesTo(fileDirectory, ostrm) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to print configuration files to log - terminating log generation");
		return -1;
	}

	// TODO: information about file structure
	return 0;
}

/*!
 * \brief printFilesTo Prints the contents of all files in chosen directory to the
 *			selected stream
 * \param inputDirectory Directory containing files to be printed
 * \param outputFile Stream to which contents are to be printed
 * \return 0 on success, -1 otherwise
 */
int printFilesTo(const fs::path &inputDirectory, std::ostream &outputFile) {

	if (!exists(inputDirectory)) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to find directory %s", inputDirectory.c_str());
		return -1;
	}
	for (const auto &dirEntry : fs::directory_iterator(inputDirectory)) {
		if (fs::is_regular_file(dirEntry.status())) {
			const auto inputFile = dirEntry.path();
			std::ifstream istrm(inputFile.string());
			if (!istrm.is_open()) {
				LogMessage(LOG_LEVEL_ERROR, "Unable to open file %s", inputFile.c_str());
				return -1;
			}
			istrm.unsetf(std::ios_base::skipws);
			for (std::istream_iterator<char> it(istrm); it != std::istream_iterator<char>(); ++it) {
				outputFile << *it;
			}
			outputFile << std::endl;
			istrm.close();
		}
	}
	return 0;
}

/*!
 * \brief getJournalFilesFromToday Fetches a list of file paths corresponding to journal
 *			files created on the current date.
 * \return A std::vector containing file paths
 */
std::vector<fs::path> getJournalFilesFromToday() {
	return getJournalFilesFrom(std::chrono::system_clock::now());
}


/*!
 * \brief getJournalFilesFrom Fetches a list of file paths corresponding to journal
 *			files created on the specified date.
 * \param date Date for which journals are to be fetched
 * \return A std::vector containing file paths
 */
std::vector<fs::path> getJournalFilesFrom(const std::chrono::system_clock::time_point &date) {
	std::vector<fs::path> journalsFromDate;
	std::vector<char> buffer(PATH_MAX, '\0');

	UtilGetJournalDirectoryPath(buffer.data(), buffer.size());
	buffer.erase(std::find(buffer.begin(), buffer.end(), '\0'),
								 buffer.end());
	fs::path journalDirPath(buffer.begin(), buffer.end());

	if (!exists(journalDirPath)) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to find journal directory %s", journalDirPath.string().c_str());
		return journalsFromDate;
	}

	auto dateString = getDateAsString(date);

	for (const auto &dirEntry : fs::directory_iterator(journalDirPath)) {
		if (fs::is_regular_file(dirEntry.status())
				&& dirEntry.path().extension().string().find(JOURNAL_FILE_ENDING) != std::string::npos) {
			// Check if file contains current date
			size_t pos =  dirEntry.path().string().find(dateString);
			if (pos != std::string::npos) {
				journalsFromDate.push_back(dirEntry.path());
			}
		}
	}
	return journalsFromDate;
}


void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		quit = true;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}

/*!
 * \brief initializeModule Initializes this module by creating log, connecting to the message queue bus,
 *			setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;
	struct timespec sleepTimePeriod, remTime;
	sleepTimePeriod.tv_sec = 0;
	sleepTimePeriod.tv_nsec = 1000000;
	int maxRetries = 10, retryNumber;

	// Initialize log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, MODULE_NAME " task running with PID: %d", getpid());

	// Set up signal handlers
	LogMessage(LOG_LEVEL_DEBUG, "Initializing signal handler");
	if (signal(SIGINT, signalHandler) == SIG_ERR) {
		perror("signal");
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to initialize signal handler");
	}

	// Initialize message bus connection
	LogMessage(LOG_LEVEL_DEBUG, "Initializing connection to message bus");
	for (retryNumber = 0; iCommInit() != 0 && retryNumber < maxRetries; ++retryNumber) {
		nanosleep(&sleepTimePeriod, &remTime);
	}
	if (retryNumber == maxRetries) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to initialize connection to message bus");
	}

	return retval;
}
