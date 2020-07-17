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

namespace fs = std::experimental::filesystem;

#define MODULE_NAME "JournalControl"
class Journal {
public:
	class Bookmark {
	public:
		fs::path filePath;
		std::streampos filePosition;
		bool valid = false;
		void place(const fs::path &path, const std::ios_base::seekdir &pos) {
			std::ifstream istrm(path, pos == std::ios_base::end ? std::ios_base::ate
																		 : std::ios_base::in);
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
/*------------------------------------------------------------
  -- Static variables.
  ------------------------------------------------------------*/
static volatile bool quit = false;

/*------------------------------------------------------------
  -- Static function declarations.
  ------------------------------------------------------------*/
static void signalHandler(int signo);
static int initializeModule(const LOG_LEVEL logLevel);
static void storeJournalStartBookmarks(std::unordered_set<Journal> &journals);
static void storeJournalStopBookmarks(std::unordered_set<Journal> &journals);
static int generateOutputJournal(std::unordered_set<Journal> &journals);
static std::vector<fs::path> getJournalFilesFromToday(void);
static std::string getCurrentDateAsString(void);

/*------------------------------------------------------------
  -- Main task.
  ------------------------------------------------------------*/
void journalcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	std::vector<char> mqReceiveBuffer(MBUS_MAX_DATALEN, 0);
	std::vector<char> mqSendBuffer(MBUS_MAX_DATALEN, 0);
	enum COMMAND command = COMM_INV;
	ssize_t receivedBytes = 0;
	struct timeval recvTime;

	std::unordered_set<Journal> journals;

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
			storeJournalStartBookmarks(journals);
			break;
		case COMM_STOP:
		case COMM_ABORT:
			// Temporary: Treat ABORT as stop signal
			// Save stop references
			storeJournalStopBookmarks(journals);
			// Merge journals into named output
			generateOutputJournal(journals);
			break;

		case COMM_GETSTATUS:
			std::fill(mqSendBuffer.begin(), mqSendBuffer.end(), 0);
			snprintf(mqSendBuffer.data(), mqSendBuffer.size(), "%s", MODULE_NAME);
			mqSendBuffer.back() = '\0';
			if (iCommSend(COMM_GETSTATUS_OK, mqSendBuffer.data(), mqSendBuffer.size()) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending status reply");
			}
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


/*!
 * \brief storeJournalStartBookmarks Stores references to the current end of file of all journals.
 * \param journals Set of journals in which start bookmarks are to be saved
 */
void storeJournalStartBookmarks(std::unordered_set<Journal> &journals) {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	journals.clear();

	for (const auto &journalFile : journalFilesFromToday) {
		Journal journal;
		auto datePosition = journalFile.stem().string().find("-" + currentDate);
		journal.moduleName = journalFile.stem().string().substr(0, datePosition);
		journal.startReference.place(journalFile, std::ios_base::end);
		journal.containedFiles.insert(journalFile);
		journals.insert(journal);
	}
}


/*!
 * \brief storeJournalStopBookmarks Stores references to the current end of file of all journals.
 * \param journals Set of journals in which stop bookmarks are to be saved
 */
void storeJournalStopBookmarks(std::unordered_set<Journal> &journals) {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	for (const auto &journalFile : journalFilesFromToday) {
		// Find existing journal matching module name from file
		Journal soughtJournal;
		auto datePosition = journalFile.stem().string().find("-" + currentDate);
		soughtJournal.moduleName = journalFile.stem().string().substr(0, datePosition);
		soughtJournal.startReference.place(journalFile, std::ios_base::beg);
		// Insert checks if an equivalent element (same module) already exists
		auto matchingJournal = journals.insert(soughtJournal).first;
		matchingJournal->stopReference.place(journalFile, std::ios_base::end);
		matchingJournal->containedFiles.insert(journalFile);
	}

	// Handle the journals for which no file existed from today
	for (auto &journal : journals) {
		if (!journal.stopReference.valid) {
			journal.stopReference.place(journal.startReference.filePath, std::ios_base::end);
		}
	}
}


/*!
 * \brief generateOutputJournal Generates a merged file based on input journals
 *			with ascending timestamps. The output journal file is created by
 *			interleaving relevant entries from all input journals.
 * \param journals Set of journals in which data can be found
 * \return 0 on success, -1 otherwise
 */
int generateOutputJournal(std::unordered_set<Journal> &journals) {

	int retval = 0;
	std::string scenarioName(PATH_MAX, '\0');
	std::string journalDir(PATH_MAX, '\0');

	// Construct output file name and path
	if (DataDictionaryGetScenarioName(scenarioName.data(), scenarioName.size()) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to get scenario name parameter to generate output file");
		return -1;
	}
	UtilGetJournalDirectoryPath(journalDir.data(), journalDir.size());
	scenarioName.resize(scenarioName.find_first_of('\0'));
	journalDir.resize(journalDir.find_first_of('\0'));
	fs::path journalDirPath(journalDir + scenarioName + JOURNAL_FILE_ENDING);

	std::ofstream ostrm(journalDirPath);
	if (!ostrm.is_open()) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open %s for writing", journalDirPath.c_str());
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
	for (const auto &journal : journals) {
		for (const auto &file : journal.containedFiles) {
			auto &section = inputFiles.emplace_back();
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
				LogMessage(LOG_LEVEL_DEBUG, "File %s contained %d characters of log data from period of interest",
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
			LogMessage(LOG_LEVEL_DEBUG, "Reached end of log file %s", oldestFile->path.filename().c_str());
			oldestFile->istrm.close();
			inputFiles.erase(oldestFile);
		}
	}

	ostrm.close();
	return retval;
}

/*!
 * \brief getCurrentDateAsString Creates a string on the format YYYY-MM-DD of the current date.
 * \return A std::string containing the current date
 */
std::string getCurrentDateAsString() {
	using Clock = std::chrono::system_clock;
	std::string currentDate(12, '\0');
	auto now = Clock::to_time_t(Clock::now());
	auto now_tm = std::localtime(&now);
	std::strftime(currentDate.data(), currentDate.size(), "%Y-%m-%d", now_tm);
	currentDate.resize(currentDate.find_first_of('\0'));
	return currentDate;
}


/*!
 * \brief getJournalFilesFromToday Fetches a list of file paths corresponding to journal
 *			files created on the current date.
 * \return A std::vector containing file paths
 */
std::vector<fs::path> getJournalFilesFromToday() {
	std::vector<fs::path> journalsFromToday;
	std::string buffer(PATH_MAX, '\0');
	UtilGetJournalDirectoryPath(buffer.data(), buffer.size());
	buffer.resize(buffer.find_first_of('\0'));
	fs::path journalDirPath(buffer);
	if (!exists(journalDirPath)) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to find journal directory %s", journalDirPath.string().c_str());
		return journalsFromToday;
	}

	auto currentDate = getCurrentDateAsString();

	for (const auto &dirEntry : fs::directory_iterator(journalDirPath)) {
		if (fs::is_regular_file(dirEntry.status())
				&& dirEntry.path().extension().string().find(JOURNAL_FILE_ENDING) != std::string::npos) {
			// Check if file contains current date
			size_t pos =  dirEntry.path().string().find(currentDate);
			if (pos != std::string::npos) {
				journalsFromToday.push_back(dirEntry.path());
			}
		}
	}
	return journalsFromToday;
}
