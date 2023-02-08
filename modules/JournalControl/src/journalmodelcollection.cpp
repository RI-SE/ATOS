#include "journalmodelcollection.hpp"
#include "datadictionary.h"
#include "journal.hpp"
#include <fstream>
#include <iterator>
#include <algorithm>


#define DATE_STRING_MAX_LEN 20

/*!
 * \brief placeStartBookmarks Stores references to the current end of file of all journals.
 */
void JournalModelCollection::placeStartBookmarks() {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	this->clear();

	for (const auto &journalFile : journalFilesFromToday) {
		RCLCPP_DEBUG(get_logger(), "Storing start bookmark in file %s", journalFile.c_str());
		JournalModel journal(get_logger());
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
void JournalModelCollection::placeStopBookmarks() {

	auto journalFilesFromToday = getJournalFilesFromToday();
	auto currentDate = getCurrentDateAsString();

	for (const auto &journalFile : journalFilesFromToday) {
		RCLCPP_DEBUG(get_logger(), "Storing stop bookmark in file %s", journalFile.c_str());
		// Find existing journal matching module name from file
		JournalModel soughtJournal(get_logger());
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
 * \brief JournalModelCollection::insertNonBookmarked Inserts journal files that were not included
 *			among the start/stop bookmarks for some reason, e.g. that the start/stop bookmarks
 *			are separated by more than a day, or that the journal file did not extend to the
 *			stop bookmark day.
 */
void JournalModelCollection::insertNonBookmarked() {

	if (startDay.time_since_epoch().count() == 0) {
		RCLCPP_ERROR(get_logger(), "Start time invalid, unable to insert non-bookmarked journals");
		return;
	}
	if (stopDay.time_since_epoch().count() == 0) {
		RCLCPP_ERROR(get_logger(), "Stop time invalid, unable to insert non-bookmarked journals");
		return;
	}
	if (stopDay.time_since_epoch().count() < startDay.time_since_epoch().count()) {
		RCLCPP_ERROR(get_logger(), "Stop time is before start time, unable to insert non-bookmarked journals");
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
			JournalModel soughtJournal(get_logger());
			auto datePosition = journalFile.stem().string().find("-" + date);
			soughtJournal.moduleName = journalFile.stem().string().substr(0, datePosition);
			// Insert checks if an equivalent element (same module) already exists
			auto matchingJournal = insert(soughtJournal).first;
			if (matchingJournal->containedFiles.insert(journalFile).second) {
				RCLCPP_DEBUG(get_logger(), "Inserted non-bookmarked file %s",
						   journalFile.c_str());
			}
			if (!matchingJournal->startReference.valid) {
				RCLCPP_DEBUG(get_logger(), "Storing start bookmark at beginning of file %s",
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
					RCLCPP_DEBUG(get_logger(), "Storing end bookmark at end of file %s",
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
int JournalModelCollection::dumpToFile() {

	int retval = 0;
	char scenarioName[PATH_MAX] = {'\0'};
	char journalDir[PATH_MAX] = {'\0'};

	// Construct output file name and path
	if (DataDictionaryGetScenarioName(scenarioName, sizeof (scenarioName)) != READ_OK) {
		RCLCPP_ERROR(get_logger(), "Unable to get scenario name parameter to generate output file");
		return -1;
	}
	UtilGetJournalDirectoryPath(journalDir, sizeof (journalDir));
	fs::path journalDirPath(std::string(journalDir) + std::string(scenarioName) + JOURNAL_FILE_ENDING);

	std::ofstream ostrm(journalDirPath);
	if (!ostrm.is_open()) {
		RCLCPP_ERROR(get_logger(), "Unable to open %s for writing", journalDirPath.c_str());
		return -1;
	}

	try {
		printJournalHeaderTo(ostrm);
	}
	catch (const std::runtime_error& e) {
		RCLCPP_ERROR(get_logger(), "Unable to write journal header to %s: %s",
				   journalDirPath.c_str(), e.what());
		ostrm.close();
		return -1;
	}

	/*!
	 * \brief The JournalFileSection struct is used to keep track of reading one
	 *			file section which is part of a journal.
	 */
	struct JournalFileSection {
		fs::path path;				//!< Path to the file referred to
		std::ifstream istrm;		//!< Input stream for accessing the file
		std::streampos beg;			//!< First character index of relevant section
		std::streampos end;			//!< Last character index of relevant section
		std::string lastRead;		//!< Last read string from ::istrm member
		unsigned int nReadRows = 0;	//!< Number of rows read from ::istrm member
		//! The < operator tells which of two is oldest at its last read line
		bool operator< (const JournalFileSection &other) const {
			std::istringstream strThis(lastRead), strOther(other.lastRead);
			double timeThis = 0.0, timeOther = 0.0;
			strThis >> timeThis;
			strOther >> timeOther;
			return timeThis < timeOther;
		}
	};

	RCLCPP_INFO(get_logger(), "Creating output log for journals\n%s", this->toString().c_str());
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
				RCLCPP_DEBUG(get_logger(), "Opened file %s", file.c_str());
				if (file == journal.startReference.getFilePath()) {
					section.beg = journal.startReference.getPosition();
				}
				else {
					section.istrm.seekg(0, section.istrm.beg);
					section.beg = section.istrm.tellg();
				}

				if (file == journal.stopReference.getFilePath()) {
					section.end = journal.stopReference.getPosition();
				}
				else {
					section.istrm.seekg(0, section.istrm.end);
					section.end = section.istrm.tellg();
				}
				section.istrm.seekg(section.beg);

				if (section.end - section.beg < 0) {
					RCLCPP_ERROR(get_logger(), "End precedes beginning in file %s: beg @%ld, end @%ld",
							   file.c_str(), section.beg, section.end);
					section.istrm.close();
					inputFiles.pop_back();
				}
				else if (section.beg == section.end) {
					RCLCPP_DEBUG(get_logger(), "No data found");
					section.istrm.close();
					inputFiles.pop_back();
				}
				else if (!std::getline(section.istrm, section.lastRead)) {
					RCLCPP_DEBUG(get_logger(), "Failed to read line");
					section.istrm.close();
					inputFiles.pop_back();
				}
				else {
					section.nReadRows++;
				}
			}
			else {
				RCLCPP_ERROR(get_logger(), "Unable to open %s for reading", file.c_str());
				inputFiles.pop_back();
				retval = -1;
			}
		}
	}

	// Each iteration, transfer the line starting with the oldest timestamp to the output file and read the next
	while(!inputFiles.empty()) {
		std::vector<JournalFileSection>::iterator oldestFile = std::min_element(inputFiles.begin(), inputFiles.end());
		ostrm << oldestFile->lastRead << std::endl;
		if (!std::getline(oldestFile->istrm, oldestFile->lastRead)) {
			RCLCPP_DEBUG(get_logger(), "Reached end of journal file %s, read %u rows",
					   oldestFile->path.c_str(), oldestFile->nReadRows);
			oldestFile->istrm.close();
			inputFiles.erase(oldestFile);
		}
		else if (oldestFile->nReadRows > oldestFile->end - oldestFile->beg) {
			RCLCPP_DEBUG(get_logger(), "Read %u rows from journal file %s",
					   oldestFile->nReadRows, oldestFile->path.c_str());
			oldestFile->istrm.close();
			inputFiles.erase(oldestFile);
		}
		else {
			oldestFile->nReadRows++;
		}
	}

	ostrm.close();
	RCLCPP_INFO(get_logger(), "Generated output journal %s", journalDirPath.stem().c_str());
	return retval;
}



int JournalModelCollection::printJournalHeaderTo(std::ofstream &ostrm) {
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

	printFilesTo(fileDirectory, ostrm);

	ostrm << "------------------------------------------" << std::endl;
	ostrm << "Whole trajectory files" << std::endl;
	ostrm << "------------------------------------------" << std::endl;

	UtilGetTrajDirectoryPath(trajectoryDirectory.data(), trajectoryDirectory.size());
	trajectoryDirectory.erase(std::find(trajectoryDirectory.begin(), trajectoryDirectory.end(), '\0'),
							  trajectoryDirectory.end());
	std::remove(std::find(trajectoryDirectory.begin(), trajectoryDirectory.end(), '\0'),
				trajectoryDirectory.end(), '\0');
	fileDirectory.assign(trajectoryDirectory.begin(), trajectoryDirectory.end());

	printFilesTo(fileDirectory, ostrm);

	ostrm << "------------------------------------------" << std::endl;
	ostrm << "Whole configuration files" << std::endl;
	ostrm << "------------------------------------------" << std::endl;

	UtilGetConfDirectoryPath(configurationDirectory.data(), configurationDirectory.size());
	configurationDirectory.erase(std::find(configurationDirectory.begin(), configurationDirectory.end(), '\0'),
								 configurationDirectory.end());
	std::remove(std::find(configurationDirectory.begin(), configurationDirectory.end(), '\0'),
				configurationDirectory.end(), '\0');
	fileDirectory.assign(configurationDirectory.begin(), configurationDirectory.end());

	printFilesTo(fileDirectory, ostrm);

	// TODO: information about file structure
	return 0;
}


/*!
 * \brief getCurrentDateAsString Creates a string on the format YYYY-MM-DD of the current date.
 * \return A std::string containing the current date
 */
std::string JournalModelCollection::getCurrentDateAsString() {
	return getDateAsString(std::chrono::system_clock::now());
}

/*!
 * \brief getDateAsString Creates a string on the format YYYY-MM-DD of the specified date.
 * \param date Timestamp for which date string is to be extracted.
 * \return A std::string containing the date representation
 */
std::string JournalModelCollection::getDateAsString(const std::chrono::system_clock::time_point &date) {
	using Clock = std::chrono::system_clock;
	char dateString[DATE_STRING_MAX_LEN] = {'\0'};
	auto dateRaw = Clock::to_time_t(date);
	auto dateStructPtr = std::localtime(&dateRaw);
	std::strftime(dateString, DATE_STRING_MAX_LEN, "%Y-%m-%d", dateStructPtr);
	return dateString;
}

/*!
 * \brief getJournalFilesFrom Fetches a list of file paths corresponding to journal
 *			files created on the specified date.
 * \param date Date for which journals are to be fetched
 * \return A std::vector containing file paths
 */
std::vector<fs::path> JournalModelCollection::getJournalFilesFrom(const std::chrono::system_clock::time_point &date) {
	std::vector<fs::path> journalsFromDate;
	std::vector<char> buffer(PATH_MAX, '\0');

	UtilGetJournalDirectoryPath(buffer.data(), buffer.size());
	buffer.erase(std::find(buffer.begin(), buffer.end(), '\0'),
								 buffer.end());
	fs::path journalDirPath(buffer.begin(), buffer.end());

	if (!exists(journalDirPath)) {
		throw std::runtime_error("Journal directory " + journalDirPath.string() + " does not exist");
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

/*!
 * \brief printFilesTo Prints the contents of all files in chosen directory to the
 *			selected stream
 * \param inputDirectory Directory containing files to be printed
 * \param outputFile Stream to which contents are to be printed
 * \return 0 on success, -1 otherwise
 */
int JournalModelCollection::printFilesTo(const fs::path &inputDirectory, std::ostream &outputFile) {

	if (!exists(inputDirectory)) {
		throw std::runtime_error("Unable to find directory " + inputDirectory.string());
	}
	for (const auto &dirEntry : fs::directory_iterator(inputDirectory)) {
		if (fs::is_regular_file(dirEntry.status())) {
			const auto inputFile = dirEntry.path();
			std::ifstream istrm(inputFile.string());
			if (!istrm.is_open()) {
				throw std::runtime_error("Unable to open file " + inputFile.string());
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
