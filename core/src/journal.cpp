#include "journal.h"
#include "logging.h"
#include "util.h"

#include <linux/limits.h>
#include <sys/stat.h>
#include <string>
#include <chrono>
#include <ctime>
#include <cerrno>

#define JOURNAL_LABEL_MAX_LENGTH 100
#define FILENAME_DATESTR_MAX_LENGTH 100
#define FILENAME_DATESTR_FORMAT "%Y-%m-%d"
#define JOURNAL_FILE_WRITE_MODE "a"
#define DELIMITER ";"

#define DAY_LENGTH_S (24*60*60)

using Clock = std::chrono::system_clock;
using Days = std::chrono::duration<int, std::ratio_multiply<std::chrono::hours::period, std::ratio<24> >::type>;
using Seconds = std::chrono::duration<double>;
template<class Duration>
using TimePoint = std::chrono::time_point<Clock, Duration>;

static std::string journalPath;
static std::string journalLabel;
static TimePoint<Days> creationDate;

static int reinitializeJournal(void);
static int checkDate(void);
static FILE* beginJournalEntry(void);
static int endJournalEntry(FILE*);

int JournalInit(const char* journalName) {
	journalLabel = journalName;
	return reinitializeJournal();
}

int JournalRecordString(const char* format, ...) {
	va_list args;
	FILE* fp;
	checkDate();
	fp = beginJournalEntry();
	if (fp != nullptr) {
		va_start(args, format);
		vfprintf(fp, format, args);
		va_end(args);
		return endJournalEntry(fp);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open journal file %s for writing", journalPath.c_str());
		return -1;
	}
}

int JournalRecordMonitorData(const ObjectMonitorType* data) {
	FILE* fp;
	char errorString[1024];
	checkDate();
	fp = beginJournalEntry();
	if (fp != nullptr) {
		data->isTimestampValid ? fprintf(fp, "%.6f" DELIMITER, data->timestamp.tv_sec + data->timestamp.tv_usec / 1000000.0)
							   : fprintf(fp, "NaN" DELIMITER);
		data->position.isPositionValid	? fprintf(fp, "%.3f" DELIMITER "%.3f" DELIMITER "%.3f" DELIMITER,
												 data->position.xCoord_m, data->position.yCoord_m, data->position.zCoord_m)
										: fprintf(fp, "NaN" DELIMITER "NaN" DELIMITER "NaN" DELIMITER);
		data->position.isHeadingValid ? fprintf(fp, "%.2f" DELIMITER, data->position.heading_rad)
									  : fprintf(fp, "NaN" DELIMITER);
		data->speed.isLongitudinalValid ? fprintf(fp, "%.2f" DELIMITER, data->speed.longitudinal_m_s)
										: fprintf(fp, "NaN" DELIMITER);
		data->speed.isLateralValid ? fprintf(fp, "%.2f" DELIMITER, data->speed.lateral_m_s)
								   : fprintf(fp, "NaN" DELIMITER);
		data->acceleration.isLongitudinalValid ? fprintf(fp, "%.3f" DELIMITER, data->acceleration.longitudinal_m_s2)
											   : fprintf(fp, "NaN" DELIMITER);
		data->acceleration.isLateralValid ? fprintf(fp, "%.3f" DELIMITER, data->acceleration.lateral_m_s2)
											   : fprintf(fp, "NaN" DELIMITER);
		switch (data->drivingDirection) {
		case DriveDirectionType::OBJECT_DRIVE_DIRECTION_FORWARD:
			fprintf(fp, "FWD" DELIMITER);
			break;
		case DriveDirectionType::OBJECT_DRIVE_DIRECTION_BACKWARD:
			fprintf(fp, "REV" DELIMITER);
			break;
		case DriveDirectionType::OBJECT_DRIVE_DIRECTION_UNAVAILABLE:
		default:
			fprintf(fp, "NaN" DELIMITER);
			break;
		}
		fprintf(fp, "%s" DELIMITER, objectStateToASCII(data->state));
		switch (data->armReadiness) {
		case ObjectArmReadinessType::OBJECT_READY_TO_ARM:
			fprintf(fp, "READY_TO_ARM" DELIMITER);
			break;
		case ObjectArmReadinessType::OBJECT_NOT_READY_TO_ARM:
			fprintf(fp, "NOT_READY_TO_ARM" DELIMITER);
			break;
		case ObjectArmReadinessType::OBJECT_READY_TO_ARM_UNAVAILABLE:
		default:
			fprintf(fp, "NaN" DELIMITER);
			break;
		}
		errorStatusToASCII(data->error, errorString, sizeof (errorString));
		fprintf(fp, "%s" DELIMITER, errorString);
		return endJournalEntry(fp);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open journal file %s for writing", journalPath.c_str());
		return -1;
	}
}

int reinitializeJournal() {
	char dateStr[FILENAME_DATESTR_MAX_LENGTH];
	char journalDirPath[PATH_MAX];
	std::string printout;
	char * cptr;
	struct stat sb;

	// Get current time
	auto now = Clock::now();
	creationDate = std::chrono::time_point_cast<Days>(now);
	auto now_t = Clock::to_time_t(now);
	auto now_tm = *std::localtime(&now_t);
	std::strftime(dateStr, FILENAME_DATESTR_MAX_LENGTH, FILENAME_DATESTR_FORMAT, &now_tm);

	// Form base filename from path and label
	UtilGetJournalDirectoryPath(journalDirPath, sizeof (journalDirPath));
	journalPath = journalDirPath + journalLabel + "-" + dateStr + JOURNAL_FILE_ENDING;

	// Check if log directory exists
	if(stat(journalDirPath, &sb)) {
		// Create directory if not
		if (mkdir(journalDirPath, 0775) != -1) {
			LogMessage(LOG_LEVEL_INFO, "Created directory %s", journalDirPath);
		}
		else {
			LogMessage(LOG_LEVEL_ERROR, "Unable to create directory <%s>", journalDirPath);
			return -1;
		}
	}

	// Remove the newline at end of date string
	strcpy(dateStr, std::asctime(&now_tm));
	cptr = strchrnul(dateStr, '\n');
	*cptr = '\0';

	// Check if journal already exists
	if (access(journalPath.c_str(), F_OK) != -1) {
		LogMessage(LOG_LEVEL_DEBUG, "Found existing journal %s", journalPath.c_str());
		printout = "Opened on ";
		printout.append(dateStr);
	}
	else {
		LogMessage(LOG_LEVEL_DEBUG, "No journal found, creating %s", journalPath.c_str());
		printout = "Created on ";
		printout.append(dateStr);
	}

	// Print initialization message to journal to catch errors
	if (JournalRecordString(printout.c_str()) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to write to file %s", journalPath.c_str());
		return -1;
	}

	LogMessage(LOG_LEVEL_INFO, "Opened journal file %s", journalPath.c_str());
	return 0;
}

int checkDate() {
	auto today = std::chrono::time_point_cast<Days>(Clock::now());
	if (today == creationDate) {
		return 0;
	}
	LogMessage(LOG_LEVEL_INFO, "Detected day rollover - beginning new journal");
	if (reinitializeJournal() == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to reinitialize journal");
		return -1;
	}
	return 0;
}

FILE* beginJournalEntry() {
	FILE *fp = fopen(journalPath.c_str(), JOURNAL_FILE_WRITE_MODE);
	if (fp != nullptr) {
		//Seconds now = Clock::now().time_since_epoch();
		auto now = std::chrono::time_point_cast<Seconds>(Clock::now());
		fprintf(fp, "%f: ", now.time_since_epoch().count());
	}
	return fp;
}

int endJournalEntry(FILE* fp) {
	fprintf(fp, "\n");
	return fclose(fp);
}
