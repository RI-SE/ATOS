#include "logging.h"
#include <sys/time.h>
#include <time.h>
#include <string.h>


/*! Colors for the log */
#define COLOR_ERROR "\x1b[31m"
#define COLOR_WARNING "\x1b[33m"
#define COLOR_INFO "\x1b[0m"
#define COLOR_DEBUG "\x1b[32m"
#define COLOR_BYTES "\x1b[35m"
#define COLOR_RESET "\x1b[0m"

// String used to create the log file
#define FILENAME_DATESTR_MAX_LENGTH 100 /*!< Maximum length of the date tag used for the log file */
#define FILENAME_DATESTR_FORMAT "%Y-%m-%d" /*!< Format of the date tag used for the log file */
#define LOG_FILE_WRITE_MODE "a" /*!< Mode of writing to the log file; 'a' appends to the end */

// String used for each log entry
#define LOG_DATESTR_MAX_LENGTH 100 /*!< Maximum length of the tag placed before each log entry */
#define LOG_DATESTR_FORMAT "%Y-%m-%d|%H:%M:%S" /*!< Format of the tag placed before each log entry */

// Used for debug logs
#define RECORD_DATESTR_MAX_LENGTH 100
#define RECORD_DATESTR_FORMAT "%Y-%m-%d-%H%M%S"
#define BYTE_RECORD_WRITE_MODE "wb"

/*! Used as reference to a log file */
typedef struct
{
    LOG_LEVEL level; /*!< Filter level for the log */
    char logModuleName[LOG_MODULE_NAME_MAX_LENGTH]; /*!< Name of the module using the log */
    char fullLogPath[PATH_MAX]; /*!< Path to the log file used */
} LOG;

static LOG moduleLog; /*!< Local log reference (different for each ::fork of the process */

/*! Log used for reporting logging problems, common between all modules */
static LOG logging_meta_info;
#define META_LOG_NAME "logging-meta-info"

/*!
* \brief Initialize (or reinitialize) a log for writing. Creates a new file in the specified directory,
*   including the specified logName and date of creation. If such a file already exists, a reference
*   to that file is returned instead. Subsequent write operations append to the file and do not over-
*   write previous entries. The second input allows specifying the minimum urgency a message must
*   have to be printed to the file.
* \param logName A null terminated string reference used for naming the log file
* \param logMinLevel Minimum urgency of messages to be printed to the returned log file
* \example
*     // This creates a log named "foo_log-2019-03-08.log" which filters out debug messages
*     LogInit("foo_log", LOG_LEVEL_INFO);
*/
void LogInit(char* logName, LOG_LEVEL logMinLevel)
{
    char dateStr[FILENAME_DATESTR_MAX_LENGTH];
    time_t rawTime;
    struct tm * timeInfo;

    bzero(moduleLog.fullLogPath, PATH_MAX);

    // Build log path name string
    strcpy(moduleLog.fullLogPath, LOG_DIR_PATH);
    strcat(moduleLog.fullLogPath, "/");
    strcat(moduleLog.fullLogPath, logName);
    strcat(moduleLog.fullLogPath, "-");

    // Include current date in log name
    time(&rawTime);
    timeInfo = localtime(&rawTime);
    strftime(dateStr, FILENAME_DATESTR_MAX_LENGTH, FILENAME_DATESTR_FORMAT, timeInfo);
    strcat(moduleLog.fullLogPath, dateStr);

    strcat(moduleLog.fullLogPath, ".log");

    // Save log filter level
    moduleLog.level = logMinLevel;
    strcpy(moduleLog.logModuleName,logName);

    // Create log for reporting possible logging problems
    strcpy(logging_meta_info.fullLogPath, LOG_DIR_PATH);
    strcat(logging_meta_info.fullLogPath, "/");
    strcat(logging_meta_info.fullLogPath, META_LOG_NAME);
    strcat(logging_meta_info.fullLogPath, "-");
    strcat(logging_meta_info.fullLogPath, dateStr);
    strcat(logging_meta_info.fullLogPath, ".log");

    // Ensure any file issues are caught early by printing to the file here
    LogMessage(LOG_LEVEL_INFO, "Log initialized");
}



/*!
* \brief Write an entry into the specified log, with a specified message level attached to the message.
*   The log entry contains the time and date of the entry (one second precision), the message level
*   (e.g. IFO, ERR etc.) and the actual entry. The message is also printed to stdout if it has
*   sufficient urgency.
* \param messageLevel Urgency of the message to be written
* \param format A null terminated format string which may or may not contain a number of %d, %s etc..
* \example
*     LogInit("foo_log", LOG_LEVEL_INFO);
*     // This enters the following string into foo_log: "[2019-03-08|09:58:23|ERR]: Division by zero (0)!"
*     LogMessage(LOG_LEVEL_ERROR, "Division by %s (%d)!", "zero", 0);
*/
void LogMessage(LOG_LEVEL messageLevel, const char* format, ...)
{
    if (messageLevel == LOG_LEVEL_ERROR)
    {
        // TODO: Maybe add stuff into the format string accordingly
        perror(format);
    }

    time_t rawTime;
    struct tm * timeInfo;
    char dateStr[LOG_DATESTR_MAX_LENGTH];
    FILE* fp;

    // To allow for variable argument length
    va_list args;

    if (messageLevel <= moduleLog.level)
    {
        // Do a printout of the message
        va_start(args, format);
        printf("[%s] ", moduleLog.logModuleName);
        vprintf(format, args);
        printf("\n");
        va_end(args);

        // Print to log file
        fp = fopen(moduleLog.fullLogPath, LOG_FILE_WRITE_MODE);
        time(&rawTime);
        timeInfo = localtime(&rawTime);
        strftime(dateStr, LOG_DATESTR_MAX_LENGTH, LOG_DATESTR_FORMAT, timeInfo);
        if (fp != NULL)
        {
            fprintf(fp, "[%s|", dateStr);
            switch (messageLevel)
            {
            case LOG_LEVEL_ERROR:
                fprintf(fp, COLOR_ERROR "ERR" COLOR_RESET);
                break;
            case LOG_LEVEL_WARNING:
                fprintf(fp, COLOR_WARNING "WRN" COLOR_RESET);
                break;
            case LOG_LEVEL_INFO:
                fprintf(fp, COLOR_INFO "IFO" COLOR_RESET);
                break;
            case LOG_LEVEL_DEBUG:
                fprintf(fp, COLOR_DEBUG "DBG" COLOR_RESET);
                break;
            }
            va_start(args, format);
            fprintf(fp, "]: ");
            vfprintf(fp, format, args);
            fprintf(fp, "\n");
            va_end(args);

            fclose(fp);
        }
        else
        {
            fp = fopen(logging_meta_info.fullLogPath, "a");
            if (fp != NULL)
            {
                printf("[%s] Unable to open log file for writing: %s\n", moduleLog.logModuleName, moduleLog.fullLogPath);
                fprintf(fp, "[%s|ERR]: Unable to open log file for writing: %s\n", dateStr, moduleLog.fullLogPath);
                fclose(fp);
            }
            else
            {
                printf("Error: Cannot write to any log files!\n"); // In this case we likely don't have write permissions
            }
        }
    }
}

/*!
 * \brief Create a raw data file with the contents of a byteArray, starting at firstIndex and ending at nBytes-firstIndex.
 *     The file is named according to logName with the date it was created (down to the second), ending with ".raw". If a
 *     previous record exists with the same name, the old record is overwritten.
 * \param byteArray Array of bytes to be written to file
 * \param firstIndex Index of the first byte to be written
 * \param nBytes Number of bytes to write
 * \param logName Descriptive name of the byte record file
 */
void LogRecordBytes(char * byteArray, unsigned long int firstIndex, unsigned long int nBytes, const char * logName)
{
    time_t rawTime;
    struct tm * timeInfo;
    char dateStr[FILENAME_DATESTR_MAX_LENGTH];
    FILE* fp;
    char fullRecordPath[PATH_MAX];

    // Include current date in record name
    time(&rawTime);
    timeInfo = localtime(&rawTime);
    strftime(dateStr, RECORD_DATESTR_MAX_LENGTH, RECORD_DATESTR_FORMAT, timeInfo);

    bzero(fullRecordPath, PATH_MAX);

    // Build record path name string
    strcpy(fullRecordPath, LOG_DIR_PATH);
    strcat(fullRecordPath, "/");
    strcat(fullRecordPath, logName);
    strcat(fullRecordPath, "-");

    strcat(fullRecordPath, dateStr);
    strcat(fullRecordPath, ".raw");

    // Try to open the specified record file
    fp = fopen(fullRecordPath, BYTE_RECORD_WRITE_MODE);

    if (fp == NULL)
    {
        fp = fopen(moduleLog.fullLogPath, LOG_FILE_WRITE_MODE);
        printf("[%s] Unable to open log file for writing: %s\n", moduleLog.logModuleName, moduleLog.fullLogPath);
        fclose(fp);
        return;
    }

    // Write the bytes to the file
    fwrite(byteArray+firstIndex, 1, nBytes-firstIndex, fp);
    fclose(fp);

    // Log the action
    LogMessage(LOG_LEVEL_DEBUG,"Created raw byte file <%s>",fullRecordPath);
}


/*!
 * \brief Print the ASCII hex byte array representation of byteArray to log file, starting at firstIndex
 *     and printing nBytes. This function disregards the log level as it is intended for quick debugging.
 * \param byteArray Array of bytes to be written to file
 * \param firstIndex Index of the first byte to be written
 * \param nBytes Number of bytes to write
 */
void LogPrintBytes(char * byteArray, unsigned long int firstIndex, unsigned long int nBytes)
{
    if (firstIndex >= nBytes)
    {
        LogMessage(LOG_LEVEL_ERROR,"Cannot start at index larger than byte array length");
        return;
    }

    time_t rawTime;
    struct tm * timeInfo;
    char dateStr[LOG_DATESTR_MAX_LENGTH];
    FILE* fp;

    // Print to log file
    fp = fopen(moduleLog.fullLogPath, LOG_FILE_WRITE_MODE);
    time(&rawTime);
    timeInfo = localtime(&rawTime);
    strftime(dateStr, LOG_DATESTR_MAX_LENGTH, LOG_DATESTR_FORMAT, timeInfo);

    if (fp == NULL)
    {
        printf("[%s] Unable to open log file for writing: %s\n", moduleLog.logModuleName, moduleLog.fullLogPath);
        return;
    }

    fprintf(fp, "[%s|" COLOR_BYTES "N/A" COLOR_RESET "]: ", dateStr);
    printf("[%s] ", moduleLog.logModuleName);
    for (unsigned long int i = firstIndex; i < nBytes; ++i)
    {
        printf("%X ", byteArray[i]);
        fprintf(fp,"%X ", byteArray[i]);
    }

    fprintf(fp, "\n");
    printf("\n");

    fclose(fp);
}


/*!
* \brief Write an entry into the specified log, without specifying the log level. The log entry contains the
*   time and date of the entry (one second precision), the tag 'N/A' for log level and the actual entry.
*   The message is also printed to stdout. This function is intended for quick debugging and should not be
*   used for non-debugging purposes.
* \param format A null terminated format string which may or may not contain a number of %d, %s etc..
* \example
*     LogInit("foo_log", LOG_LEVEL_INFO);
*     // This enters the following string into foo_log: "[2019-03-08|09:58:23|N/A]: Division by zero (0)!"
*     LogPrint("Division by %s (%d)!", "zero", 0);
*/
void LogPrint(const char* format, ...)
{
    time_t rawTime;
    struct tm * timeInfo;
    char dateStr[LOG_DATESTR_MAX_LENGTH];
    FILE* fp;
    va_list args;

    // Do a printout of the message
    va_start(args, format);
    printf("[%s] ", moduleLog.logModuleName);
    vprintf(format, args);
    printf("\n");
    va_end(args);

    // Print to log file
    fp = fopen(moduleLog.fullLogPath, LOG_FILE_WRITE_MODE);
    time(&rawTime);
    timeInfo = localtime(&rawTime);
    strftime(dateStr, LOG_DATESTR_MAX_LENGTH, LOG_DATESTR_FORMAT, timeInfo);
    if (fp != NULL)
    {
        va_start(args, format);
        fprintf(fp, "[%s|N/A]: ", dateStr);
        vfprintf(fp, format, args);
        fprintf(fp, "\n");
        va_end(args);

        fclose(fp);
    }
    else
    {
        printf("[%s] Unable to open log file for writing: %s\n", moduleLog.logModuleName, moduleLog.fullLogPath);
    }
}
