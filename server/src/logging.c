#include "logging.h"
#include <sys/time.h>
#include <time.h>
#include <string.h>

// String used to create the log file
#define FILENAME_DATESTR_MAX_LENGTH 100
#define FILENAME_DATESTR_FORMAT "%Y-%m-%d"
#define LOG_FILE_WRITE_MODE "a"

// String used for each log entry
#define LOG_DATESTR_MAX_LENGTH 100
#define LOG_DATESTR_FORMAT "%Y-%m-%d|%H:%M:%S"


// Used as reference to a log
typedef struct
{
    LOG_LEVEL level;
    char logModuleName[LOG_MODULE_NAME_MAX_LENGTH];
    char fullLogPath[PATH_MAX];
} LOG;

// Local static variable (which will be different for every fork() of the process)
static LOG moduleLog;

// For logging errors in writing to logs
static LOG logging_meta_info;
#define META_LOG_NAME "logging-meta-info"

void init_log(char* logName, LOG_LEVEL logMinLevel)
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

    moduleLog = moduleLog;

    // Ensure any file issues are caught early by printing to the file here
    log_message(LOG_LEVEL_INFO, "Log initialized.");
}

void log_message(LOG_LEVEL messageLevel, const char* format, ...)
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
                fprintf(fp, "ERR");
                break;
            case LOG_LEVEL_WARNING:
                fprintf(fp, "WRN");
                break;
            case LOG_LEVEL_INFO:
                fprintf(fp, "IFO");
                break;
            case LOG_LEVEL_DEBUG:
                fprintf(fp, "DBG");
                break;
            default:
                fprintf(fp, "???");
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


