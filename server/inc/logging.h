#ifndef LOGGING_H
#define LOGGING_H

#include <stdarg.h>
#include <stdio.h>
// Should this be OS independent?
#include <linux/limits.h>

#define LOG_DIR_PATH "log" /*!< Directory where log files are to be stored */
#define LOG_MODULE_NAME_MAX_LENGTH 100 /*!< Maximum length of module name */

/*! Used for selecting individual log entry urgency as well as log
* filtering level */
typedef enum
{
    LOG_LEVEL_ERROR=0,      /*!< Error level message    */
    LOG_LEVEL_WARNING=1,    /*!< Warning level message  */
    LOG_LEVEL_INFO=2,       /*!< Info level message     */
    LOG_LEVEL_DEBUG=3       /*!< Debug level message    */
} LOG_LEVEL;

void LogInit(char* logName, LOG_LEVEL logMinLevel);
void LogMessage(LOG_LEVEL messageLevel, const char* format, ...);
void LogPrintBytes(char * byteArray, unsigned long int firstIndex, unsigned long int length);
void LogPrint(const char* format, ...);

#endif
