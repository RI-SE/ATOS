#ifndef LOGGING_H
#define LOGGING_H

#include <stdarg.h>
#include <stdio.h>
// Should this be OS independent?
#include <linux/limits.h>

#define LOG_DIR_PATH "log"
#define LOG_MODULE_NAME_MAX_LENGTH 100

// Used for selecting individual log entry urgency as well as log
// filtering level
typedef enum
{
    LOG_LEVEL_ERROR=0,
    LOG_LEVEL_WARNING=1,
    LOG_LEVEL_INFO=2,
    LOG_LEVEL_DEBUG=3
} LOG_LEVEL;

/* LogInit(logName, logMinLevel):
*   Initialize (or reinitialize) a log for writing. Creates a new file in the specified directory,
*   including the specified logName and date of creation. If such a file already exists, a reference
*   to that file is returned instead. Subsequent write operations append to the file and do not over-
*   write previous entries. The second input allows specifying the minimum urgency a message must
*   have to be printed to the file.
*   Inputs:
*     - logName: A null terminated string reference used for naming the log file
*     - logMinLevel: Minimum urgency of messages to be printed to the returned log file
*   Example:
*     // This creates a log named "foo_log-2019-03-08.log" which filters out debug messages
*     LogInit("foo_log", LOG_LEVEL_INFO);
*/
void LogInit(char* logName, LOG_LEVEL logMinLevel);

/* LogMessage(messageLevel, format, ...):
*   Write an entry into the specified log, with a specified message level attached to the message.
*   The log entry contains the time and date of the entry (one second precision), the message level
*   (e.g. IFO, ERR etc.) and the actual entry. The message is also printed to stdout if it has
*   sufficient urgency.
*   Inputs:
*     - messageLevel: Urgency of the message to be written
*     - format: A null terminated format string which may or may not contain a number of %d, %s etc..
*     - ...: Any number of inputs which match format specifiers in the format string
*   Example:
*     LogInit("foo_log", LOG_LEVEL_INFO);
*     // This enters the following string into foo_log: "[2019-03-08|09:58:23|ERR]: Division by zero (0)!"
*     LogMessage(LOG_LEVEL_ERROR, "Division by %s (%d)!", "zero", 0);
*/
void LogMessage(LOG_LEVEL messageLevel, const char* format, ...);

/* LogPrint(format, ...):
******  THIS FUNCTION IS INTENDED FOR QUICK DEBUGGING AND SHOULD NOT TO BE USED IN FINAL CODE ******
*   Write an entry into the specified log, without specifying the log level. The log entry contains the
*   time and date of the entry (one second precision), the tag 'N/A' for log level and the actual entry.
*   The message is also printed to stdout.
*   Inputs:
*     - format: A null terminated format string which may or may not contain a number of %d, %s etc..
*     - ...: Any number of inputs which match format specifiers in the format string
*   Example:
*     LogInit("foo_log", LOG_LEVEL_INFO);
*     // This enters the following string into foo_log: "[2019-03-08|09:58:23|N/A]: Division by zero (0)!"
*     LogPrint("Division by %s (%d)!", "zero", 0);
*/
void LogPrint(const char* format, ...);

#endif
