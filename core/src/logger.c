/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : logger.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#include "positioning.h"
#include "maestroTime.h"
#include "logger.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LOG_PATH MAESTRO_TEST_DIR_PATH "journal/"
#define LOG_FILE_ENDING ".log"
#define CSV_FILE_ENDING ".csv"
#define FORWARD_SLASH "/"
#define LOG_CONTROL_MODE 0
#define LOG_REPLAY_MODE 1
#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define TIMESTAMP_BUFFER_LENGTH 20
#define SPECIFIC_CHAR_THRESHOLD_COUNT 10
#define MAX_UTC_TIMESTAMP_STRLEN 21	// Maximum string length of an uint64 UTC timestamp is strlen("%u",1.8447e19)+1 i.e. 21
#define MAX_DATE_STRLEN 25		// Maximum string length of a time stamp on the format "2035;12;31;24;59;59;1000" is 25

#define MAX_LOG_ROW_LENGTH (MAX_DATE_STRLEN + strlen(";") + 2*(MAX_UTC_TIMESTAMP_STRLEN + strlen(";")) + strlen("255") + strlen(";") + MBUS_MAX_DATALEN + strlen("\n") + 1)

#define ACCESS_MODE_READ "r"
#define ACCESS_MODE_WRITE "w"
#define ACCESS_MODE_WRITE_AND_READ "w+"
#define ACCESS_MODE_APPEND "a"
#define ACCESS_MODE_APPEND_AND_READ "a+"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vCreateLogFolder(char logFolder[MAX_FILE_PATH]);
static void vInitializeLog(char *logFilePath, unsigned int filePathLength, char *csvLogFilePath,
						   unsigned int csvFilePathLength);
static int ReadLogLine(FILE * fd, char *Buffer);
static int CountFileRows(FILE * fd);
void vLogCommand(enum COMMAND command, char *commandData, struct timeval recvTime, char *pcLogFile,
				 char *pcLogFileComp);
void vLogMonitorData(char *commandData, ssize_t commandDatalen, struct timeval recvTime, char *pcLogFile,
					 char *pcLogFileComp);
void vLogScenarioControlData(enum COMMAND command, unsigned char *commandData, ssize_t commandDatalen,
							 struct timeval recvTime, char *pcLogFile, char *pcLogFileComp);
static void signalHandler(int signo);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/
#define MODULE_NAME "Logger"
static volatile int iExit = 0;

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/


void logger_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {
	char pcLogFile[MAX_FILE_PATH];	//!< Log file path and name buffer
	char pcLogFileComp[MAX_FILE_PATH];	//!< CSV log file path and name buffer
	char busReceiveBuffer[MBUS_MAX_DATALEN];	//!< Buffer for receiving from message bus
	char busSendBuffer[MBUS_MAX_DATALEN];	//!< Buffer for sending to message bus
	char pcReadBuffer[MAX_LOG_ROW_LENGTH];	//!< Buffer for reading files
	char subStrings[MBUS_MAX_DATALEN];
	char journalPathDir[MAX_FILE_PATH];

	struct timeval recvTime;

	// Listen for commands
	enum COMMAND command = COMM_INV;
	ssize_t receivedBytes = 0;

	int GPSweek;
	FILE *replayfd;
	struct timespec sleep_time, ref_time;
	U8 isFirstInit = 1;

	// Execution mode
	int LoggerExecutionMode = LOG_CONTROL_MODE;
	int RowCount = 0;

	// Initialize log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Logger task running with PID: %d", getpid());

	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Initialize message bus connection
	if (iCommInit())
		util_error("Unable to initialize connection to message bus");

	// Create log directory if it does not exist
	UtilGetJournalDirectoryPath(journalPathDir, sizeof (journalPathDir));
	vCreateLogFolder(journalPathDir);

	// our time
	char *src;
	uint64_t NewTimestamp, OldTimestamp;

	while (!iExit) {
		bzero(busReceiveBuffer, sizeof (busReceiveBuffer));

		receivedBytes = iCommRecv(&command, busReceiveBuffer, sizeof (busReceiveBuffer), &recvTime);

		switch (command) {
		case COMM_ABORT:
			break;

		case COMM_INIT:
			if (isFirstInit) {
				LogMessage(LOG_LEVEL_INFO, "Initializing test log...");
				vInitializeLog(pcLogFile, sizeof (pcLogFile), pcLogFileComp, sizeof (pcLogFileComp));
				isFirstInit = 0;
			}
			else {
				LogMessage(LOG_LEVEL_WARNING, "Received unexpected INIT command");
			}

			break;

		case COMM_TRCM:
		case COMM_ACCM:
		case COMM_TREO:
		case COMM_EXAC:
			if (!isFirstInit)
				vLogScenarioControlData(command, busReceiveBuffer, receivedBytes, recvTime, pcLogFile,
										pcLogFileComp);
			else
				LogMessage(LOG_LEVEL_WARNING, "Received command %u while log uninitialized", command);
			break;
		case COMM_STRT:
		case COMM_ARM:
		case COMM_DISARM:
		case COMM_STOP:
		case COMM_CONNECT:
		case COMM_VIOP:
		case COMM_TRAJ:
		case COMM_TRAJ_TOSUP:
		case COMM_TRAJ_FROMSUP:
		case COMM_ASP:
		case COMM_LOG:
			if (!isFirstInit)
				vLogCommand(command, busReceiveBuffer, recvTime, pcLogFile, pcLogFileComp);
			else
				LogMessage(LOG_LEVEL_WARNING, "Received command %u while log uninitialized", command);
			break;

		case COMM_DISCONNECT:
			isFirstInit = 1;
			break;

		case COMM_MONR:
			if (!isFirstInit) {
				vLogMonitorData(busReceiveBuffer, receivedBytes, recvTime, pcLogFile, pcLogFileComp);
			}
			else
				LogMessage(LOG_LEVEL_WARNING, "Received command %u while log uninitialized", command);
			break;

		case COMM_OSEM:
			strcpy(subStrings, busReceiveBuffer);

			// Returns first datapoint of OSEM (GPSWeek)
			char *token = strtok(busReceiveBuffer, ";");

			if (token != NULL) {
				GPSweek = atoi(token);
				LogMessage(LOG_LEVEL_INFO, "GPS week of OSEM: %d", GPSweek);
			}
			else {
				LogMessage(LOG_LEVEL_WARNING, "Could not read GPS week in OSEM");
			}


			// Rest of OSEM if needed
			/*
			   while (token != NULL) {
			   printf("%s\n", token);
			   token = strtok(NULL, ";");
			   }
			 */

			break;

		case COMM_OBC_STATE:
			LogMessage(LOG_LEVEL_DEBUG, "Disregarding object control state reporting");
			break;

		case COMM_REPLAY:
			// This function was not kept up to date and some work must be done to make it work again
			LogMessage(LOG_LEVEL_WARNING, "Replay function out of date");
			if (iCommSend(COMM_CONTROL, NULL, 0) < 0)
				util_error("Communication error - exiting");
			break;

			LoggerExecutionMode = LOG_REPLAY_MODE;
			LogMessage(LOG_LEVEL_INFO, "Logger in REPLAY mode <%s>", busReceiveBuffer);

			replayfd = fopen(busReceiveBuffer, ACCESS_MODE_READ);
			RowCount = UtilCountFileRows(replayfd);
			fclose(replayfd);

			replayfd = fopen(busReceiveBuffer, ACCESS_MODE_READ);
			LogMessage(LOG_LEVEL_INFO, "Rows: %d", RowCount);;
			if (replayfd) {
				UtilReadLineCntSpecChars(replayfd, pcReadBuffer);	// Just read first line and ignore it
				int SpecChars = 0, j = 0;
				int FirstIteration = 1;

				//char *src;
				//uint64_t NewTimestamp, OldTimestamp;
				do {
					bzero(pcReadBuffer, sizeof (pcReadBuffer));
					SpecChars = UtilReadLineCntSpecChars(replayfd, pcReadBuffer);

					j++;
					if (SpecChars == SPECIFIC_CHAR_THRESHOLD_COUNT) {
						MonitorDataType monrData;

						UtilStringToMonitorData(pcReadBuffer, sizeof (pcReadBuffer), &monrData);
						NewTimestamp = TimeGetAsGPSqmsOfWeek(&monrData.data.timestamp);

						if (!FirstIteration) {	/* Wait a little bit */
							sleep_time.tv_sec = 0;
							sleep_time.tv_nsec = (NewTimestamp - OldTimestamp) * 1000000;
							(void)nanosleep(&sleep_time, &ref_time);
						}
						else
							OldTimestamp = NewTimestamp;

						// TODO: Convert monrData to binary form
						//if (iCommSend(COMM_MONR, , ) < 0)
						//  util_error("Communication error - exiting");

						FirstIteration = 0;
						OldTimestamp = NewTimestamp;
					}

				} while (--RowCount >= 0);

			}
			else {
				LogMessage(LOG_LEVEL_WARNING, "Failed to open file: %s", busReceiveBuffer);
			}

			LogMessage(LOG_LEVEL_INFO, "Replay done");

			if (iCommSend(COMM_CONTROL, NULL, 0) < 0)
				util_error("Communication error - exiting");

			break;

		case COMM_CONTROL:
			LoggerExecutionMode = LOG_CONTROL_MODE;
			LogMessage(LOG_LEVEL_INFO, "Logger in CONTROL mode");
			break;

		case COMM_EXIT:
			iExit = 1;
			break;

		case COMM_DATA_DICT:
			LogMessage(LOG_LEVEL_INFO, "Data dictionary has been modified");
			break;

		case COMM_INV:
			break;

		case COMM_GETSTATUS:
			memset(busSendBuffer, 0, sizeof (busSendBuffer));
			sprintf(busSendBuffer, "%s", MODULE_NAME);
			if (iCommSend(COMM_GETSTATUS_OK, busSendBuffer, sizeof (busSendBuffer)) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending GETSTATUS.");
			}
			break;

		case COMM_GETSTATUS_OK:
			break;

		default:
			LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
		}
	}

	(void)iCommClose();

	LogMessage(LOG_LEVEL_INFO, "Logger exiting");
}



/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		iExit = 1;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}

void vCreateLogFolder(char logFolder[MAX_FILE_PATH]) {
	int iResult;
	struct stat st = { 0 };

	// Create log directory if it does not exist
	if (stat(logFolder, &st) == -1) {
		if (errno == ENOENT) {
			LogMessage(LOG_LEVEL_INFO, "Creating log directory <%s>", logFolder);
			iResult = mkdir(logFolder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			if (iResult < 0) {
				util_error("Unable to create log directory");
			}
		}
		else {
			util_error("Unable to stat log directory");
		}
	}
}


void vInitializeLog(char *logFilePath, unsigned int filePathLength, char *csvLogFilePath,
					unsigned int csvFilePathLength) {
	struct timeval tvTime;
	char logFileDirectoryPath[MAX_FILE_PATH];
	char trajPathDir[MAX_FILE_PATH];
	char confPathDir[MAX_FILE_PATH];
	char confFilePath[MAX_FILE_PATH];
	char trigFilePath[MAX_FILE_PATH];
	char journalPathDir[MAX_FILE_PATH];
	char DateBuffer[FILENAME_MAX];
	FILE *filefd, *fileread;
	char msString[10];
	char sysCommand[100];
	char pcBuffer[MAX_LOG_ROW_LENGTH];
	DIR *dir;
	struct dirent *ent;
	int read;

	UtilGetConfDirectoryPath(confPathDir, sizeof (confPathDir));
	strcat(confFilePath, confPathDir);
	strcat(confFilePath, CONF_FILE_NAME);
	strcat(trigFilePath, confPathDir);
	strcat(trigFilePath, TRIGGER_ACTION_FILE_NAME);
	UtilGetTrajDirectoryPath(trajPathDir, sizeof (trajPathDir));
	UtilGetJournalDirectoryPath(journalPathDir, sizeof (journalPathDir));

	// Clear the two path strings
	bzero(logFilePath, filePathLength);
	bzero(csvLogFilePath, csvFilePathLength);

	// Calculate the date and time when the logfile is created
	TimeSetToCurrentSystemTime(&tvTime);
	TimeGetAsDateTime(&tvTime, "%Y-%m-%d_%H.%M.%S", DateBuffer, sizeof (DateBuffer));

	// Append milliseconds
	sprintf(msString, ".%i", (int)tvTime.tv_usec / 1000);
	strcat(DateBuffer, msString);

	// Create log folder named with initialization date and time
	(void)strcpy(logFileDirectoryPath, journalPathDir);
	(void)strcat(logFileDirectoryPath, DateBuffer);

	vCreateLogFolder(logFileDirectoryPath);

	(void)strcat(logFileDirectoryPath, FORWARD_SLASH);

	// Copy configuration file to log directory
	LogMessage(LOG_LEVEL_INFO, "Copying configuration file to log directory");
	(void)strcpy(sysCommand, "cp ");
	(void)strcat(sysCommand, confFilePath);
	(void)strcat(sysCommand, " ");
	(void)strcat(sysCommand, logFileDirectoryPath);
	(void)system(sysCommand);

	// Copy configuration file to log directory
	LogMessage(LOG_LEVEL_INFO, "Copying TriggerAndAction file to log directory");
	(void)strcpy(sysCommand, "cp ");
	(void)strcat(sysCommand, trigFilePath);
	(void)strcat(sysCommand, " ");
	(void)strcat(sysCommand, logFileDirectoryPath);
	(void)system(sysCommand);

	// Check if ./traj directory exists
	if ((dir = opendir(trajPathDir)) == NULL) {
		sprintf(sysCommand, "Unable to open traj directory <%s>", trajPathDir);
		util_error(sysCommand);
	}
	else
		closedir(dir);

	// Copy trajectory files to subdirectory
	LogMessage(LOG_LEVEL_INFO, "Copying trajectory files to log directory");
	(void)strcpy(sysCommand, "cp -R ");
	(void)strcat(sysCommand, trajPathDir);
	(void)strcat(sysCommand, " ");
	(void)strcat(sysCommand, logFileDirectoryPath);
	(void)system(sysCommand);

	// TODO: Copy geofence files

	// Create filenames using date and time
	(void)strcpy(logFilePath, logFileDirectoryPath);
	(void)strcat(logFilePath, DateBuffer);
	(void)strcat(logFilePath, LOG_FILE_ENDING);
	(void)strcpy(csvLogFilePath, logFilePath);
	(void)strcat(csvLogFilePath, CSV_FILE_ENDING);

	LogMessage(LOG_LEVEL_INFO, "Opening log file to use: <%s>", logFilePath);
	LogMessage(LOG_LEVEL_INFO, "Opening csv file to use: <%s>", csvLogFilePath);

	// Print trajectory files to log
	LogMessage(LOG_LEVEL_DEBUG, "Printing trajectories to log");
	filefd = fopen(logFilePath, ACCESS_MODE_WRITE_AND_READ);

	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"------------------------------------------\nWhole Trajectory files:\n------------------------------------------\n");
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	// Open directory traj
	if ((dir = opendir(trajPathDir)) != NULL) {
		while ((ent = readdir(dir)) != NULL) {
			// Copy all files in trajectory and add them to the log file
			bzero(pcBuffer, sizeof (pcBuffer));
			strcpy(pcBuffer, trajPathDir);
			strcat(pcBuffer, ent->d_name);
			if (access(pcBuffer, 0) == 0) {
				fileread = fopen(pcBuffer, ACCESS_MODE_READ);
				read = fgetc(fileread);
				while (read != EOF) {
					fputc(read, filefd);
					read = fgetc(fileread);
				}
				fclose(fileread);
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Failed to open <%s>", pcBuffer);
			}
		}
		closedir(dir);
	}
	else {
		sprintf(sysCommand, "Unable to open traj directory <%s>", trajPathDir);
		util_error(sysCommand);
	}
	fclose(filefd);

	// Print configuration to log
	LogMessage(LOG_LEVEL_DEBUG, "Printing configuration to log");
	filefd = fopen(logFilePath, ACCESS_MODE_APPEND_AND_READ);

	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"\n------------------------------------------\nWhole Config file:\n------------------------------------------\n");
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);


	/* If file conf file exists and we have read permission do */
	if (access(confFilePath, 0) == 0) {
		/*read the .conf file and print it in to the .log file */
		fileread = fopen(confFilePath, ACCESS_MODE_READ);
		read = fgetc(fileread);
		while (read != EOF) {
			fputc(read, filefd);
			read = fgetc(fileread);
		}
		fclose(fileread);
	}
	else {
		sprintf(sysCommand, "Unable to open <%s>", confFilePath);
		util_error(sysCommand);
	}

	// Add some information about the standard log file format and what is what in the MONR message
	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"\n------------------------------------------\nInformation about log structure\n------------------------------------------\nLog started; Date:%s\nGenerall structure:\n",
			DateBuffer);
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);
	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<Command message nr>;<Data>\n");
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"Monor message structure(command message nr = 3):\n<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<GPS Time ms>;<Command message nr>;<Data>;<Object_address (IP number)>;<0>;");
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"<GPS Second of week (unit 0.25 milliseconds)>;<x-position, unit 0.001 meter>;<y-position, unit 0.001 meter>;<z-position, unit0.001>;<heading, unit 0.01 degrees>;<Logitudinal speed,");
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	bzero(pcBuffer, sizeof (pcBuffer));

	sprintf(pcBuffer, "Version;%s\n", MaestroVersion);
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	sprintf(pcBuffer, "unit 0.01 m/s>;<Lateral speed, unit 0.01 m/s>;<Longitudinal Acceleration, unit 0.001 m/s^2>;<Lateral Acceleration, unit 0.001 m/s^2>;<Driving direction>;<Object state>;<Ready to ARM>;<ErrorState>\n");	// add more her if we want more data
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);
	bzero(pcBuffer, sizeof (pcBuffer));
	sprintf(pcBuffer,
			"Command message nr:\nCOMM_START:%d\nCOMM_STOP:%d\nCOMM_MONR:%d\nCOMM_EXIT:%d\nCOMM_ARM:%d\nCOMM_DISARM:%d\nCOMM_REPLAY:%d\nCOMM_CONTROL:%d\nCOMM_ABORT:%d\nCOMM_INIT:%d\nCOMM_CONNECT:%d\nCOMM_OBC_STATE:%d\nCOMM_DISCONNECT:%d\nCOMM_LOG:%d\nCOMM_VIOP:%d\nCOMM_INV:%d\n------------------------------------------\n Log start\n------------------------------------------\n",
			COMM_STRT, COMM_STOP, COMM_MONR, COMM_EXIT, COMM_ARM, COMM_DISARM, COMM_REPLAY,
			COMM_CONTROL, COMM_ABORT, COMM_INIT, COMM_CONNECT, COMM_OBC_STATE, COMM_DISCONNECT, COMM_LOG,
			COMM_VIOP, COMM_INV);
	(void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

	fclose(filefd);
}

/*!
 * \brief vLogCommand Logs a command with associated data to two log files
 * \param command Message bus command number
 * \param commandData Null terminated string of command data
 * \param recvTime Time when command was received
 * \param pcLogFile Path to log file
 * \param pcLogFileComp Path to .csv log file
 */
void vLogCommand(enum COMMAND command, char *commandData, struct timeval recvTime, char *pcLogFile,
				 char *pcLogFileComp) {
	FILE *filefd;
	char DateBuffer[MAX_DATE_STRLEN];

	bzero(DateBuffer, sizeof (DateBuffer));
	TimeGetAsDateTime(&recvTime, "%Y;%m;%d;%H;%M;%S;%q", DateBuffer, sizeof (DateBuffer));

	// Remove newlines in HTTP requests for nicer printouts.
	for (unsigned long i = 0; i < strlen(commandData); i++) {
		if (commandData[i] == '\n') {
			commandData[i] = ' ';
		}
	}

	// Write to log file
	filefd = fopen(pcLogFile, ACCESS_MODE_APPEND);
	if (filefd != NULL) {
		fprintf(filefd, "%s;%ld;%u;%s\n", DateBuffer, TimeGetAsUTCms(&recvTime), (char)command, commandData);
		fclose(filefd);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcLogFile);
	}

	// Write to .csv file
	filefd = fopen(pcLogFileComp, ACCESS_MODE_APPEND);
	if (filefd != NULL) {
		fprintf(filefd, "%s;%ld;%u;%s\n", DateBuffer, TimeGetAsUTCms(&recvTime), (char)command, commandData);
		fclose(filefd);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcLogFileComp);
	}
}

/*!
 * \brief vLogMonitorData Logs a raw MONR message in a human-readable format to the selected log
 * \param commandData Data accompanying the message bus message
 * \param commandDatalen Length of the commandData array
 * \param pcLogFile Log file to write to
 * \param pcLogFileComp Secondary log file to write to (currently not used)
 */
void vLogMonitorData(char *commandData, ssize_t commandDatalen, struct timeval recvTime, char *pcLogFile,
					 char *pcLogFileComp) {
	FILE *filefd;
	char DateBuffer[MAX_DATE_STRLEN];
	char ipStringBuffer[INET_ADDRSTRLEN];
	char printBuffer[MAX_LOG_ROW_LENGTH];
	MonitorDataType monitorData;
	int printedBytes = 0;
	int totalPrintedBytes = 0;

	if (commandDatalen < 0)
		return;

	UtilPopulateMonitorDataStruct(commandData, (size_t) commandDatalen, &monitorData);

	memset(DateBuffer, 0, sizeof (DateBuffer));
	TimeGetAsDateTime(&recvTime, "%Y;%m;%d;%H;%M;%S;%q", DateBuffer, sizeof (DateBuffer));

	printedBytes = snprintf(printBuffer, sizeof (printBuffer), "%s;%ld;%ld;%d;", DateBuffer,
							TimeGetAsUTCms(&monitorData.data.timestamp),
							TimeGetAsGPSms(&monitorData.data.timestamp), (unsigned char)COMM_MONR);

	totalPrintedBytes += printedBytes;
	if (printedBytes < 0 || (size_t) totalPrintedBytes > sizeof (printBuffer)) {
		LogMessage(LOG_LEVEL_ERROR, "Error printing data to string");
		return;
	}

	printedBytes =
		snprintf(printBuffer + totalPrintedBytes, sizeof (printBuffer) - (size_t) totalPrintedBytes, "%s;",
				 inet_ntop(AF_INET, &monitorData.ClientIP, ipStringBuffer, sizeof (ipStringBuffer)));

	totalPrintedBytes += printedBytes;
	if (printedBytes < 0 || (size_t) totalPrintedBytes > sizeof (printBuffer)) {
		LogMessage(LOG_LEVEL_ERROR, "Error printing data to string");
		return;
	}

	printedBytes =
		snprintf(printBuffer + totalPrintedBytes, sizeof (printBuffer) - (size_t) totalPrintedBytes, "%u;",
				 monitorData.ClientID);

	totalPrintedBytes += printedBytes;
	if (printedBytes < 0 || (size_t) totalPrintedBytes > sizeof (printBuffer)) {
		LogMessage(LOG_LEVEL_ERROR, "Error printing data to string");
		return;
	}

	objectMonitorDataToASCII(&monitorData.data, printBuffer + totalPrintedBytes,
							 sizeof (printBuffer) - (size_t) totalPrintedBytes);

	filefd = fopen(pcLogFile, ACCESS_MODE_APPEND_AND_READ);
	if (filefd == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcLogFile);
		return;
	}
	fprintf(filefd, "%s\n", printBuffer);
	fclose(filefd);
}


void vLogScenarioControlData(enum COMMAND command, unsigned char *commandData, ssize_t commandDatalen,
							 struct timeval recvTime, char *pcLogFile, char *pcLogFileComp) {
	FILE *filefd;
	char DateBuffer[MAX_DATE_STRLEN];
	char ipStringBuffer[INET_ADDRSTRLEN];
	EXACData exac;
	ACCMData accm;
	TREOData treo;
	TRCMData trcm;

	struct timeval messageTimeField, systemTime;
	const int debug = 0;

	if (commandDatalen < 0) {
		LogMessage(LOG_LEVEL_WARNING, "Cannot log scenario control data with negative length");
		return;
	}

	TimeSetToCurrentSystemTime(&systemTime);

	//UtilPopulateMonitorDataStruct(commandData, (size_t)(commandDatalen), &monitorData, debug);
	//TimeSetToGPStime(&monrTime, TimeGetAsGPSweek(&systemTime), monitorData.MONR.GPSQmsOfWeekU32);

	bzero(DateBuffer, sizeof (DateBuffer));
	TimeGetAsDateTime(&recvTime, "%Y;%m;%d;%H;%M;%S;%q", DateBuffer, sizeof (DateBuffer));

	filefd = fopen(pcLogFile, ACCESS_MODE_APPEND_AND_READ);
	if (filefd == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcLogFile);
		return;
	}

	fprintf(filefd, "%s;%ld;%u;", DateBuffer, TimeGetAsUTCms(&recvTime), (char)command);

	switch (command) {
	case COMM_TREO:
		UtilPopulateTREODataStructFromMQ(commandData, (size_t) commandDatalen, &treo);
		LogMessage(LOG_LEVEL_INFO, "Trigger event occurred, ID %u", treo.triggerID);
		TimeSetToGPStime(&messageTimeField, TimeGetAsGPSweek(&systemTime), treo.timestamp_qmsow);
		fprintf(filefd, "%u;%ld;", treo.triggerID, TimeGetAsGPSms(&messageTimeField));
		fprintf(filefd, "%s", inet_ntop(AF_INET, &treo.ip, ipStringBuffer, sizeof (ipStringBuffer)));
		break;
	case COMM_EXAC:
		UtilPopulateEXACDataStructFromMQ(commandData, (size_t) commandDatalen, &exac);
		LogMessage(LOG_LEVEL_INFO, "Action execute request detected, ID %u", exac.actionID);
		TimeSetToGPStime(&messageTimeField, TimeGetAsGPSweek(&systemTime), exac.executionTime_qmsoW);
		fprintf(filefd, "%u;%ld;", exac.actionID, TimeGetAsGPSms(&messageTimeField));
		fprintf(filefd, "%s", inet_ntop(AF_INET, &exac.ip, ipStringBuffer, sizeof (ipStringBuffer)));
		break;
	case COMM_TRCM:
		UtilPopulateTRCMDataStructFromMQ(commandData, (size_t) commandDatalen, &trcm);
		LogMessage(LOG_LEVEL_INFO, "Trigger configuration for ID %u received, of type %u", trcm.triggerID,
				   trcm.triggerType);
		fprintf(filefd, "%u;%u;%u;%u;%u", trcm.triggerID, trcm.triggerType, trcm.triggerTypeParameter1,
				trcm.triggerTypeParameter2, trcm.triggerTypeParameter3);
		fprintf(filefd, "%s", inet_ntop(AF_INET, &trcm.ip, ipStringBuffer, sizeof (ipStringBuffer)));
		break;
	case COMM_ACCM:
		UtilPopulateACCMDataStructFromMQ(commandData, (size_t) commandDatalen, &accm);
		LogMessage(LOG_LEVEL_INFO, "Action configuration for ID %u received, of type %u", accm.actionID,
				   accm.actionType);
		fprintf(filefd, "%u;%u;%u;%u;%u", accm.actionID, accm.actionType, accm.actionTypeParameter1,
				accm.actionTypeParameter2, accm.actionTypeParameter3);
		fprintf(filefd, "%s", inet_ntop(AF_INET, &accm.ip, ipStringBuffer, sizeof (ipStringBuffer)));
		break;
	default:
		LogMessage(LOG_LEVEL_ERROR, "Unhandled command in scenario control logging: %u",
				   (unsigned char)command);
	}

	fclose(filefd);
}
