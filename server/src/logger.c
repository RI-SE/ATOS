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
#include "logger.h"
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#include "maestroTime.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LOG_PATH "./log/"
#define LOG_FILE_ENDING ".log"
#define CSV_FILE_ENDING ".csv"
#define FORWARD_SLASH "/"
#define LOG_CONTROL_MODE 0
#define LOG_REPLAY_MODE 1
#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define TIMESTAMP_BUFFER_LENGTH 20
#define SPECIFIC_CHAR_THRESHOLD_COUNT 10
#define MAX_UTC_TIMESTAMP_STRLEN 21 // Maximum string length of an uint64 UTC timestamp is strlen("%u",1.8447e19)+1 i.e. 21
#define MAX_DATE_STRLEN 25 // Maximum string length of a time stamp on the format "2035;12;31;24;59;59;1000" is 25

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
static void vInitializeLog(char * logFilePath, unsigned int filePathLength, char * csvLogFilePath, unsigned int csvFilePathLength);
static int ReadLogLine(FILE *fd, char *Buffer);
static int CountFileRows(FILE *fd);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/
#define MODULE_NAME "Logger"

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void logger_task(TimeType* GPSTime, GSDType *GSD, LOG_LEVEL logLevel)

{
    char pcLogFile[MAX_FILE_PATH];                          //!< Log file path and name buffer
    char pcLogFileComp[MAX_FILE_PATH];                      //!< CSV log file path and name buffer
    char busReceiveBuffer[MBUS_MAX_DATALEN];                //!< Buffer for receiving from message bus
    char busSendBuffer[MBUS_MAX_DATALEN];                   //!< Buffer for sending to message bus
    char DateBuffer[MAX_DATE_STRLEN];                       //!< Buffer for holding a timestamp in human readable text format
    char pcReadBuffer[MAX_LOG_ROW_LENGTH];                  //!< Buffer for reading files
    char pcBuffer[MAX_LOG_ROW_LENGTH];                      //!< General purpose buffer
    char subStrings[MBUS_MAX_DATALEN];
    struct timeval time, recvTime;

    // Listen for commands
    enum COMMAND command;
    int iExit = 0;

    int GPSweek;
    FILE *filefd,*replayfd, *filefdComp;
    struct timespec sleep_time, ref_time;
    U8 isFirstInit = 1;

    // Initialize log
    LogInit(MODULE_NAME, logLevel);
    LogMessage(LOG_LEVEL_INFO, "Logger task running with PID: %d", getpid());

    // Initialize message bus connection
    if(iCommInit())
        util_error("Unable to initialize connection to message bus");

    // Create folder with date as name and .log file with date as name
    struct stat st = {0};
    if (stat(LOG_PATH, &st) == -1)
    {
        LogMessage(LOG_LEVEL_INFO, "Creating log directory");
        vCreateLogFolder(LOG_PATH);
    }

    (void)strcat(pcLogFile," ");
    (void)strcat(pcLogFileComp," ");



    // Execution mode
    int LoggerExecutionMode = LOG_CONTROL_MODE;
    //int test =100;
    int RowCount = 0;
    // our time
    char *find_time;
    char *src;
    uint64_t NewTimestamp, OldTimestamp,Timestamp;

    while(!iExit)
    {

        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));

        (void)iCommRecv(&command, busReceiveBuffer, sizeof(busReceiveBuffer), &recvTime);

        if(LoggerExecutionMode == LOG_CONTROL_MODE && command != COMM_OBC_STATE && command != COMM_MONI )
        {
            bzero(DateBuffer, sizeof(DateBuffer));
            TimeGetAsDateTime(&recvTime, "Y;%m;%d;%H;%M;%S;%q", DateBuffer, sizeof(DateBuffer));

            // Remove newlines in http Requests for nicer printouts.
            for (unsigned long i = 0; i < strlen(busReceiveBuffer); i++){
                if(busReceiveBuffer[i] == '\n'){
                  busReceiveBuffer[i] = ' ';
                }
            }

            bzero(pcBuffer, sizeof(pcBuffer));
            sprintf ( pcBuffer,"%s;%ld;%d;%s\n", DateBuffer, TimeGetAsUTCms(&recvTime), (char)command, busReceiveBuffer);

            filefd = fopen(pcLogFile, ACCESS_MODE_APPEND);
            if (filefd != NULL)
            {
                (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
                fclose(filefd);
            }
            else
            {
                LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcLogFile);
            }

            filefdComp = fopen(pcLogFileComp, ACCESS_MODE_APPEND);
            if (filefdComp != NULL)
            {
                (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefdComp);
                fclose(filefdComp);
            }
            else
            {
                LogMessage(LOG_LEVEL_ERROR,"Unable to open file <%s>",pcLogFileComp);
            }

        }

        switch(command)
        {
        case COMM_DISCONNECT:

            isFirstInit = 1;
            break;

        case COMM_ABORT:

            isFirstInit = 1;
            break;
        case COMM_MONR:
            // TODO: use new MONR message
            break;
        case COMM_MONI:

            filefd = fopen(pcLogFile, ACCESS_MODE_APPEND_AND_READ);

            strcpy(subStrings,busReceiveBuffer);

            char* GPSSecondOfWeek = strtok(subStrings, ";");

            int counter = 0;
            while (GPSSecondOfWeek != NULL && counter < 2)  // Get GPS second of week
            {
              //printf("%s\n", token);
              GPSSecondOfWeek = strtok(NULL, ";");
              counter++;
            }

            TimeSetToGPStime(&time, (uint16_t)GPSweek, (uint32_t)(atoi(GPSSecondOfWeek)*4));

            bzero(DateBuffer, sizeof(DateBuffer));
            TimeGetAsDateTime(&recvTime, "Y;%m;%d;%H;%M;%S;%q", DateBuffer, sizeof(DateBuffer));

            bzero(pcBuffer, sizeof(pcBuffer));
            sprintf (pcBuffer, "%s;%ld;%ld;%d;%s\n", DateBuffer, TimeGetAsUTCms(&time), TimeGetAsGPSms(&time), command, busReceiveBuffer);

            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

            fclose(filefd);

            break;

        case COMM_OSEM:

            strcpy(subStrings, busReceiveBuffer);

            // Returns first datapoint of OSEM (GPSWeek)
            char* token = strtok(busReceiveBuffer, ";");
            GPSweek = atoi(token);

            LogMessage(LOG_LEVEL_INFO, "GPS week of OSEM: %d", GPSweek);

            // Rest of OSEM if needed
            /*
            while (token != NULL) {
              printf("%s\n", token);
              token = strtok(NULL, ";");
            }
            */

            break;

        case COMM_OBC_STATE:

            LogMessage(LOG_LEVEL_DEBUG,"Disregarding object control state reporting");
            break;

        case COMM_REPLAY:

            LoggerExecutionMode = LOG_REPLAY_MODE;
            LogMessage(LOG_LEVEL_INFO, "Logger in REPLAY mode <%s>", busReceiveBuffer);

            replayfd = fopen (busReceiveBuffer, ACCESS_MODE_READ);
            RowCount = UtilCountFileRows(replayfd);
            fclose(replayfd);

            replayfd = fopen (busReceiveBuffer, ACCESS_MODE_READ);
            LogMessage(LOG_LEVEL_INFO, "Rows: %d", RowCount);;
            if(replayfd)
            {
                UtilReadLineCntSpecChars(replayfd, pcReadBuffer); //Just read first line
                int SpecChars = 0, j=0;
                char TimestampBuffer[TIMESTAMP_BUFFER_LENGTH];
                int FirstIteration = 1;
                //char *src;
                //uint64_t NewTimestamp, OldTimestamp;
                do
                {
                    bzero(pcReadBuffer, sizeof(pcReadBuffer));
                    SpecChars = UtilReadLineCntSpecChars(replayfd, pcReadBuffer);

                    j++;
                    if(SpecChars == SPECIFIC_CHAR_THRESHOLD_COUNT)
                    {
                        /* Read to second ';' in row = 418571059920: 3 1;0;418571059920;577776566;127813082;0;0;3600;0; */
                        src = strchr(pcReadBuffer, ';');
                        src = strchr(src+1, ';');

                        /* Get the current timestamp */
                        bzero(TimestampBuffer, TIMESTAMP_BUFFER_LENGTH);
                        strncpy(TimestampBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)strchr(src, ';')  - 1);
                        NewTimestamp = atol(TimestampBuffer);

                        if(!FirstIteration)
                        {	/* Wait a little bit */
                            sleep_time.tv_sec = 0;
                            sleep_time.tv_nsec = (NewTimestamp - OldTimestamp)*1000000;
                            (void)nanosleep(&sleep_time,&ref_time);
                        } else OldTimestamp = NewTimestamp;

                        //printf("Wait time : %ld ms\n", NewTimestamp - OldTimestamp);
                        //printf("Timestamp: %s\n", TimestampBuffer);
                        /* Build the message */
                        /* Read to second ' ' in row = 418571059920: 3 1;0;418571059920;577776566;127813082;0;0;3600;0; */
                        src = strchr(pcReadBuffer, ' ');
                        src = strchr(src+1, ' ');
                        bzero(busSendBuffer, sizeof(busSendBuffer));
                        //strcpy(busSendBuffer, "MONR;");
                        strcat(busSendBuffer, src+1);
                        if(iCommSend(COMM_MONI, busSendBuffer, strlen(busSendBuffer)+1) < 0)
                            util_error("Communication error - exiting");

                        FirstIteration = 0;
                        OldTimestamp = NewTimestamp;
                    };
                    LogMessage(LOG_LEVEL_INFO,"%d:%d:%d<%s>",RowCount,j,SpecChars,busSendBuffer);

                    /*
                    bzero(TimeStampUTCBufferRecv,MQ_ETSI_LENGTH);
                    (void)iCommRecv(&iCommand,busReceiveBuffer,MQ_MAX_MESSAGE_LENGTH,TimeStampUTCBufferRecv);

                    if(iCommand == COMM_STOP)
                    {
                        printf("Replay stopped by user.\n");
                        (void)iCommSend(COMM_CONTROL, NULL);
                    }*/

                } while(--RowCount >= 0);

            }
            else
            {
                LogMessage(LOG_LEVEL_WARNING, "Failed to open file: %s", busReceiveBuffer);
            }

            LogMessage(LOG_LEVEL_INFO,"Replay done");

            if(iCommSend(COMM_CONTROL, NULL, 0) < 0)
                util_error("Communication error - exiting");

            break;

        case COMM_CONTROL:

            LoggerExecutionMode = LOG_CONTROL_MODE;
            LogMessage(LOG_LEVEL_INFO,"Logger in CONTROL mode");
            break;

        case COMM_EXIT:

            iExit = 1;
            break;

        case COMM_INIT:

            if(isFirstInit)
            {
                LogMessage(LOG_LEVEL_INFO,"Initializing test log...");
                vInitializeLog(pcLogFile, sizeof(pcLogFile), pcLogFileComp, sizeof(pcLogFileComp));
                isFirstInit = 0;
            }
            else
            {
                LogMessage(LOG_LEVEL_WARNING,"Received unexpected INIT command");
            }

            break;
        case COMM_INV:
            break;
        default:
            LogMessage(LOG_LEVEL_WARNING,"Unhandled message bus command: %u", (char)command);
        }

    }

    (void)iCommClose();

    LogMessage(LOG_LEVEL_INFO,"Logger exiting");
}



/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/


void vCreateLogFolder(char logFolder[MAX_FILE_PATH])
{
    int iResult;
    DIR* directory;
    struct dirent *directory_entry;
    int iMaxFolder = 0;

    directory = opendir(logFolder);

    // If the directory does not exist, create it
    if(directory == NULL)
    {
        iResult = mkdir(logFolder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (iResult < 0)
        {
            util_error("Failed to create LOG dir");
        }
    }

    (void)closedir(directory);

}

void vInitializeLog(char * logFilePath, unsigned int filePathLength, char * csvLogFilePath, unsigned int csvFilePathLength)
{
    struct timeval tvTime;
    char logFileDirectoryPath[MAX_FILE_PATH];
    char DateBuffer[FILENAME_MAX];
    FILE *filefd, *fileread;
    char msString[10];
    char sysCommand[100];
    char pcBuffer[MAX_LOG_ROW_LENGTH];
    DIR *dir;
    struct dirent *ent;
    int read;

    // Clear the two path strings
    bzero(logFilePath, filePathLength);
    bzero(csvLogFilePath, csvFilePathLength);

    // Calculate the date and time when the logfile is created
    TimeSetToCurrentSystemTime(&tvTime);
    TimeGetAsDateTime(&tvTime, "%Y-%m-%d_%H.%M.%S", DateBuffer, sizeof(DateBuffer));

    // Append milliseconds
    sprintf(msString, ".%i", (int)tvTime.tv_usec/1000);
    strcat(DateBuffer, msString);

    // Create log folder named with initialization date and time
    (void)strcpy(logFileDirectoryPath, LOG_PATH);
    (void)strcat(logFileDirectoryPath, DateBuffer);

    vCreateLogFolder(logFileDirectoryPath);

    (void)strcat(logFileDirectoryPath,FORWARD_SLASH);

    // Copy configuration file to log directory
    LogMessage(LOG_LEVEL_INFO, "Copying configuration file to log directory");
    (void)strcpy(sysCommand, "cp ");
    (void)strcat(sysCommand, TEST_CONF_FILE);
    (void)strcat(sysCommand, " ");
    (void)strcat(sysCommand, logFileDirectoryPath);
    (void)system(sysCommand);

    // Check if ./traj directory exists
    if ((dir = opendir(TRAJECTORY_PATH)) == NULL)
        LogMessage(LOG_LEVEL_ERROR,"No traj directory <%s> exists - wrong path or access denied",TRAJECTORY_PATH);
    else
        closedir(dir);
    // Copy trajectory files to subdirectory
    LogMessage(LOG_LEVEL_INFO, "Copying trajectory files to log directory");
    (void)strcpy(sysCommand, "cp -R ");
    (void)strcat(sysCommand, TRAJECTORY_PATH);
    (void)strcat(sysCommand, " ");
    (void)strcat(sysCommand, logFileDirectoryPath);
    (void)system(sysCommand);

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

    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer,"------------------------------------------\nWhole Trajectory files:\n------------------------------------------\n");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

    // Open directory ./traj/
    if ((dir = opendir(TRAJECTORY_PATH)) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            // Copy all files in trajectory and add them to the log file
            bzero(pcBuffer, sizeof(pcBuffer));
            strcpy(pcBuffer, TRAJECTORY_PATH);
            strcat(pcBuffer, ent->d_name);
            if (access(pcBuffer, 0) == 0)
            {
                fileread = fopen(pcBuffer, ACCESS_MODE_READ);
                read = fgetc(fileread);
                while (read != EOF)
                {
                    fputc(read,filefd);
                    read = fgetc(fileread);
                }
                fclose(fileread);
            }
            else {
                LogMessage(LOG_LEVEL_ERROR,"Failed to open <%s>", pcBuffer);
            }
        }
        closedir(dir);
    }
    else
    {
        LogMessage(LOG_LEVEL_ERROR, "No traj directory <%s> exists - wrong path or access denied", TRAJECTORY_PATH);
    }
    fclose(filefd);

    // Print configuration to log
    LogMessage(LOG_LEVEL_DEBUG, "Printing configuration to log");
    filefd = fopen(logFilePath, ACCESS_MODE_APPEND_AND_READ);

    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer, "\n------------------------------------------\nWhole Config file:\n------------------------------------------\n");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);


    /* If file conf file exists and we have read permission do*/
    if (access(TEST_CONF_FILE, 0) == 0)
    {
      /*read the .conf file and print it in to the .log file */
      fileread = fopen(TEST_CONF_FILE, ACCESS_MODE_READ);
      read = fgetc(fileread);
      while(read!= EOF)
      {
          fputc(read,filefd);
          read = fgetc(fileread);
      }
      fclose(fileread);
    }
    else
    {
        LogMessage(LOG_LEVEL_ERROR, "Failed to open <%s>", TEST_CONF_FILE);
    }

    // Add some information about the standard log file format and what is what in the MONR message
    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer, "\n------------------------------------------\nInformation about log structure\n------------------------------------------\nLog started; Date:%s\nGenerall structure:\n",DateBuffer);
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);
    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer,"<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<Command message nr>;<Data>\n");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer,"Monor message structure(command message nr = 3):\n<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<GPS Time ms>;<Command message nr>;<Data>;<Object_address (IP number)>;<0>;");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer,"<GPS Second of week (unit 0.25 milliseconds)>;<x-position, unit 0.001 meter>;<y-position, unit 0.001 meter>;<z-position, unit0.001>;<heading, unit 0.01 degrees>;<Logitudinal speed,");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

    bzero(pcBuffer, sizeof(pcBuffer));

    sprintf(pcBuffer,"Version;%s\n",MaestroVersion);
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);

    sprintf(pcBuffer,"unit 0.01 m/s>;<Lateral speed, unit 0.01 m/s>;<Longitudinal Acceleration, unit 0.001 m/s^2>;<Lateral Acceleration, unit 0.001 m/s^2>;<Driving direction>;<Object state>;<Ready to ARM>;<ErrorState>\n"); // add more her if we want more data
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);
    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer, "Command message nr:\nCOMM_START:%d\nCOMM_STOP:%d\nCOMM_MONI%d\nCOMM_EXIT:%d\nCOMM_ARMD:%d\nCOMM_REPLAY:%d\nCOMM_CONTROL:%d\nCOMM_ABORT:%d\nCOMM_TOM:%d\nCOMM_INIT:%d\nCOMM_CONNECT:%d\nCOMM_OBC_STATE:%d\nCOMM_DISCONNECT:%d\nCOMM_LOG:%d\nCOMM_VIOP:%d\nCOMM_INV:%d\n------------------------------------------\n Log start\n------------------------------------------\n",COMM_STRT,COMM_STOP,COMM_MONI,COMM_EXIT,COMM_ARMD,COMM_REPLAY,COMM_CONTROL,COMM_ABORT,COMM_TOM,COMM_INIT,COMM_CONNECT,COMM_OBC_STATE,COMM_DISCONNECT,COMM_LOG,COMM_VIOP,COMM_INV);
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);


    fclose(filefd);
}
