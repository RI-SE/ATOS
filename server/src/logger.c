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
#include <mqueue.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#include "util.h"
#include "logging.h"
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
#define MQ_MAX_UTC_LENGTH 30
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
static const LOG_LEVEL logLevel = LOG_LEVEL_INFO;

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void logger_task()
{
    char pcLogFile[MAX_FILE_PATH];
    char pcLogFileComp[MAX_FILE_PATH];
    char pcBuffer[MQ_MAX_MESSAGE_LENGTH+100];
    char pcRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
    char TimeStampUTCBufferRecv[MQ_MAX_UTC_LENGTH];
    char DateBuffer[MQ_MAX_MESSAGE_LENGTH];
    char pcSendBuffer[MQ_MAX_MESSAGE_LENGTH];
    char pcReadBuffer[MQ_MAX_MESSAGE_LENGTH];

    LogInit(MODULE_NAME,logLevel);
    LogMessage(LOG_LEVEL_INFO,"Logger task running with PID: %d",getpid());

    int GPSweek;
    struct timeval tvTime ;
    uint64_t LogTimeStart;
    DIR *dir;
    struct dirent *ent;
    FILE *filefd,*fileread,*replayfd, *filefdComp;
    struct timespec sleep_time, ref_time;
    U8 isFirstInit = 1;
    TimeSetToCurrentSystemTime(&tvTime);

    (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);
    //(void)iCommInit(IPC_SEND,MQ_LG_1,0);

    /* Create folder with date as name and .log file with date as name */

    struct stat st = {0};
    if (stat(LOG_PATH, &st) == -1)
    {
        LogMessage(LOG_LEVEL_INFO,"Nonexistent log directory - will be created");
        vCreateLogFolder(LOG_PATH);
    }

    (void)strcat(pcLogFile," ");
    (void)strcat(pcLogFileComp," ");


    /* Listen for commands */
    int iExit = 0;
    int iCommand;

    /* Execution mode*/
    int LoggerExecutionMode = LOG_CONTROL_MODE;
    //int test =100;
    int RowCount = 0;
    // our time
    char *find_time;
    char *src;
    uint64_t NewTimestamp, OldTimestamp,Timestamp;

    //vCreateLogFolder("./log/");

    while(!iExit)
    {
        bzero(pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
        bzero(TimeStampUTCBufferRecv,MQ_MAX_UTC_LENGTH);
        (void)iCommRecv(&iCommand,pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH,TimeStampUTCBufferRecv);

        if(LoggerExecutionMode == LOG_CONTROL_MODE && iCommand!=COMM_OBC_STATE && iCommand!=COMM_MONI )
        {
            Timestamp = atol(TimeStampUTCBufferRecv);
            bzero(DateBuffer,MQ_MAX_MESSAGE_LENGTH);
            UtilgetDateTimefromUTCCSVformat ((int64_t) Timestamp, DateBuffer,sizeof(DateBuffer));
            bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);

            //Remove newlines in http Requests for nicer printouts.
            for (int i = 0; i < strlen(pcRecvBuffer); i++){
                if(pcRecvBuffer[i] == '\n'){
                  pcRecvBuffer[i] = ' ';
                }
            }

            sprintf ( pcBuffer,"%s;%s;%d;%s\n", DateBuffer,TimeStampUTCBufferRecv, iCommand, pcRecvBuffer);
            filefd = fopen(pcLogFile,"a");
            filefdComp = fopen(pcLogFileComp,"a");

            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefdComp);

            fclose(filefd);
            fclose(filefdComp);

        }

        switch(iCommand)
        {
        case COMM_DISCONNECT:

            isFirstInit = 1;
            break;

        case COMM_ABORT:

            isFirstInit = 1;
            break;

        case COMM_MONI:

            filefd = fopen(pcLogFile, "a+");

            char *str;
            str = malloc(sizeof(pcRecvBuffer) + 1);
            strcpy(str,pcRecvBuffer);

            char* GPSSecondOfWeek = strtok(str, ";");

            int counter = 0;
            while (GPSSecondOfWeek != NULL && counter < 2)  // Get GPS second of week
            {
              //printf("%s\n", token);
              GPSSecondOfWeek = strtok(NULL, ";");
              counter++;
            }

            uint64_t GPSms = UtilgetGPSmsFromUTCms(UtilgetUTCmsFromGPStime(GPSweek, atoi(GPSSecondOfWeek))); //Calculate GPSms

            Timestamp = atol(TimeStampUTCBufferRecv);
            bzero(DateBuffer,MQ_MAX_MESSAGE_LENGTH);
            UtilgetDateTimefromUTCCSVformat ((int64_t) Timestamp, DateBuffer,sizeof(DateBuffer));
            bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
            sprintf ( pcBuffer,"%s;%s;%lu;%d;%s\n", DateBuffer,TimeStampUTCBufferRecv, GPSms, iCommand, pcRecvBuffer);

            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
            //void)fwrite(pcBuffer,1,strlen(pcBuffer),filefdComp);

            fclose(filefd);

            break;

        case COMM_OSEM:

            str = malloc(sizeof(pcRecvBuffer) + 1);
            strcpy(str,pcRecvBuffer);

            // Returns first datapoint of OSEM (GPSWeek)
            char* token = strtok(pcRecvBuffer, ";");
            GPSweek = atoi(token);

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
            LogMessage(LOG_LEVEL_INFO,"Logger in REPLAY mode <%s>",pcRecvBuffer);
            //replayfd = fopen ("log/33/event.log", "r");
            replayfd = fopen (pcRecvBuffer, "r");
            RowCount = UtilCountFileRows(replayfd);
            fclose(replayfd);
            //replayfd = fopen ("log/33/event.log", "r");
            replayfd = fopen (pcRecvBuffer, "r");
            LogMessage(LOG_LEVEL_INFO,"Rows: %d",RowCount);;
            if(replayfd)
            {
                UtilReadLineCntSpecChars(replayfd, pcReadBuffer);//Just read first line
                int SpecChars = 0, j=0;
                char TimestampBuffer[TIMESTAMP_BUFFER_LENGTH];
                int FirstIteration = 1;
                //char *src;
                //uint64_t NewTimestamp, OldTimestamp;
                do
                {
                    bzero(pcReadBuffer,MQ_MAX_MESSAGE_LENGTH);
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
                        bzero(pcSendBuffer,MQ_MAX_MESSAGE_LENGTH);
                        //strcpy(pcSendBuffer, "MONR;");
                        strcat(pcSendBuffer, src+1);
                        (void)iCommSend(COMM_MONI, pcSendBuffer);
                        FirstIteration = 0;
                        OldTimestamp = NewTimestamp;
                    };
                    LogMessage(LOG_LEVEL_INFO,"%d:%d:%d<%s>",RowCount,j,SpecChars,pcSendBuffer);

                    /*
                    bzero(TimeStampUTCBufferRecv,MQ_ETSI_LENGTH);
                    (void)iCommRecv(&iCommand,pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH,TimeStampUTCBufferRecv);

                    if(iCommand == COMM_STOP)
                    {
                        printf("Replay stopped by user.\n");
                        (void)iCommSend(COMM_CONTROL, NULL);
                    }*/

                } while(--RowCount >= 0);

            }
            else
            {
                LogMessage(LOG_LEVEL_WARNING, "Failed to open file: %s", pcRecvBuffer);
            }

            LogMessage(LOG_LEVEL_INFO,"Replay done");
            //(void)iCommInit(IPC_RECV_SEND,MQ_LG,0);
            (void)iCommSend(COMM_CONTROL, NULL);

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

        default:
            LogMessage(LOG_LEVEL_WARNING,"Unhandled command in logger: %d",iCommand);
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
    char pcBuffer[MQ_MAX_MESSAGE_LENGTH+100];
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
    filefd = fopen(logFilePath, "w+");

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
                fileread = fopen(pcBuffer, "r");
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
    filefd = fopen(logFilePath,"a+");

    bzero(pcBuffer, sizeof(pcBuffer));
    sprintf(pcBuffer, "\n------------------------------------------\nWhole Config file:\n------------------------------------------\n");
    (void)fwrite(pcBuffer, 1, strlen(pcBuffer), filefd);


    /* If file conf file exists and we have read permission do*/
    if (access(TEST_CONF_FILE, 0) == 0)
    {
      /*read the .conf file and print it in to the .log file */
      fileread = fopen(TEST_CONF_FILE,"r");
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

