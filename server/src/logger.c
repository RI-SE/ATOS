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

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LOG_PATH "./log/"
#define LOG_FILE ".log" // lets use date instead
#define Forward_slash "/"
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
static int ReadLogLine(FILE *fd, char *Buffer);
static int CountFileRows(FILE *fd);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void logger_task()
{
    char pcCommand[100];
    char pcLogFolder[MAX_FILE_PATH];
    char pcLogFile[MAX_FILE_PATH];
    char pcLogFileComp[MAX_FILE_PATH];
    char pcBuffer[MQ_MAX_MESSAGE_LENGTH+100];
    char pcRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
    char TimeStampUTCBufferRecv[MQ_MAX_UTC_LENGTH];
    char DateBuffer[MQ_MAX_MESSAGE_LENGTH];
    char pcSendBuffer[MQ_MAX_MESSAGE_LENGTH];
    char pcReadBuffer[MQ_MAX_MESSAGE_LENGTH];
    char read;
    struct timeval tvTime ;
    uint64_t LogTimeStart;
    DIR *dir;
    struct dirent *ent;
    FILE *filefd,*fileread,*replayfd, *filefdComp;
    struct timespec sleep_time, ref_time;
    U8 FirstInitU8 = 0;
    gettimeofday(&tvTime,NULL);


    bzero(pcBuffer, MQ_MAX_MESSAGE_LENGTH+100);
    bzero(DateBuffer,MQ_MAX_MESSAGE_LENGTH);
    //Calculate the date when the logfile is created more or less
    LogTimeStart = UtilgetCurrentUTCtimeMS();
    UtilgetDateTimeFromUTCForMapNameCreation((int64_t)LogTimeStart, DateBuffer,sizeof(DateBuffer));

    bzero(pcLogFolder,MAX_FILE_PATH);
    bzero(pcLogFile,MAX_FILE_PATH);
    (void)strcpy(pcLogFolder,LOG_PATH);
    (void)strcat(pcLogFolder,DateBuffer);

    (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);
    //(void)iCommInit(IPC_SEND,MQ_LG_1,0);

    /* Create folder with date as name and .log file with date as name */
    vCreateLogFolder(pcLogFolder);
    (void)strcpy(pcLogFile,pcLogFolder);
    (void)strcat(pcLogFile,Forward_slash);
    (void)strcat(pcLogFile,DateBuffer);
    bzero(pcLogFileComp,MAX_FILE_PATH);
    (void)strcpy(pcLogFileComp,pcLogFile);
    (void)strcat(pcLogFileComp,"Csv");
    (void)strcat(pcLogFile,LOG_FILE);
    (void)strcat(pcLogFileComp,LOG_FILE);

    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: Open log file to use: <%s>\n",pcLogFile);
    filefd = fopen(pcLogFile, "w+");
    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer,"------------------------------------------\nWhole Trajectory files:\n------------------------------------------\n");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);


    /* Copy drive files */
    (void)strcpy(pcCommand,"cp -R ");
    (void)strcat(pcCommand,TRAJECTORY_PATH);
    (void)strcat(pcCommand," ");
    (void)strcat(pcCommand,pcLogFolder);
    (void)system(pcCommand);


    // Open directory ./traj/
    if ((dir=opendir(TRAJECTORY_PATH))!=NULL)
    {
      while((ent=readdir(dir))!=NULL)
      {
        //copy all files in trajectory and add them to the log file
        bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
        strcpy(pcBuffer,TRAJECTORY_PATH);
        strcat(pcBuffer,ent->d_name);
        if (0==access(pcBuffer,0))
        {
          fileread = fopen(pcBuffer,"r");
          read = fgetc(fileread);
          while(read != EOF)
          {
              fputc(read,filefd);
              read = fgetc(fileread);
          }
          fclose(fileread);
        }
      }
      closedir(dir);
    }
    else
    {
      perror("No traj folder exist, wrong path or access denied\n" );
    }
    /* If traj file exist and we have reader permission do*/


    /* Copy conf file */
    (void)strcpy(pcCommand,"cp ");
    (void)strcat(pcCommand,TEST_CONF_FILE);
    (void)strcat(pcCommand," ");
    (void)strcat(pcCommand,pcLogFolder);
    (void)system(pcCommand);

    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer, "------------------------------------------\nWhole Config file:\n------------------------------------------\n");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    /* If file conf file exist and we have reader permission do*/
    if (0==access(TEST_CONF_FILE,0))
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
        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"Cant open .conf file; %s\n",TEST_CONF_FILE);
        bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
        sprintf(pcBuffer,"Failed to Open .conf file;%s\n",TEST_CONF_FILE);
        (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    }

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

    // added some information about the standard log file format and what is what in the MONR message
    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer, "\n------------------------------------------\nInformation about log structure\n------------------------------------------\nLog started; Date:%s\nGenerall structure:\n",DateBuffer);
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer,"<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<Command message nr>;<Data>\n");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer,"Monor message structure(command message nr = 3):\n<Year>;<Month>;<Day>;<Hour>;<Minute>;<Second>;<Millisecond>;<UTC Time ms>;<Command message nr>;<Data>;<Object_address (IP number)>;<0>;");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer,"<GPS Second of week (unit 0.25 milliseconds)>;<x-position, unit 0.001 meter>;<y-position, unit 0.001 meter>;<z-position, unit0.001>;<heading, unit 0.01 degrees>;<Logitudinal speed,");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);

    sprintf(pcBuffer,"Version;%s\n",MaestroVersion);
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

    sprintf(pcBuffer,"unit 0.01 m/s>;<Lateral speed, unit 0.01 m/s>;<Longitudinal Acceleration, unit 0.001 m/s^2>;<Lateral Acceleration, unit 0.001 m/s^2>;<Driving direction>;<Object state>;<Ready to ARM>;<ErrorState>\n"); // add more her if we want more data
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf(pcBuffer, "Command message nr:\nCOMM_START:%d\nCOMM_STOP:%d\nCOMM_MONI%d\nCOMM_EXIT:%d\nCOMM_ARMD:%d\nCOMM_REPLAY:%d\nCOMM_CONTROL:%d\nCOMM_ABORT:%d\nCOMM_TOM:%d\nCOMM_INIT:%d\nCOMM_CONNECT:%d\nCOMM_OBC_STATE:%d\nCOMM_DISCONNECT:%d\nCOMM_LOG:%d\nCOMM_VIOP:%d\nCOMM_INV:%d\n------------------------------------------\n Log start\n------------------------------------------\n",COMM_STRT,COMM_STOP,COMM_MONI,COMM_EXIT,COMM_ARMD,COMM_REPLAY,COMM_CONTROL,COMM_ABORT,COMM_TOM,COMM_INIT,COMM_CONNECT,COMM_OBC_STATE,COMM_DISCONNECT,COMM_LOG,COMM_VIOP,COMM_INV);
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
    filefdComp = fopen(pcLogFileComp,"w+");

    while(!iExit)
    {
        bzero(pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
        bzero(TimeStampUTCBufferRecv,MQ_MAX_UTC_LENGTH);
        (void)iCommRecv(&iCommand,pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH,TimeStampUTCBufferRecv);
        if(LoggerExecutionMode == LOG_CONTROL_MODE && iCommand!=COMM_OBC_STATE)
        {

            Timestamp = atol(TimeStampUTCBufferRecv);
            bzero(DateBuffer,MQ_MAX_MESSAGE_LENGTH);
            UtilgetDateTimefromUTCCSVformat ((int64_t) Timestamp, DateBuffer,sizeof(DateBuffer));
            bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
            sprintf ( pcBuffer,"%s;%s;%d;%s\n", DateBuffer,TimeStampUTCBufferRecv, iCommand, pcRecvBuffer);
            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
            (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefdComp);

        }

        if(iCommand == COMM_REPLAY)
        {
            LoggerExecutionMode = LOG_REPLAY_MODE;
            printf("Logger in REPLAY mode <%s>\n", pcRecvBuffer);
            //replayfd = fopen ("log/33/event.log", "r");
            replayfd = fopen (pcRecvBuffer, "r");
            RowCount = UtilCountFileRows(replayfd);
            fclose(replayfd);
            //replayfd = fopen ("log/33/event.log", "r");
            replayfd = fopen (pcRecvBuffer, "r");
            printf("Rows %d\n", RowCount);
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
                    printf("%d:%d:%d<%s>\n", RowCount, j, SpecChars, pcSendBuffer);

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
                printf("Failed to open file:%s\n",  pcRecvBuffer);
            }

            printf("Replay done.\n");
            //(void)iCommInit(IPC_RECV_SEND,MQ_LG,0);
            (void)iCommSend(COMM_CONTROL, NULL);

        }
        else if(iCommand == COMM_CONTROL)
        {
            LoggerExecutionMode = LOG_CONTROL_MODE;
            printf("Logger in CONTROL mode\n");
        }
        else if(iCommand == COMM_EXIT)
        {

            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"%s","Logger exit\n");

            iExit = 1;
        }
        else if (iCommand == COMM_INIT)
        {
            /*
            if(FirstInitU8 == 1)
            {
                // Create folder and event.log file
                vCreateLogFolder(pcLogFolder);
                (void)strcpy(pcLogFile,pcLogFolder);
                (void)strcat(pcLogFile,LOG_FILE);

                DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: Open log file to use: <%s>\n",pcLogFile);
                filefd = fopen (pcLogFile, "w+");

                bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
                strcpy(pcBuffer, "Log started...\n");
                (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

                // Copy drive files
                (void)strcpy(pcCommand,"cp -R ");
                (void)strcat(pcCommand,TRAJECTORY_PATH);
                (void)strcat(pcCommand," ");
                (void)strcat(pcCommand,pcLogFolder);
                (void)system(pcCommand);

                // Copy conf file
                (void)strcpy(pcCommand,"cp ");
                (void)strcat(pcCommand,TEST_CONF_FILE);
                (void)strcat(pcCommand," ");
                (void)strcat(pcCommand,pcLogFolder);
                (void)system(pcCommand);
            }
            FirstInitU8 = 1;*/
        }
        else
        {
            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"Unhandled command in logger: %d",iCommand);
        }
    }


    (void)iCommClose();

    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    strcpy(pcBuffer, "Log closed\n");
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefdComp);

    fclose(filefd);
    fclose(filefdComp);
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
    if(directory == NULL)
    {
        iResult = mkdir(logFolder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (iResult < 0)
        {
            util_error("ERR: Failed to create LOG dir");
        }
    }

  /*  while (directory_entry = readdir(directory))
    {
        // Check so it's not . or ..
        if (strncmp(directory_entry->d_name,".",1))
        {

            int iTemp = atoi(directory_entry->d_name);

            if( iTemp > iMaxFolder)
            {
                iMaxFolder = iTemp;
            }
        }
    }*/
    (void)closedir(directory);

    /* step up one to create a new folder */
    //++iMaxFolder;
    //bzero(logFolder,MAX_FILE_PATH);
    //sprintf(logFolder,"%s%d/",LOG_PATH,iMaxFolder);

  /*  iResult = mkdir(logFolder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (iResult < 0)
    {
        util_error("ERR: Failed to create dir");
    }*/
}
