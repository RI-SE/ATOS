/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : logger.c
  -- Author      : Karl-Johan Ode
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

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LOG_PATH "./log/"
#define LOG_FILE "event.log"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vCreateLogFolder(char logFolder[MAX_FILE_PATH]);

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
  char pcBuffer[MQ_MAX_MESSAGE_LENGTH+100];
  char pcRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  struct timeval tvTime;
  FILE *filefd;

  (void)iCommInit(IPC_RECV,MQ_LG,0);

  /* Create folder and event.log file */
  vCreateLogFolder(pcLogFolder);
  (void)strcpy(pcLogFile,pcLogFolder);
  (void)strcat(pcLogFile,LOG_FILE);

  //#ifdef DEBUG
    printf("INF: Open log file to use: <%s>\n",pcLogFile);
    fflush(stdout);
  //#endif
  filefd = fopen (pcLogFile, "w+");

  bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
  strcpy(pcBuffer, "Log started...\n");
  (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

  /* Copy drive files */
  (void)strcpy(pcCommand,"cp -R ");
  (void)strcat(pcCommand,TRAJECTORY_PATH);
  (void)strcat(pcCommand," ");
  (void)strcat(pcCommand,pcLogFolder);
  (void)system(pcCommand);

  /* Copy conf file */
  (void)strcpy(pcCommand,"cp ");
  (void)strcat(pcCommand,TEST_CONF_FILE);
  (void)strcat(pcCommand," ");
  (void)strcat(pcCommand,pcLogFolder);
  (void)system(pcCommand);

  /* Listen for commands */
  int iExit = 0;
  int iCommand;
  while(!iExit)
  {
    bzero(pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
    (void)iCommRecv(&iCommand,pcRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
    
    /* Write time, command, buffer */
    gettimeofday(&tvTime, NULL);
    uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - 
      MS_FROM_1970_TO_2004_NO_LEAP_SECS + 
      NBR_LEAP_SECONDS_FROM_1970*1000;
    bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
    sprintf ( pcBuffer,"%" PRIu64 ": %d %s\n",uiTime,iCommand,pcRecvBuffer);
    (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

	  if(iCommand == COMM_EXIT)
    {
      iExit = 1;  
    }
    else
    {
      #ifdef DEBUG
        printf("Unhandled command in logger\n");
      #endif
    }
  }

  (void)iCommClose();

  bzero(pcBuffer,MQ_MAX_MESSAGE_LENGTH+100);
  strcpy(pcBuffer, "Log closed...\n");
  (void)fwrite(pcBuffer,1,strlen(pcBuffer),filefd);

  fclose(filefd);
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

  directory = opendir(LOG_PATH);
  if(directory == NULL)
  {
    iResult = mkdir(LOG_PATH, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (iResult < 0)
    {
      util_error("ERR: Failed to create LOG dir");
    }
  }

  while (directory_entry = readdir(directory))
  {
    
    /* Check so it's not . or .. */
    if (strncmp(directory_entry->d_name,".",1))
    {
      
      int iTemp = atoi(directory_entry->d_name);

      if( iTemp > iMaxFolder)
      {
        iMaxFolder = iTemp;
      }
    }
  }
  (void)closedir(directory);

  /* step up one to create a new folder */
  ++iMaxFolder;
  bzero(logFolder,MAX_FILE_PATH);
  sprintf(logFolder,"%s%d/",LOG_PATH,iMaxFolder);

  iResult = mkdir(logFolder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (iResult < 0)
  {
    util_error("ERR: Failed to create dir");
  }
}