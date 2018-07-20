
/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  --------------------------------------------------------------------------------
  -- File        : main.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS main
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
-- Include files.
------------------------------------------------------------*/
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>


#include "util.h"
#include "logger.h"
#include "objectcontrol.h"
#include "systemcontrol.h"
#include "supervision.h"
#include "remotecontrol.h"
#include "timecontrol.h"
#include "simulatorcontrol.h"
#include "citscontrol.h"

/*------------------------------------------------------------
-- Defines
------------------------------------------------------------*/

static TimeType *GPSTime;
static GSDType *GSD;
/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int main(int argc, char *argv[])
{

  /*Share time between child processes*/
  GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

  pid_t pID[8];
  int iIndex = 0;
  #ifdef DEBUG
    printf("INF: Central started\n");
    fflush(stdout);
  #endif

  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: logger_task running in:  %i \n",getpid());
    #endif
    logger_task();
    exit(EXIT_SUCCESS);
  }
  ++iIndex;


  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: supervision_task running in:  %i \n",getpid());
    #endif
    supervision_task(GPSTime);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;


  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: objectcontrol_task running in:  %i \n",getpid());
    #endif
    objectcontrol_task(GPSTime, GSD);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

  char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];
  bzero(pcTempBuffer,MAX_UTIL_VARIBLE_SIZE);
  if(iUtilGetParaConfFile("VisualizationAdapter",pcTempBuffer))
  {
    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
      util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {
      #ifdef DEBUG
        printf("INF: visualization 0 running in:  %i \n",getpid());
      #endif

      char *newargv[] = { NULL, NULL };
      char *newenviron[] = { NULL };
      newargv[0] = pcTempBuffer; 
      execve(pcTempBuffer, newargv, newenviron);
      util_error("ERR: Failed to create visualization adapter");
    }
    ++iIndex;
  }
 
  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: remotecontrol_task running in:  %i \n",getpid());
    #endif
    remotecontrol_task(GPSTime);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: timecontrol_task running in:  %i \n",getpid());
    #endif
    timecontrol_task(GPSTime, GSD);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

  pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: simulatorcontrol_task running in:  %i \n",getpid());
    #endif
    simulatorcontrol_task(GPSTime);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

 pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {
    #ifdef DEBUG
      printf("INF: citscontrol_task running in:  %i \n",getpid());
    #endif
    citscontrol_task(GPSTime);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

  #ifdef DEBUG
    printf("INF: systemcontrol_task running in:  %i \n",getpid());
  #endif
    
  systemcontrol_task(GPSTime, GSD);
}
