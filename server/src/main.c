
/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  --------------------------------------------------------------------------------
  -- File        : main.c
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
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

#include "logger.h"
#include "objectcontrol.h"
#include "systemcontrol.h"
#include "supervision.h"
#include "util.h"
#include "usercontrol.h"

/*------------------------------------------------------------
-- Defines
------------------------------------------------------------*/

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int main(int argc, char *argv[])
  {
  pid_t pID[4];
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
    supervision_task();
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
    objectcontrol_task();
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
    //#ifdef DEBUG
      printf("INF: usercontrol_task running in:  %i \n",getpid());
    //#endif
    usercontrol_task();
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
     // #ifdef DEBUG
        printf("INF: visualization 0 running in:  %i \n",getpid());
     // #endif

      char *newargv[] = { NULL, NULL };
      char *newenviron[] = { NULL };
      newargv[0] = pcTempBuffer; 
      execve(pcTempBuffer, newargv, newenviron);
      util_error("ERR: Failed to create visualization adapter");
    }
    ++iIndex;
  }
 
  //#ifdef DEBUG
    printf("INF: systemcontrol_task running in:  %i \n",getpid());
  //#endif
    
  systemcontrol_task();
}
