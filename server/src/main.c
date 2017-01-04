
/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  --------------------------------------------------------------------------------
  -- File        : main.c
  -- Author      : Karl-Johan Ode
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
#include <sys/types.h>
#include <unistd.h>

#include "logger.h"
#include "objectcontrol.h"
#include "visualization.h"
#include "systemcontrol.h"
#include "supervision.h"
#include "util.h"

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
    #ifdef DEBUG
      printf("INF: visualization_task running in:  %i \n",getpid());
    #endif
    visualization_task();
    exit(EXIT_SUCCESS);
  }
  ++iIndex;

  #ifdef DEBUG
    printf("INF: systemcontrol_task running in:  %i \n",getpid());
  #endif
  systemcontrol_task();
}