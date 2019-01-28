
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
#include "supervisorcontrol.h"
//#include "citscontrol.h"


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

    // Set the debug level
    // TODO: make debug level a starting parameter
    // make sure that the same debug parameter is passed to all processes
    dbg_setdebug(DEBUG_LEVEL_HIGH);

    /*Share time between child processes*/

    GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    GSD->ExitU8 = 0;
    GSD->ScenarioStartTimeU32 = 0;

    pid_t pID[8];
    int iIndex = 0;

    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: Central started\n");
    fflush(stdout);


    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
        util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {

        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: logger_task running in:  %i \n",getpid());

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
        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: supervision_task running in:  %i \n",getpid());

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
        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: objectcontrol_task running in:  %i \n",getpid());

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
            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: visualization 0 running in:  %i \n",getpid());


            char *newargv[] = { NULL, NULL };
            char *newenviron[] = { NULL };
            newargv[0] = pcTempBuffer;
            execve(pcTempBuffer, newargv, newenviron);
            util_error("ERR: Failed to create visualization adapter");
        }
        ++iIndex;
    }

/*
    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
      util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {

        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: remotecontrol_task running in:  %i \n",getpid());

      remotecontrol_task(GPSTime);
      exit(EXIT_SUCCESS);
    }
    ++iIndex;
*/

    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
        util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {

        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: timecontrol_task running in:  %i \n",getpid());

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

        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: simulatorcontrol_task running in:  %i \n",getpid());

      simulatorcontrol_task(GPSTime, GSD);
      exit(EXIT_SUCCESS);
    }
    ++iIndex;
  
  /*
 pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {

      DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: citscontrol_task running in:  %i \n",getpid());

    citscontrol_task(GPSTime, GSD);
    exit(EXIT_SUCCESS);
  }
  ++iIndex;
  
*/

    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
        util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {

        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: supervisorcontrol_task running in:  %i \n",getpid());
        supervisorcontrol_task(GPSTime, GSD);
        exit(EXIT_SUCCESS);
    }
    ++iIndex;
  


    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"INF: systemcontrol_task running in:  %i \n",getpid());

    
    systemcontrol_task(GPSTime, GSD);
}
