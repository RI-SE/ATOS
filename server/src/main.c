
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
#include "supervisorcontrol.h"
#include "logging.h"


/*------------------------------------------------------------
-- Defines
------------------------------------------------------------*/

static TimeType *GPSTime;
static GSDType *GSD;

#define MODULE_NAME "Central"
static LOG moduleLog;
static const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;
/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    // Set the debug level
    // TODO: make debug level a starting parameter
    // make sure that the same debug parameter is passed to all processes
    printf("Version %s\n",MaestroVersion );
    dbg_setdebug(DEBUG_LEVEL_HIGH);

    moduleLog = init_log(MODULE_NAME,logLevel);

    /*Share time between child processes*/

    GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    GSD->ExitU8 = 0;
    GSD->ScenarioStartTimeU32 = 0;
    GPSTime->TimeInitiatedU8 = 0;

    pid_t pID[8];
    int iIndex = 0;

    log_message(&moduleLog, LOG_LEVEL_INFO, "Central started");


    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
        util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {
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
        objectcontrol_task(GPSTime, GSD);
        exit(EXIT_SUCCESS);
    }
    ++iIndex;

  /*  char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];
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
            log_message(&moduleLog,LOG_LEVEL_INFO,"Visualization 0 running in:  %i",getpid());


            char *newargv[] = { NULL, NULL };
            char *newenviron[] = { NULL };
            newargv[0] = pcTempBuffer;
            execve(pcTempBuffer, newargv, newenviron);
            util_error("ERR: Failed to create visualization adapter");
        }
        ++iIndex;
    }
*/
/*
    pID[iIndex] = fork();
    if(pID[iIndex] < 0)
    {
      util_error("ERR: Failed to fork");
    }
    if(pID[iIndex] == 0)
    {

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
        timecontrol_task(GPSTime, GSD);
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

        log_message(&moduleLog,LOG_LEVEL_INFO,"simulatorcontrol_task running in:  %i",getpid());

      simulatorcontrol_task(GPSTime, GSD);
      exit(EXIT_SUCCESS);
    }
    ++iIndex;
  */

  /*
 pID[iIndex] = fork();
  if(pID[iIndex] < 0)
  {
    util_error("ERR: Failed to fork");
  }
  if(pID[iIndex] == 0)
  {

      log_message(&moduleLog,LOG_LEVEL_INFO,"citscontrol_task running in:  %i",getpid());

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
        supervisorcontrol_task(GPSTime, GSD);
        exit(EXIT_SUCCESS);
    }
    ++iIndex;

    systemcontrol_task(GPSTime, GSD);
}
