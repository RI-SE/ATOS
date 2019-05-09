
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
#include <string.h>
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
//#include "remotecontrol.h"
#include "timecontrol.h"
#include "supervisorcontrol.h"
#include "logging.h"

/*------------------------------------------------------------
-- Types
------------------------------------------------------------*/
typedef void (*ModuleTask)(TimeType*, GSDType*, LOG_LEVEL); //! Function pointer type for module "main" functions
typedef struct
{
    LOG_LEVEL commonLogLevel;
} Options;

/*------------------------------------------------------------
-- Defines
------------------------------------------------------------*/

static TimeType *GPSTime;
static GSDType *GSD;

static const ModuleTask allModules[] = {logger_task, timecontrol_task, supervision_task, supervisorcontrol_task, systemcontrol_task, objectcontrol_task};
static const size_t numberOfModules = sizeof(allModules) / sizeof(ModuleTask);

#define MODULE_NAME "Central"

/*------------------------------------------------------------
-- Private functions
------------------------------------------------------------*/
int readArgumentList(int argc, char *argv[], Options *opts);
void printHelp(char* progName);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    unsigned int moduleNumber = 0;
    Options options;
    pid_t pID[numberOfModules];

    if (readArgumentList(argc, argv, &options))
        return 0;

    // Share time between child processes
    GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

    GSD->ExitU8 = 0;
    GSD->ScenarioStartTimeU32 = 0;
    GPSTime->isTimeInitializedU8 = 0;

    // Initialise log
    LogInit(MODULE_NAME, options.commonLogLevel);
    LogPrint("Version %s", MaestroVersion);
    LogMessage(LOG_LEVEL_INFO, "Central started");
    LogMessage(LOG_LEVEL_DEBUG, "Verbose mode enabled");

    // For all modules in allModules, start corresponding process in a fork
    for (moduleNumber = 0; moduleNumber < numberOfModules-1; ++moduleNumber)
    {
        pID[moduleNumber] = fork();
        if (pID[moduleNumber] < 0)
        {
            util_error("Failed to fork");
        }
        else if (pID[moduleNumber] == 0)
        {
            // Call module task
            (*allModules[moduleNumber])(GPSTime, GSD, options.commonLogLevel);
            exit(EXIT_SUCCESS);
        }
    }

    // Use the main fork for the final task
    (*allModules[moduleNumber])(GPSTime, GSD, options.commonLogLevel);
    exit(EXIT_SUCCESS);
}


int readArgumentList(int argc, char *argv[], Options *opts)
{
    char *progName = strrchr(argv[0],'/');
    if (progName == NULL)
    {
        progName = argv[0];
    }
    else
    {
        // Skip the slash
        if (progName[1] == '\0')
            return -1;
        else
            progName++;
    }

    // Initialise to default options
    opts->commonLogLevel = LOG_LEVEL_INFO;



    // Loop over all input arguments
    for(int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
        {
            printHelp(progName);
            return 1;
        }
        else if (!strcmp(argv[i], "-v") || !strcmp(argv[i],"--verbose"))
        {
            opts->commonLogLevel = LOG_LEVEL_DEBUG;
        }
        else
        {
            // If option didn't match any known option
            printf("%s: invalid option -- '%s'\n", progName, argv[i]);
            printf("Try '%s --help' for more information.\n", argv[0]);
            return 1;
        }
    }
    return 0;
}

void printHelp(char* progName)
{
    printf("Usage: %s [OPTION]\n", progName);
    printf("Runs all modules of test server.\n\n");
    printf("  -v, --verbose \tcreate detailed logs\n");
    printf("  -h, --help \t\tdisplay this help and exit\n");
}
