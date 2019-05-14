
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
#include "mqbus.h"



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

/*------------------------------------------------------------
-- Server modules
------------------------------------------------------------*/
//! allModules contains the tasks to be run in the server. To enable or disable a task, add or remove the main module function in this array
static const ModuleTask allModules[] = {
    logger_task,
    timecontrol_task,
    supervision_task,
    supervisorcontrol_task,
    systemcontrol_task,
    objectcontrol_task
};
static const size_t numberOfModules = sizeof(allModules) / sizeof(ModuleTask);

/*------------------------------------------------------------
-- Private functions
------------------------------------------------------------*/
int readArgumentList(int argc, char *argv[], Options *opts);
void printHelp(char* progName);
int initializeMessageQueueBus(void);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
#define MODULE_NAME "Central"
int main(int argc, char *argv[])
{
    unsigned int moduleNumber = 0;
    Options options;
    pid_t pID[numberOfModules];

    if (readArgumentList(argc, argv, &options))
        exit(EXIT_FAILURE);

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

    // Initialise message queue bus
    if(initializeMessageQueueBus())
        exit(EXIT_FAILURE);

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

/*!
 * \brief initialiseMessageQueueBus Wrapper function for MQBusInit with explanatory log printouts
 * \return 0 upon success, -1 upon failure
 */
int initializeMessageQueueBus(void)
{
    int nbrOfQueues = numberOfModules;
    enum MQBUS_ERROR result = MQBusInit(nbrOfQueues);

    // Printouts according to result
    switch (result)
    {
    case MQBUS_OK:
        LogMessage(LOG_LEVEL_DEBUG, "Initialized message queue bus");
        return 0;
    case MQBUS_INVALID_INPUT_ARGUMENT:
        LogMessage(LOG_LEVEL_ERROR, "Invalid number of message queues specified upon initialization: %d", nbrOfQueues);
        break;
    case MQBUS_QUEUE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Unable to initialize resource message queue: already exists");
        break;
    case MQBUS_RESOURCE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Resource message queue does not exist");
        break;
    case MQBUS_EMPTY:
        LogMessage(LOG_LEVEL_ERROR, "Message queue empty");
        break;
    case MQBUS_MAX_QUEUES_LIMIT_REACHED:
        LogMessage(LOG_LEVEL_ERROR, "Maximum number of message queues reached");
        break;
    case MQBUS_OPEN_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to open message queue");
        break;
    case MQBUS_CLOSE_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to close message queue");
        break;
    case MQBUS_DESCRIPTOR_NOT_FOUND:
        LogMessage(LOG_LEVEL_ERROR, "Message queue descriptor not found");
        break;
    case MQBUS_NO_READABLE_MQ:
        LogMessage(LOG_LEVEL_ERROR, "Connection to message queue bus does not exist");
        break;
    case MQBUS_MQ_COULD_NOT_BE_DESTROYED:
        LogMessage(LOG_LEVEL_ERROR, "Could not delete message queue");
        break;
    }
    return -1;
}


/*!
 * \brief readArgumentList Decodes the list of input arguments and fills an options struct
 * \param argc Length of argument vector
 * \param argv String array, starting with the executable name
 * \param opts Options struct
 * \return 0 upon success, -1 upon failure, 1 when clarification is needed
 */
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

/*!
 * \brief printHelp
 * \param progName
 */
void printHelp(char* progName)
{
    printf("Usage: %s [OPTION]\n", progName);
    printf("Runs all modules of test server.\n\n");
    printf("  -v, --verbose \tcreate detailed logs\n");
    printf("  -h, --help \t\tdisplay this help and exit\n");
}
