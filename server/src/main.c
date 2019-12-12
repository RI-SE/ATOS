
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
#include "datadictionary.h"
#include "timecontrol.h"
#include "supervisorcontrol.h"
#include "citscontrol.h"

#include "logging.h"
#include "mqbus.h"
#include "maestroTime.h"



/*------------------------------------------------------------
-- Types
------------------------------------------------------------*/
typedef void (*ModuleTask) (TimeType *, GSDType *, LOG_LEVEL);	//!< Function pointer type for module "main" functions
typedef struct {
	LOG_LEVEL commonLogLevel;	//!< Logging level of the server.
	int extraMessageQueues;		//!< Number of extra message queues to create on startup.
} Options;

/*------------------------------------------------------------
-- Defines
------------------------------------------------------------*/
#define PROCESS_WAIT_TIMEOUT_MS 3000	//!< Time to wait for all modules to exit before closing message bus
#define CHILD_POLL_PERIOD_S 2	//!< Time period (seconds) between process status polls when no shutdown signal has been received
#define CHILD_POLL_PERIOD_NS 0	//!< Time period (nanoseconds) between process status polls when no shutdown signal has been received
#define CHILD_POLL_PERIOD_SHUTDOWN_S 0	//!< Time period (seconds) between process status polls when a shutdown signal has been received
#define CHILD_POLL_PERIOD_SHUTDOWN_NS 1000*1000	//!< Time period (nanoseconds) between process status polls when a shutdown signal has been received

static TimeType *GPSTime;
static GSDType *GSD;
static struct timespec childPollPeriodTime = { CHILD_POLL_PERIOD_S, CHILD_POLL_PERIOD_NS };	//!< Time period between process polls

static char doShutdown = 0;		//!< Exit boolean
static struct timeval waitStartTime, waitedTime;	//!< Poll timeout timers

/*------------------------------------------------------------
-- Server modules
------------------------------------------------------------*/
//! allModules contains the tasks to be run in the server. To enable or disable a task, add or remove the main module function in this array
static const ModuleTask allModules[] = {

	logger_task,
	timecontrol_task,
	systemcontrol_task,
	objectcontrol_task
#ifdef CITS_ENABLED
		, citscontrol_task
#endif
};

static const int numberOfModules = sizeof (allModules) / sizeof (ModuleTask);

/*------------------------------------------------------------
-- Private functions
------------------------------------------------------------*/
int readArgumentList(int argc, char *argv[], Options * opts);
void printHelp(char *progName);
int initializeMessageQueueBus(Options * opts);
int shutdownMessageQueueBus(void);
int waitForModuleExit(pid_t pIDs[], unsigned int numberOfModules, char moduleExitStatus[]);
void signal_handler(int signo);

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
#define MODULE_NAME "Central"
int main(int argc, char *argv[]) {
	unsigned int moduleNumber = 0;
	Options options;
	pid_t pID[numberOfModules];
	char moduleExitStatus[numberOfModules];
	ReadWriteAccess_t dataDictInitResult = UNDEFINED;

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

	// Check file path validity
	if (UtilVerifyTestDirectory() == -1)
		util_error("Test directory could not be verified");

	// Initialise data dictionary
	LogMessage(LOG_LEVEL_INFO, "Initializing data dictionary");
	dataDictInitResult = DataDictionaryConstructor(GSD);
    if (dataDictInitResult != READ_OK && dataDictInitResult != READ_WRITE_OK){
		util_error("Unable to initialize shared memory space");
    }else {
        LogMessage(LOG_LEVEL_INFO, "Data dictionary succesfully initiated");
    }
	LogMessage(LOG_LEVEL_INFO, "About to enter mq init");

	// Initialise message queue bus
	if (initializeMessageQueueBus(&options))
		exit(EXIT_FAILURE);

	// For all modules in allModules, start corresponding process in a fork
	for (moduleNumber = 0; moduleNumber < numberOfModules; ++moduleNumber) {
		pID[moduleNumber] = fork();
		if (pID[moduleNumber] < 0) {
			util_error("Failed to fork");
		}
		else if (pID[moduleNumber] == 0) {
			// Call module task
			(*allModules[moduleNumber]) (GPSTime, GSD, options.commonLogLevel);
			exit(EXIT_SUCCESS);
		}
	}

	if (signal(SIGINT, signal_handler) == SIG_ERR)
		util_error("Unable to create signal handler");

	// Enter hold function while server is running
	(void)waitForModuleExit(pID, numberOfModules, moduleExitStatus);

	// Perform final cleanup
	LogMessage(LOG_LEVEL_DEBUG, "Cleaning up message bus resources");
	if (shutdownMessageQueueBus())
		util_error("Unable to successfully clean up message bus resources");
	else

		exit(EXIT_SUCCESS);
}

/*!
 * \brief initialiseMessageQueueBus Wrapper function for MQBusInit with explanatory log printouts
 * \param opts A pointer to the current options the program has. This function will only look for ::extraMessageQueues
 * \return 0 upon success, -1 upon failure
 */
int initializeMessageQueueBus(Options * opts) {

	// If the user supplied an additional number of modules in the input, this will create the appropriate amout of message queues.
	int nbrOfQueues = numberOfModules + opts->extraMessageQueues;
	enum MQBUS_ERROR result = MQBusInit(nbrOfQueues);

	// Printouts according to result
	switch (result) {
	case MQBUS_OK:
		LogMessage(LOG_LEVEL_DEBUG, "Initialized message queue bus");
		return 0;
	case MQBUS_INVALID_INPUT_ARGUMENT:
		LogMessage(LOG_LEVEL_ERROR, "Invalid number of message queues specified upon initialization: %d",
				   nbrOfQueues);
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
	case MQBUS_MQ_FULL:
		LogMessage(LOG_LEVEL_ERROR, "Message bus slot full");
		break;
	case MQBUS_WRITE_FAIL:
		LogMessage(LOG_LEVEL_ERROR, "Message bus write failed");
		break;
	}
	return -1;
}

/*!
 * \brief shutdownMessageQueueBus Deletes all message queues
 * \return 0 on success, -1 on error
 */
int shutdownMessageQueueBus(void) {
	enum MQBUS_ERROR result = MQBusDelete();

	// Printouts according to result
	switch (result) {
	case MQBUS_OK:
		LogMessage(LOG_LEVEL_DEBUG, "Deleted message queue bus");
		return 0;
	default:
		LogMessage(LOG_LEVEL_ERROR, "Error while deleting message bus");
		break;
	}
	return -1;
}

/*!
 * \brief waitForModuleExit When doShutdown has been set to 1, waits a preset number of seconds for all child
 * processes to exit cleanly, then returns
 * \param pIDs Array holding PIDs for child processes
 * \param numberOfModules Number of elements in pIDs
 * \param moduleExitStatus Exit status array to hold exit status of each child
 * \return 0 if all child processes exited cleanly, -1 otherwise
 */
int waitForModuleExit(pid_t pIDs[], unsigned int numberOfModules, char moduleExitStatus[]) {
	unsigned int moduleNumber, nExitedModules = 0;
	int status;

	bzero(moduleExitStatus, numberOfModules);
	LogMessage(LOG_LEVEL_INFO, "Awaiting shutdown signal...");
	do {
		// Loop over all modules which have not yet exited
		for (moduleNumber = 0; moduleNumber < numberOfModules; ++moduleNumber) {
			if (moduleExitStatus[moduleNumber])
				continue;

			switch (waitpid(pIDs[moduleNumber], &status, WNOHANG)) {
			case -1:
				util_error("wait() error");
			case 0:
				// Module still running
				break;
			default:
				// Module exited, record exit manner and increase
				if (WIFEXITED(status)) {
					moduleExitStatus[moduleNumber] = 1;
					nExitedModules++;
				}
				else {
					moduleExitStatus[moduleNumber] = -1;
					nExitedModules++;
				}
				break;
			}
		}

		// If shutdown is desired, wait specified amount of time before forcing return
		TimeSetToCurrentSystemTime(&waitedTime);
		timersub(&waitedTime, &waitStartTime, &waitedTime);

		if (doShutdown && TimeGetAsUTCms(&waitedTime) > PROCESS_WAIT_TIMEOUT_MS) {
			LogMessage(LOG_LEVEL_ERROR, "Timed out while waiting for modules to exit");
			break;
		}

		nanosleep(&childPollPeriodTime, NULL);
	} while (nExitedModules < numberOfModules);

	// Loop over all exited modules to report exit manner
	if (nExitedModules < numberOfModules) {
		LogMessage(LOG_LEVEL_WARNING, "Modules exited erroneously:");
		for (moduleNumber = 0; moduleNumber < numberOfModules; ++moduleNumber) {
			switch (moduleExitStatus[moduleNumber]) {
			case 0:
				LogMessage(LOG_LEVEL_WARNING, "PID %d did not exit during wait period of %.3f seconds",
						   pIDs[moduleNumber], PROCESS_WAIT_TIMEOUT_MS / 1000.0);
				break;
			case -1:
				LogMessage(LOG_LEVEL_WARNING, "PID %d exited with an error code", pIDs[moduleNumber]);
				break;
			}
		}
		return -1;
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "All modules exited successfully");
		return 0;
	}
}

/*!
 * \brief readArgumentList Decodes the list of input arguments and fills an options struct
 * \param argc Length of argument vector
 * \param argv String array, starting with the executable name
 * \param opts Options struct
 * \return 0 upon success, -1 upon failure, 1 when clarification is needed
 */
int readArgumentList(int argc, char *argv[], Options * opts) {
	enum ArgState {
		NO_STATE,
		NR_MODULES_INPUT
	};							//<! The available arguments states. This is used to track what the next arguments should be, after a user have given a command.

	char *progName = strrchr(argv[0], '/');

	if (progName == NULL) {
		progName = argv[0];
	}
	else {
		// Skip the slash
		if (progName[1] == '\0')
			return -1;
		else
			progName++;
	}

	// Initialise to default options
	opts->commonLogLevel = LOG_LEVEL_INFO;
	opts->extraMessageQueues = 0;


	int argState = NO_STATE;

	// Loop over all input arguments
	for (int i = 1; i < argc; ++i) {
		switch (argState) {
		case NR_MODULES_INPUT:
			opts->extraMessageQueues = atoi(argv[i]);
			argState = NO_STATE;
			continue;
		case NO_STATE:
		default:
			break;
		}

		if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			printHelp(progName);
			return 1;
		}
		else if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) {
			opts->commonLogLevel = LOG_LEVEL_DEBUG;
		}
		else if (!strcmp(argv[i], "-m") || !strcmp(argv[i], "--additional-modules")) {
			argState = NR_MODULES_INPUT;
		}
		else {
			// If option didn't match any known option
			printf("%s: invalid option -- '%s'\n", progName, argv[i]);
			printf("Try '%s --help' for more information.\n", argv[0]);
			return 1;
		}
	}

	if (argState != NO_STATE) {
		printf("%s: insufficient number of arguments.\n", progName);
		printf("Try '%s --help' for more information.\n", argv[0]);
		return 1;
	}

	return 0;
}

/*!
 * \brief printHelp Prints help text with list of all possible execution options
 * \param progName Name of the program as called
 */
void printHelp(char *progName) {
	printf("Usage: %s [OPTION]\n", progName);
	printf("Runs all modules of test server.\n\n");
	printf("  -v, --verbose \t\t\tcreate detailed logs\n");
	printf("  -h, --help \t\t\t\tdisplay this help and exit\n");
	printf
		("  -m, --additional-modules [number] \tenables the connection of a [number] of extra modules to connect.\n");
}

/*!
 * \brief signal_handler Handles signals (e.g. SIGINT) - forces exit if it catches an unimplemented signal
 * \param signo Signal number
 */
void signal_handler(int signo) {
	switch (signo) {
	case SIGINT:
		LogMessage(LOG_LEVEL_INFO, "Waiting for shutdown of all modules...");
		LogMessage(LOG_LEVEL_DEBUG, "Waiting %.3f seconds before forcing shutdown",
				   PROCESS_WAIT_TIMEOUT_MS / 1000.0);
		TimeSetToCurrentSystemTime(&waitStartTime);
		childPollPeriodTime.tv_sec = CHILD_POLL_PERIOD_SHUTDOWN_S;
		childPollPeriodTime.tv_nsec = CHILD_POLL_PERIOD_SHUTDOWN_NS;
		doShutdown = 1;
		break;
	default:
		util_error("Caught unhandled signal");
	}
}
