#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "state.hpp"
#include "logging.h"
#include "util.h"
#include "journal.h"
#include "datadictionary.h"

#define MODULE_NAME "RelativeKinematics"

static void signalHandler(int signo);
static int initializeModule(const LOG_LEVEL logLevel);

static bool quit = false;

int main() {
	COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0,10000000};
	const struct timespec abortWaitTime = {1,0};
	struct timespec remTime;
	const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;

	std::string statusReply = MODULE_NAME;

	// Initialize
	if (initializeModule(logLevel) < 0) {
		util_error("Failed to initialize module");
	}

	ScenarioHandler scenarioHandler(ScenarioHandler::RELATIVE_KINEMATICS);
	while (!quit) {
		if (iCommRecv(&command, mqRecvData, MQ_MSG_SIZE, nullptr) < 0) {
			util_error("Message bus receive error");
		}

		switch (command) {
		case COMM_INV:
			nanosleep(&sleepTimePeriod,&remTime);
			break;
		case COMM_OBC_STATE:
			break;
		case COMM_GETSTATUS:
			iCommSend(COMM_GETSTATUS_OK, statusReply.c_str(), statusReply.size()+1);
			break;
		case COMM_GETSTATUS_OK:
			break;
		case COMM_INIT:
			try {
				scenarioHandler.handleInitCommand();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Initialization failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_CONNECT:
			try {
				scenarioHandler.handleConnectCommand();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Connection failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_DISCONNECT:
			try {
				scenarioHandler.handleDisconnectCommand();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Disconnection failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_ARM:
			try {
				scenarioHandler.handleArmCommand();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Arm failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_STRT:
			try {
				// TODO set start delay
				scenarioHandler.handleStartCommand();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Start failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_ABORT:
			// Any exceptions here should crash the program
			scenarioHandler.handleAbortCommand();
			break;
		case COMM_ABORT_DONE:
			//Abort is done at object level
			try	{ 
				scenarioHandler.handleAllClearCommand();
			} catch(std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Failed clear abort - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}		
			break;
		case COMM_ACCM:
			// TODO
			scenarioHandler.handleActionConfigurationCommand();
			break;
		case COMM_EXAC:
			// TODO
			break;
		default:
			LogMessage(LOG_LEVEL_INFO, "Received command %u", command);
		}
	}

	return 0;
}




void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		quit = true;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}

/*!
 * \brief initializeModule Initializes this module by creating log, connecting to the message queue bus,
 *			setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;
	struct timespec sleepTimePeriod, remTime;
	sleepTimePeriod.tv_sec = 0;
	sleepTimePeriod.tv_nsec = 1000000;
	int maxRetries = 10, retryNumber;

	// Initialize log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, MODULE_NAME " task running with PID: %d", getpid());

	// Set up signal handlers
	LogMessage(LOG_LEVEL_DEBUG, "Initializing signal handler");
	if (signal(SIGINT, signalHandler) == SIG_ERR) {
		perror("signal");
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to initialize signal handler");
	}

	// Create test journal
	if (JournalInit(MODULE_NAME) == -1) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to create test journal");
	}

	// Initialize message bus connection
	LogMessage(LOG_LEVEL_DEBUG, "Initializing connection to message bus");
	for (retryNumber = 0; iCommInit() != 0 && retryNumber < maxRetries; ++retryNumber) {
		nanosleep(&sleepTimePeriod, &remTime);
	}
	if (retryNumber == maxRetries) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to initialize connection to message bus");
	}
	else {
		if (DataDictionaryInitStateData() != READ_OK
				|| DataDictionaryInitObjectData() != READ_OK) {
			retval = -1;
			LogMessage(LOG_LEVEL_ERROR,
					   "Found no previously initialized shared memory");
		}
	}

	return retval;
}
