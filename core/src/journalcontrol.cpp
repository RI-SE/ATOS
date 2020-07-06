/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2020 AstaZero
  ------------------------------------------------------------------------------
  -- File        : journalcontrol.cpp
  -- Author      : Lukas Wikander
  -- Description :
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <signal.h>
#include <vector>
#include <unordered_set>

#include "journalcontrol.h"


/*------------------------------------------------------------
  -- Definitions.
  ------------------------------------------------------------*/
#define MODULE_NAME "JournalControl"
class Journal {
public:
	Journal();
};
/*------------------------------------------------------------
  -- Static variables.
  ------------------------------------------------------------*/
static volatile bool quit = false;

/*------------------------------------------------------------
  -- Static function declarations.
  ------------------------------------------------------------*/
static void signalHandler(int signo);
static int initializeModule(LOG_LEVEL logLevel);
static void storeJournalBookmarks(std::unordered_set<Journal> &journals);

/*------------------------------------------------------------
  -- Main task.
  ------------------------------------------------------------*/
void journalcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	std::vector<char> mqReceiveBuffer(MBUS_MAX_DATALEN, 0);
	std::vector<char> mqSendBuffer(MBUS_MAX_DATALEN, 0);
	enum COMMAND command = COMM_INV;
	ssize_t receivedBytes = 0;
	struct timeval recvTime;

	std::unordered_set<Journal> journals;

	// Initialize
	if (initializeModule(logLevel) < 0) {
		util_error("Failed to initialize module");
	}

	while (!quit) {
		std::fill(mqReceiveBuffer.begin(), mqReceiveBuffer.end(), 0);
		receivedBytes = iCommRecv(&command, mqReceiveBuffer.data(), mqReceiveBuffer.size(), &recvTime);

		switch (command) {
		case COMM_STRT:
			// TODO: Save start references
			storeJournalBookmarks(journals);
			break;
		case COMM_STOP:
		case COMM_ABORT:
			// Temporary: Treat ABORT as stop signal
			// TODO: Save stop references
			// TODO: Merge journals into named

		case COMM_GETSTATUS:
			std::fill(mqSendBuffer.begin(), mqSendBuffer.end(), 0);
			snprintf(mqSendBuffer.data(), mqSendBuffer.size(), "%s", MODULE_NAME);
			mqSendBuffer.back() = '\0';
			if (iCommSend(COMM_GETSTATUS_OK, mqSendBuffer.data(), mqSendBuffer.size()) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending status reply");
			}
			break;
		case COMM_EXIT:
			quit = true;
			break;
		// Do nothing with these messages
		case COMM_GETSTATUS_OK:
			break;
		}
	}

	iCommClose();

	LogMessage(LOG_LEVEL_INFO, "Logger exiting");
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

int initializeModule(LOG_LEVEL logLevel) {
	int retval = 0;
	struct timespec sleepTimePeriod, remTime;
	sleepTimePeriod.tv_sec = 0;
	sleepTimePeriod.tv_nsec = 1000000;
	int maxRetries = 10, retryNumber;

	// Initialize log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Logger task running with PID: %d", getpid());

	// Set up signal handlers
	LogMessage(LOG_LEVEL_DEBUG, "Initializing signal handler");
	if (signal(SIGINT, signalHandler) == SIG_ERR) {
		perror("signal");
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to initialize signal handler");
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

	return retval;
}

void storeJournalBookmarks(std::unordered_set<Journal> &journals) {
	journals.clear();

	// TODO look in journals dir for how many journals
	// TODO store line references for each
}
