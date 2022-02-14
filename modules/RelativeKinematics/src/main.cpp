#include "logging.h"
#include "util.h"
#include "RelativeKinematicsModule.hpp"
#include "journal.h"
#include "datadictionary.h"

static void signalHandler(int signo);
static int initializeModule(const LOG_LEVEL logLevel);

static bool quit = false;

int main(int argc, char **argv) {
	const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;
	constexpr int rosMessageCheckRate_Hz = 10;
	if (initializeModule(logLevel) < 0) {
		util_error("Failed to initialize module");
	}
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<RelativeKinematicsModule>(MODULE_NAME));
	rclcpp::shutdown();

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
/*
int initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;

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

	if (DataDictionaryInitStateData() != READ_OK
			|| DataDictionaryInitObjectData() != READ_OK) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory");
	}

	return retval;
}
*/
int initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;

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
	if (DataDictionaryInitStateData() != READ_OK) {
		DataDictionaryFreeStateData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for state data");
	}
	if (DataDictionaryInitObjectData() != READ_OK) {
		DataDictionaryFreeObjectData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for object data");
	}

	return retval;
}
