#include <iostream>
#include <unistd.h>
#include <systemd/sd-daemon.h>
#include <csignal>
#include "braketrigger.h"
#include "trigger.h"
#include "datadictionary.h"
#include "maestroTime.h"
#include "scenario.h"
#include "logging.h"
#include "util.h"
#include "journal.h"

#define MODULE_NAME "ScenarioControl"
#define SHMEM_READ_RATE_HZ 100

void updateObjectCheckTimer(struct timeval *currentSHMEMReadTime, uint8_t SHMEMReadRate_Hz);
int updateTriggers(Scenario& scenario);
static void signalHandler(int signo);

/*------------------------------------------------------------
  -- Static variables
  ------------------------------------------------------------*/
static bool quit = false;
/************************ Main task ******************************************/
int main()
{
	COMMAND command = COMM_INV;
	char mqRecvData[MBUS_MAX_DATALEN];
	char mqSendData[MBUS_MAX_DATALEN];
	ssize_t recvDataLength = 0;
	const struct timespec sleepTimePeriod = {0,10000000};
	struct timespec remTime;
	Scenario scenario;
	TREOData treo;
	ObjectDataType monr;
	bool terminate = false;
	char configPath[MAX_FILE_PATH];
	UtilGetConfDirectoryPath(configPath, sizeof(configPath));
	strcat(configPath,TRIGGER_ACTION_FILE_NAME);
	enum {UNINITIALIZED, INITIALIZED, CONNECTED, RUNNING} state = UNINITIALIZED;

	LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	JournalInit(MODULE_NAME);

	struct timeval tvTime;
	struct timeval nextSHMEMreadTime = { 0, 0 };

	DataDictionaryInitObjectData();

	// Initialize message bus connection
	while(iCommInit() && !quit) {
		nanosleep(&sleepTimePeriod,&remTime);
	}

	// Notify service handler that startup was successful
	sd_notify(0, "READY=1");

	while(!quit) {
		if (state == RUNNING) {
			// Make all active triggers cause their corresponding actions
			scenario.refresh();

			// Allow for retriggering on received TREO messages
			scenario.resetISOTriggers();
		}


		if ((recvDataLength = iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr)) < 0) {
			util_error("Message bus receive error");
		}

		switch (command) {
		case COMM_INIT:
			if (state == UNINITIALIZED) {
				try {
					LogMessage(LOG_LEVEL_INFO, "Initializing scenario");
					scenario.initialize(configPath);
					state = INITIALIZED;
				}
				catch (std::invalid_argument e) {
					std::string errMsg = "Invalid scenario file format: " + std::string(e.what());
					util_error(errMsg.c_str());
				}
				catch (std::ifstream::failure) {
					std::string errMsg = "Unable to open scenario file <" + std::string(configPath) + ">";
					util_error(errMsg.c_str());
				}
			}
			break;
		case COMM_OBJECTS_CONNECTED:
			if (state == INITIALIZED) {
				state = CONNECTED;
				LogMessage(LOG_LEVEL_INFO, "Distributing scenario configuration");
				scenario.sendConfiguration();
			}
			break;
		case COMM_OBC_STATE:
			// Ignore the state of object control
			break;
		case COMM_TREO:
			// Decode MQ data
			memcpy(&treo.triggerID, mqRecvData, sizeof(treo.triggerID));
			memcpy(&treo.timestamp_qmsow, mqRecvData+sizeof(treo.triggerID), sizeof(treo.timestamp_qmsow));
			memcpy(&treo.ip, mqRecvData+sizeof(treo.triggerID)+sizeof(treo.timestamp_qmsow), sizeof(treo.ip));

			if (state == RUNNING) {
				// Trigger corresponding trigger
				scenario.updateTrigger(treo.triggerID, treo);
			}
			break;
		case COMM_EXAC:
			LogMessage(LOG_LEVEL_ERROR, "Received unexpected execute action message");
			terminate = true;
			break;
		case COMM_TRCM:
			LogMessage(LOG_LEVEL_ERROR, "Received unexpected trigger configuration message");
			terminate = true;
			break;
		case COMM_ACCM:
			LogMessage(LOG_LEVEL_ERROR, "Received unexpected action configuration message");
			terminate = true;
			break;
		case COMM_EXIT:
			LogMessage(LOG_LEVEL_INFO, "Received exit command");
			terminate = true;
			break;
		case COMM_INV:
			nanosleep(&sleepTimePeriod,&remTime);
			break;
		case COMM_ABORT:
			LogMessage(LOG_LEVEL_INFO, "Received abort command");
			if (state == RUNNING) {
				state = CONNECTED;
			}
			break;
		case COMM_ARM:
			LogMessage(LOG_LEVEL_INFO, "Resetting scenario");
			scenario.reset();
			break;
		case COMM_STRT:
			if (state == CONNECTED) {
				LogMessage(LOG_LEVEL_INFO, "Received start message - transitioning to running state");
				state = RUNNING;
				// Update the triggers immediately on transition
				// to ensure they are always in a valid state when
				// they are checked for the first time
				updateObjectCheckTimer(&nextSHMEMreadTime, SHMEM_READ_RATE_HZ);
				updateTriggers(scenario);
			}
			else LogMessage(LOG_LEVEL_ERROR, "Received unexpected START command (current state: %u)",static_cast<unsigned char>(state));
			break;
		case COMM_DISCONNECT:
			LogMessage(LOG_LEVEL_INFO,"Received disconnect command");
			state = UNINITIALIZED;
			break;
		case COMM_GETSTATUS: {
			unsigned long startTime = UtilGetPIDUptime(getpid()).tv_sec;
			memset(mqSendData, 0, sizeof (mqSendData));
			sprintf(mqSendData, "%s:%lu", MODULE_NAME, startTime);

			if (iCommSend(COMM_GETSTATUS_OK, mqSendData, sizeof (mqSendData)) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending GETSTATUS.");
			}
		}
			break;
		case COMM_GETSTATUS_OK:
			break;
		default:
			LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
		}
		TimeSetToCurrentSystemTime(&tvTime);
		if (timercmp(&tvTime, &nextSHMEMreadTime, >)) {
			updateObjectCheckTimer(&nextSHMEMreadTime, SHMEM_READ_RATE_HZ);
			if (state == RUNNING) {
				updateTriggers(scenario);
			}
		}
	}
	LogMessage(LOG_LEVEL_INFO, MODULE_NAME " exiting");
	iCommClose();
	return 0;
}




/*!
 * \brief updateTriggers reads monr messages from the shared memory and passes the iformation along to scenario which handles trigger updates.
 *			with the rate parameter
 * \param Scenario scenario object keeping information about which trigger is linked to which action and the updating and parsing of the same.
 */
int updateTriggers(Scenario& scenario) {

	std::vector<uint32_t> transmitterIDs;
	uint32_t numberOfObjects;
	ObjectDataType monitorData;

	// Get number of objects present in shared memory
	if (DataDictionaryGetNumberOfObjects(&numberOfObjects) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR,
				   "Data dictionary number of objects read error - Cannot update triggers");
		return -1;
	}
	if (numberOfObjects == 0) {
		return 0;
	}

	transmitterIDs.resize(numberOfObjects, 0);
	// Get transmitter IDs for all connected objects
	if (DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size()) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR,
				   "Data dictionary transmitter ID read error - Cannot update triggers");
		return -1;
	}


	for (const uint32_t &transmitterID : transmitterIDs) {
		if (DataDictionaryGetMonitorData(transmitterID, &monitorData.MonrData) && DataDictionaryGetObjectIPByTransmitterID(transmitterID, &monitorData.ClientIP) != READ_OK) {
			LogMessage(LOG_LEVEL_ERROR,
					   "Data dictionary monitor data read error for transmitter ID %u",
					   transmitterID);
			return -1;
		}
		else {
			scenario.updateTrigger(monitorData);
		}

	}
	return 0;
}


/*!
 * \brief updateObjectCheckTimer Adds a time interval onto the specified time struct in accordance
 *			with the rate parameter
 * \param currentSHMEMReadTime Struct containing the timewhen at when SHMEM was last accessed. After this
 *			function has been executed, the struct contains the time at which the shared memory will be accessed is to be
 *			accessed next time.
 * \param SHMEMReadRate_Hz Rate at which SHMEM is read - if this parameter is 0 the value
 *			is clamped to 1 Hz
 */
void updateObjectCheckTimer(struct timeval *currentSHMEMReadTime, uint8_t SHMEMReadRate_Hz) {
	struct timeval SHMEMTimeInterval, timeDiff, currentTime;

	SHMEMReadRate_Hz = SHMEMReadRate_Hz == 0 ? 1 : SHMEMReadRate_Hz;	// Minimum frequency 1 Hz
	SHMEMTimeInterval.tv_sec = static_cast<long>(1.0 / SHMEMReadRate_Hz);
	SHMEMTimeInterval.tv_usec = static_cast<long>((1.0 / SHMEMReadRate_Hz - SHMEMTimeInterval.tv_sec) * 1000000.0);

	// If there is a large difference between the current time and the time at which time at which shared memory was updated, update based
	// on current time instead of last send time to not spam messages until caught up
	TimeSetToCurrentSystemTime(&currentTime);
	timersub(&currentTime, currentSHMEMReadTime, &timeDiff);
	if (timercmp(&timeDiff, &SHMEMTimeInterval, <)) {
		timeradd(currentSHMEMReadTime, &SHMEMTimeInterval, currentSHMEMReadTime);
	}
	else {
		timeradd(&currentTime, &SHMEMTimeInterval, currentSHMEMReadTime);
	}
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

