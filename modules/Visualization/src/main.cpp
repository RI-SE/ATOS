
/**
 * @file main.cpp
 * @author Adam Eriksson (Adam.eriksson@astazero.com)
 * @brief  Sends UDP and TCP data for about objectmonitor and trajectory data to a visualizer
 * @version 0.1
 * @date 2020-10-16
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <signal.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "iso22133.h"

#include "trajfilehandler.hpp"
#include "udphandler.hpp"
#include "tcphandler.hpp"

#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MODULE_NAME "Visualization"
#define RECV_MESSAGE_BUFFER_LENGTH 1024
#define MONR_BUFFER_LENGTH 1024
#define TCP_VISUALIZATION_SERVER_PORT 53250
#define UDP_VISUALIZATION_SERVER_PORT 53251
#define TRAJECTORY_TX_BUFFER_SIZE 2048

/*------------------------------------------------------------
  -- Static variables
  ------------------------------------------------------------*/
static bool quit = false;

/*------------------------------------------------------------
  -- Function declarations
  ------------------------------------------------------------*/


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static void signalHandler(int signo);
static int awaitConnection(TCPHandler &tcpPort, enum COMMAND &receivedCommand, bool& areObjectsConnected);
static int transmitTrajectories(TCPHandler &tcpPort);
static int transmitObjectData(TCPHandler &tcpPort, UDPHandler &udpPort);

/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/


int main(int argc, char const* argv[]) {
	COMMAND command = COMM_INV;

	char mqRecvData[MBUS_MAX_DATALEN];
	const struct timespec sleepTimePeriod = {0,10000000};

	struct timespec remTime;

	char debug = 0;
	bool areObjectsConnected = false;
	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Initialize message bus connection
	while(iCommInit() && !quit) {
		nanosleep(&sleepTimePeriod,&remTime);
	}

	// Notify service handler that startup was successful
	sd_notify(0, "READY=1");

	ReadWriteAccess_t retval = DataDictionaryInitObjectData();

	if (retval != READ_OK) {
		exit(EXIT_FAILURE);
	}

	while(!quit){
		TCPHandler visualizerTCPPort(TCP_VISUALIZATION_SERVER_PORT , "", "Server", 1, O_NONBLOCK);
		UDPHandler visualizerUDPPort(UDP_VISUALIZATION_SERVER_PORT, "", 0, "Server");

		if (awaitConnection(visualizerTCPPort, command, areObjectsConnected) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Failed to establish connection");
			if (command == COMM_EXIT) {
				quit = true;
			}
			continue;
		}

		if (areObjectsConnected) {
			transmitTrajectories(visualizerTCPPort);
		}

		while(!quit && visualizerTCPPort.getConnectionOn() > 0){
			do {
				if (iCommRecv(&command, mqRecvData, sizeof (mqRecvData), nullptr) < 0) {
					util_error("Message bus receive error");
				}
				switch (command) {
				case COMM_EXIT:
					quit = true;
					break;
				case COMM_OBJECTS_CONNECTED:
					transmitTrajectories(visualizerTCPPort);
					areObjectsConnected = true;
					break;
				}
			} while (command != COMM_INV);

			if (transmitObjectData(visualizerTCPPort, visualizerUDPPort) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Failed to transmit object data");
				break;
			}
			nanosleep(&sleepTimePeriod,&remTime); // sleep might not be needed! but added it becouse i am not sure how much it will use sharedmemory resources
		}
		visualizerTCPPort.TCPHandlerclose();
		visualizerUDPPort.UDPHandlerclose();
		LogMessage(LOG_LEVEL_INFO, "Disconnected");
	}
}


void signalHandler(int signo){
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		quit = true;
		exit(EXIT_FAILURE);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}


int transmitObjectData(TCPHandler &tcpPort, UDPHandler &udpPort) {
	uint32_t numberOfObjects;
	std::vector<uint32_t> transmitterIDs;
	ObjectMonitorType monitorData;
	struct timeval monitorDataReceiveTime;
	uint8_t isoTransmitterID = 0;
	long bytesSent = 0;
	std::vector<char> trashBuffer(MONR_BUFFER_LENGTH);
	std::vector<char> udpTransmitBuffer(MONR_BUFFER_LENGTH);

	DataDictionaryGetNumberOfObjects(&numberOfObjects);
	transmitterIDs.resize(numberOfObjects);
	DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size());

	for (const auto &transmitterID : transmitterIDs) {
		if (transmitterID == 0){
			continue;
		}

		DataDictionaryGetMonitorData(transmitterID, &monitorData);
		DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);// i am getting duplicates from this one
		if (timerisset(&monitorDataReceiveTime)
				&& monitorData.position.isPositionValid
				&& monitorData.position.isHeadingValid) {
			// get transmitterid and encode monr to iso.
			// Checking so we still connected to visualizer
			if (tcpPort.receiveTCP(trashBuffer, 0) < 0
					|| udpPort.receiveUDP(trashBuffer) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Error when checking visualizer socket");
				return -1;
			}

			// TODO PODI - now we cheat with header transmitter ID
			isoTransmitterID = static_cast<uint8_t>(transmitterID);
			setTransmitterID(isoTransmitterID);
			long retval = encodeMONRMessage(&monitorDataReceiveTime,
											monitorData.position,
											monitorData.speed,
											monitorData.acceleration,
											monitorData.drivingDirection,
											monitorData.state,
											monitorData.armReadiness,
											0, // TODO
											udpTransmitBuffer.data(),
											udpTransmitBuffer.size(), 0);
			if (retval < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Failed when encoding MONR message");
				return -1;
			}
			udpTransmitBuffer.resize(static_cast<unsigned long>(retval));

			bytesSent = udpPort.sendUDP(udpTransmitBuffer);
			if (bytesSent < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Error when sending on visualizer UDP socket");
				return -1;
			}
		}
	}
	return 0;
}

int awaitConnection(
		TCPHandler &tcpPort,
		enum COMMAND &receivedCommand,
		bool &areObjectsConnected) {

	uint32_t numberOfObjects = 0;
	receivedCommand = COMM_INV;
	char mqRecvData[MBUS_MAX_DATALEN];

	LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
	while(tcpPort.getConnectionOn() != 1) {
		tcpPort.TCPHandlerAccept(5);

		do {
			if (iCommRecv(&receivedCommand, mqRecvData, sizeof (mqRecvData), nullptr) < 0) {
				util_error("Message bus receive error");
			}

			switch (receivedCommand) {
			case COMM_EXIT:
				return -1;
			case COMM_OBJECTS_CONNECTED:
				areObjectsConnected = true;
				break;
			case COMM_DISCONNECT: // TODO what happens when an object forces the disconnection?
				areObjectsConnected = false;
				break;
			}
		} while (receivedCommand != COMM_INV);
	}

	LogMessage(LOG_LEVEL_INFO, "TCP connection established");
	return 0;
}

int transmitTrajectories(TCPHandler &tcpPort) {

	std::vector<char> trajPath(PATH_MAX, '\0');
	std::vector<char> transmitBuffer;
	int retval = 0, rc;

	UtilGetTrajDirectoryPath(trajPath.data(), trajPath.size());

	for (const auto &entry : fs::directory_iterator(trajPath.data())) {
		/* TO DO:
		should have a checker
		here to see that all the objects are connected
		before sending traj to visualizer*/

		std::ifstream file(entry.path());
		std::string line;
		int retval = 0;
		while (std::getline(file,line)) {
			rc = parseTraj(line, transmitBuffer);

			if (rc == -1) {
				retval = -1;
				break;
			}
			else if ((MONR_BUFFER_LENGTH-retval) < transmitBuffer.size()) {
				tcpPort.sendTCP(transmitBuffer);
				transmitBuffer.clear();
			}
		}
	}
	return retval;
}
