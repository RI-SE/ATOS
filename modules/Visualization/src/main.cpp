
/**
 * @file main.cpp
 * @author Adam Eriksson (Adam.eriksson@astazero.com)
 * @brief  Sends UDP and TCP data for about objectmonitor and trajectory data to
 * a visualizer
 * @version 0.1
 * @date 2020-10-16
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <signal.h>
#include <systemd/sd-daemon.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "datadictionary.h"
#include "iso22133.h"
#include "logging.h"
#include "maestroTime.h"
#include "tcphandler.hpp"
#include "trajfilehandler.hpp"
#include "udphandler.hpp"
#include "util.h"

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
static int awaitConnection(TCPHandler& tcpPort, enum COMMAND& receivedCommand, bool& areObjectsConnected);
static int actOnMQCommand(TCPHandler& tcpPort, enum COMMAND& receivedCommand);
static int transmitTrajectories(TCPHandler& tcpPort);
static int transmitObjectData(TCPHandler& tcpPort, UDPHandler& udpPort, bool& areObjectsConnected);
static int transmitOSEM(TCPHandler& tcpPort);

/*------------------------------------------------------------
-- Main task
------------------------------------------------------------*/

int main(int argc, char const* argv[]) {
	COMMAND command = COMM_INV;
	std::vector<char> buffer(MONR_BUFFER_LENGTH);
	char mqRecvData[MBUS_MAX_DATALEN];
	const struct timespec sleepTimePeriod = {0, 10000000};
	struct timespec remTime;
	int bytesread = 0;
	char debug = 0;
	bool areObjectsConnected = false;

	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());
	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Initialize message bus connection
	while (iCommInit() && !quit) {
		nanosleep(&sleepTimePeriod, &remTime);
	}
	sd_notify(0, "READY=1");
	ReadWriteAccess_t retval = DataDictionaryInitObjectData();
	if (retval != READ_OK) {
		exit(EXIT_FAILURE);
	}

	TCPHandler visualizerTCPPort(TCP_VISUALIZATION_SERVER_PORT, "", "Server", 1, O_NONBLOCK);
	UDPHandler visualizerUDPPort(UDP_VISUALIZATION_SERVER_PORT, "", 0, "Server");
	while (!quit) {
		if (awaitConnection(visualizerTCPPort, command, areObjectsConnected) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Exit command sent, Visualization module shutting down");
			if (command == COMM_EXIT) {
				quit = true;
			}
		}

		if (areObjectsConnected) {
			LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
			transmitOSEM(visualizerTCPPort);
			areObjectsConnected = false;
		}

		transmitObjectData(visualizerTCPPort, visualizerUDPPort, areObjectsConnected);
		if (actOnMQCommand(visualizerTCPPort, command) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Exit command sent, Visualization module shutting down");
			if (command == COMM_EXIT) {
				quit = true;
			}
		}
		nanosleep(&sleepTimePeriod, &remTime);
	}

	visualizerTCPPort.TCPHandlerclose();
	visualizerUDPPort.UDPHandlerclose();
}

void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		quit = true;
		exit(EXIT_FAILURE);
	} else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}

int transmitObjectData(TCPHandler& tcpPort, UDPHandler& udpPort, bool& areObjectsConnected) {
	uint32_t numberOfObjects;
	std::vector<uint32_t> transmitterIDs;
	ObjectMonitorType monitorData;
	struct timeval monitorDataReceiveTime;
	uint8_t isoTransmitterID = 0;
	long bytesSent = 0;
	std::vector<char> trashBuffer(MONR_BUFFER_LENGTH);
	std::vector<char> udpTransmitBuffer(MONR_BUFFER_LENGTH);

	if (tcpPort.receiveTCP(trashBuffer, 0) < 0) {
		LogMessage(LOG_LEVEL_INFO, "Disconnected");
		areObjectsConnected = true;
		return -1;
	}

	DataDictionaryGetNumberOfObjects(&numberOfObjects);
	if (numberOfObjects == 0) {
		return 0;
	}

	transmitterIDs.resize(numberOfObjects);
	// get transmitterid and encode monr to iso.
	DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size());
	for (const auto& transmitterID : transmitterIDs) {
		if (transmitterID == 0) {
			continue;
		}

		DataDictionaryGetMonitorData(transmitterID, &monitorData);
		DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);
		if (timerisset(&monitorDataReceiveTime) && monitorData.position.isPositionValid
			&& monitorData.position.isHeadingValid) {
			if (tcpPort.receiveTCP(trashBuffer, 0) < 0) {
				LogMessage(LOG_LEVEL_INFO, "Disconnected");
				areObjectsConnected = true;
				return -1;
			}

			// TODO: PODI - now we cheat with header transmitter ID
			isoTransmitterID = static_cast<uint8_t>(transmitterID);
			setTransmitterID(isoTransmitterID);
			long retval = encodeMONRMessage(&monitorDataReceiveTime, monitorData.position, monitorData.speed,
											monitorData.acceleration, monitorData.drivingDirection,
											monitorData.state, monitorData.armReadiness,
											0,	// TODO:
											udpTransmitBuffer.data(), udpTransmitBuffer.size(), 0);
			if (retval < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Failed when encoding MONR message");
				return 0;
			}

			udpTransmitBuffer.resize(static_cast<unsigned long>(retval));
			udpPort.receiveUDP(trashBuffer);
			bytesSent = udpPort.sendUDP(udpTransmitBuffer);
			if (bytesSent < 0) {
				LogMessage(LOG_LEVEL_INFO, "Unable to send ObjectData to Visualizer");
			}
		}
	}
	return 0;
}

int awaitConnection(TCPHandler& tcpPort, enum COMMAND& receivedCommand, bool& areObjectsConnected) {
	receivedCommand = COMM_INV;
	char mqRecvData[MBUS_MAX_DATALEN];
	if (tcpPort.getConnectionOn() != 1) {
		LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
	}

	while (tcpPort.getConnectionOn() != 1) {
		tcpPort.TCPHandlerAccept(5);

		if (tcpPort.getConnectionOn() == 1) {
			LogMessage(LOG_LEVEL_INFO, "TCP connection established");
		}

		if (iCommRecv(&receivedCommand, mqRecvData, sizeof(mqRecvData), nullptr) < 0) {
			util_error("Message bus receive error");
		}

		switch (receivedCommand) {
		case COMM_EXIT:
			return -1;
		case COMM_OBJECTS_CONNECTED:
			areObjectsConnected = true;
			break;
		case COMM_DISCONNECT:  // TODO what happens when an object forces
							   // the disconnection?
			areObjectsConnected = false;
			break;
		case COMM_CONNECT:
			areObjectsConnected = true;
			break;
		default:
			continue;
		}
	}
	return 0;
}

int transmitTrajectories(TCPHandler& tcpPort) {
	std::vector<char> trajPath(PATH_MAX, '\0');
	std::vector<char> transmitBuffer;
	int retval = 0, rc;

	UtilGetTrajDirectoryPath(trajPath.data(), trajPath.size());

	for (const auto& entry : fs::directory_iterator(trajPath.data())) {
		/* TO DO:
		should have a checker
		here to see that all the objects are connected
		before sending traj to visualizer*/

		std::ifstream file(entry.path());
		std::string line;
		int retval = 0;
		while (std::getline(file, line)) {
			rc = parseTraj(line, transmitBuffer);

			if (rc == -1) {
				retval = -1;
				break;
			} else if ((MONR_BUFFER_LENGTH - retval) < transmitBuffer.size()) {
				tcpPort.sendTCP(transmitBuffer);
				transmitBuffer.clear();
			}
		}
	}
	return retval;
}

int transmitOSEM(TCPHandler& tcp) {
	ObjectSettingsType osem;
	GeoPosition originPosition;
	std::vector<char> tcpTransmitBuffer(MONR_BUFFER_LENGTH);
	std::vector<uint32_t> transmitterIDs;
	int bytesSent;
	uint32_t numberOfObjects;

	if (DataDictionaryGetNumberOfObjects(&numberOfObjects) != READ_OK) {
		throw std::ios_base::failure("Data dictionary number of objects read error");
	}
	if (numberOfObjects <= 0) {
		return 0;
	}

	transmitterIDs.resize(numberOfObjects);
	if (DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size()) != READ_OK) {
		throw std::ios_base::failure("Data dictionary transmitter ID read error");
	}

	for (const auto& transmitterID : transmitterIDs) {
		if (transmitterID == 0) {
			continue;
		}

		if (DataDictionaryGetOrigin(transmitterID, &originPosition) != READ_OK) {
			throw std::ios_base::failure("Data dictionary origin read error for transmitter ID "
										 + std::to_string(transmitterID));
		}

		osem.desiredTransmitterID = transmitterID;
		osem.coordinateSystemOrigin.latitude_deg = originPosition.Latitude;
		osem.coordinateSystemOrigin.longitude_deg = originPosition.Longitude;
		osem.coordinateSystemOrigin.altitude_m = originPosition.Altitude;
		osem.coordinateSystemOrigin.isLatitudeValid = true;
		osem.coordinateSystemOrigin.isLongitudeValid = true;
		osem.coordinateSystemOrigin.isAltitudeValid = true;

		TimeSetToCurrentSystemTime(&osem.currentTime);
		long retval = encodeOSEMMessage(&osem, tcpTransmitBuffer.data(), tcpTransmitBuffer.size(), false);
		if (retval < 0) {
			throw std::invalid_argument("Failed to encode OSEM message");
		}

		tcpTransmitBuffer.resize(static_cast<unsigned long>(retval));
		bytesSent = tcp.sendTCP(tcpTransmitBuffer);

		if (bytesSent < 0) {
			throw std::ios_base::failure("Error when sending on the visualizer tcp socket");
		}
	}

	return 0;
}

int actOnMQCommand(TCPHandler& tcpPort, COMMAND& receivedCommand) {
	receivedCommand = COMM_INV;
	char mqRecvData[MBUS_MAX_DATALEN];

	if (iCommRecv(&receivedCommand, mqRecvData, sizeof(mqRecvData), nullptr) < 0) {
		util_error("Message bus receive error");
	}
	switch (receivedCommand) {
	case COMM_EXIT:
		return -1;
	case COMM_OBJECTS_CONNECTED:
		LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
		transmitOSEM(tcpPort);
		break;
	case COMM_CONNECT:
		LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
		transmitOSEM(tcpPort);
		break;
	}
}
