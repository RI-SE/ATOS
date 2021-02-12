
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
#include <systemd/sd-daemon.h>

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
static int transmitOSEM(TCPHandler &tcpPort);

///ssize_t encodeOSEMMessage(const struct timeval* controlCenterTime, const uint32_t *desiredTransmitterID, const double_t * latitude_deg, const double_t * longitude_deg, const float * altitude_m, const float * maxPositionDeviation_m, const float * maxLateralDeviation_m, const float * minimumPositioningAccuracy_m, char * osemDataBuffer, const size_t bufferLength, const char debug);
/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/


int main(int argc, char const* argv[]) {
	COMMAND command = COMM_INV;
	std::vector<char> buffer(MONR_BUFFER_LENGTH);
	char mqRecvData[MBUS_MAX_DATALEN];
	const struct timespec sleepTimePeriod = {0,10000000};
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
		visualizerUDPPort.receiveUDP(buffer);
		buffer.clear();
		// have to read inbetween loops or we could end in limbo becouse getonconnection only looks att acceept not recive atm, 
		// TO DO: fix so that get connection also is looked at on recv andchange that to a bool but that means we have to change somestuff in tcphandler.
		visualizerTCPPort.receiveTCP(buffer,0);

		if (areObjectsConnected) {
			//LogMessage(LOG_LEVEL_INFO, "Sending Traj");
			//transmitTrajectories(visualizerTCPPort);
			LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
			transmitOSEM(visualizerTCPPort);
		}


		
		while(!quit && visualizerTCPPort.getConnectionOn() > 0 && bytesread>=0){
			do {
				if (iCommRecv(&command, mqRecvData, sizeof (mqRecvData), nullptr) < 0) {
					util_error("Message bus receive error");
				}
				switch (command) {
				case COMM_EXIT:
					quit = true;
					break;
				case COMM_OBJECTS_CONNECTED:
					//LogMessage(LOG_LEVEL_INFO, "Sending Traj");
					//transmitTrajectories(visualizerTCPPort);
					LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
					transmitOSEM(visualizerTCPPort);
					areObjectsConnected = true;
					break;
				}
			} while (command != COMM_INV);

			// TODO fix the timing we send etc
				

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
	if (numberOfObjects<=0){
		return 0;
	}
	transmitterIDs.resize(numberOfObjects);
	DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size());
	for (const auto &transmitterID : transmitterIDs) {
		if (transmitterID == 0){
			continue;
		}
		
		DataDictionaryGetMonitorData(transmitterID, &monitorData);
		DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);
		if (timerisset(&monitorDataReceiveTime)
				&& monitorData.position.isPositionValid
				&& monitorData.position.isHeadingValid) {
			// get transmitterid and encode monr to iso.
			// Checking so we still connected to visualizer
			if (tcpPort.receiveTCP(trashBuffer, 0) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Error when checking visualizer socket");
				return -1;
			}
			// only here to get server/client behavior
			

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
				return 0; //TO DO fix this, we can't break out of the while look becouse we failed with encoding monr, that will end up killing tcp and udp becouse a bad encode, should be handled in another way 
			}
			udpTransmitBuffer.resize(static_cast<unsigned long>(retval));
			udpPort.receiveUDP(trashBuffer);
			bytesSent = udpPort.sendUDP(udpTransmitBuffer);
			if (bytesSent < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Error when sending on visualizer UDP socket");
				return 0;//TO DO fix this, we can't break just becouse we couldent send a UDP message, even do it should not happen, but a example when it will happen is depending on which task started first visualizer or app etc.
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


int transmitOSEM(TCPHandler &tcp){
	timeval tv;
	GeoPosition originPosition;
	std::vector<char> tcpTransmitBuffer(MONR_BUFFER_LENGTH);
	std::vector<uint32_t> transmitterIDs;
	int bytesent;
	uint32_t numberOfObjects;	

	if(DataDictionaryGetNumberOfObjects(&numberOfObjects)!=READ_OK){
		LogMessage(LOG_LEVEL_ERROR, "Data dictionary number of objects read error ");
	}
	if (numberOfObjects<=0){
		return 0;
	}

	transmitterIDs.resize(numberOfObjects);
	if(DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size())!=READ_OK){
		LogMessage(LOG_LEVEL_ERROR,"Data dictionary get TransmitterID read error");
	}
	//Question: do we want to send for each
	for (const auto &transmitterID : transmitterIDs) {
		if (transmitterID == 0){
			continue;
		}
		
		if(DataDictionaryGetOrigin(transmitterID, &originPosition) != READ_OK){
			LogMessage(LOG_LEVEL_ERROR,
						"Data dictionary origion data read error for transmitter ID %u",
						transmitterID);
		}
		gettimeofday(&tv, NULL);
		float Altitude = (float) originPosition.Altitude;
		long retval = encodeOSEMMessage(&tv, 
										&transmitterID, 
										&originPosition.Latitude,
										&originPosition.Longitude,
										&Altitude, // don't like but this is how it is...
										NULL, 
										NULL, 
										NULL,
										tcpTransmitBuffer.data(), 
										tcpTransmitBuffer.size(),
										0);//TODO
		tcpTransmitBuffer.resize(static_cast<unsigned long>(retval));
		bytesent = tcp.sendTCP(tcpTransmitBuffer);

		if (bytesent < 0){
			LogMessage(LOG_LEVEL_ERROR,"Error when sending on the visualizer tcp socket");

		}
	}

	return 0;
}