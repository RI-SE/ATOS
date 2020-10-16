#include <systemd/sd-daemon.h>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <experimental/filesystem>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "iso22133.h"

#include "udphandler.hpp"
#include "tcphandler.hpp"

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
static int parseTraj(std::string line, std::vector<char>& TCPbuffer);
static int awaitConnection(TCPHandler &tcpPort, enum COMMAND &receivedCommand, bool& areObjectsConnected);
static int transmitTrajectories(TCPHandler &tcpPort);
static int transmitObjectData(TCPHandler &tcpPort, UDPHandler &udpPort);
/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/

int main(int argc, char const* argv[]){

    std::vector<char> TCPBuffer;
	std::vector<char> UDPBuffer(MONR_BUFFER_LENGTH);
    std::vector<char> checkTCPconnectionBuffer(MONR_BUFFER_LENGTH);
	std::vector<char> checkUDPconnectionBuffer(MONR_BUFFER_LENGTH);


    std::string IPaddr;

	COMMAND command = COMM_INV;

	char mqRecvData[MBUS_MAX_DATALEN];
    const struct timespec sleepTimePeriod = {0,10000000};

	struct timespec remTime;

	char debug = 0;
	bool areObjectsConnected = false;
    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);

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


        /* So what do I need for the new system here? */

        /* fpro or opro fpro should be sent here
            traj should also be sent.

        */

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

	for (const auto &entry : std::experimental::filesystem::directory_iterator(trajPath.data())) {
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

int parseTraj(std::string line,std::vector<char>& buffer)
{
    struct timeval relTime;
    CartesianPosition position;
    SpeedType speed;
    AccelerationType acceleration;
    TrajectoryFileHeader fileHeader;
    TrajectoryFileLine fileLine;

    double curvature = 0;
    std::stringstream ss (line);
    std::string segment;
    getline(ss,segment,';');
    ssize_t printedBytes;
    int debug = 0;

    memset(&fileHeader, 0, sizeof (fileHeader));
    memset(&fileLine, 0, sizeof (fileLine));

    std::vector<char> tmpvector(MONR_BUFFER_LENGTH);
    std::vector<char> cstr(line.c_str(), line.c_str() + line.size() + 1);

    if(segment.compare("TRAJECTORY") == 0){

        UtilParseTrajectoryFileHeader(cstr.data(),&fileHeader);
        if ((printedBytes = encodeTRAJMessageHeader(fileHeader.ID > UINT16_MAX ? 0 : (uint16_t) fileHeader.ID,
                                                fileHeader.majorVersion, fileHeader.name,
                                                strlen(fileHeader.name), fileHeader.numberOfLines,
                                                tmpvector.data(), tmpvector.size(), debug)) == -1) {
        LogMessage(LOG_LEVEL_ERROR, "Unable to encode trajectory message");

        return -1;
        }


    }
    else if (segment.compare("LINE") == 0){


        UtilParseTrajectoryFileLine(cstr.data(), &fileLine);

        relTime.tv_sec = (time_t) fileLine.time;
        relTime.tv_usec = (time_t) ((fileLine.time - relTime.tv_sec) * 1000000);
        position.xCoord_m = fileLine.xCoord;
        position.yCoord_m = fileLine.yCoord;
        position.isPositionValid = fileLine.zCoord != NULL;
        position.zCoord_m = position.isPositionValid ? *fileLine.zCoord : 0;
        position.heading_rad = fileLine.heading;
        position.isHeadingValid = true;
        speed.isLongitudinalValid = fileLine.longitudinalVelocity != NULL;
        speed.isLateralValid = fileLine.lateralVelocity != NULL;
        speed.longitudinal_m_s = fileLine.longitudinalVelocity != NULL ? *fileLine.longitudinalVelocity : 0;
        speed.lateral_m_s = fileLine.lateralVelocity != NULL ? *fileLine.lateralVelocity : 0;
        acceleration.isLongitudinalValid = fileLine.longitudinalAcceleration != NULL;
        acceleration.isLateralValid = fileLine.lateralAcceleration != NULL;
        acceleration.longitudinal_m_s2 =
            fileLine.longitudinalAcceleration != NULL ? *fileLine.longitudinalAcceleration : 0;

        acceleration.lateral_m_s2 = fileLine.lateralAcceleration != NULL ? *fileLine.lateralAcceleration : 0;
        if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
                                                   (float)fileLine.curvature, tmpvector.data(),
                                                   tmpvector.size(), debug)) == -1) {
            return -1;
        }
    }
    else if(segment.compare("ENDTRAJECTORY")== 0){

        if((printedBytes = encodeTRAJMessageFooter(tmpvector.data(), tmpvector.size(), debug))==-1){
            return -1;

        }

    }
    else{

        UtilParseTrajectoryFileLine(cstr.data(),&fileLine);


        relTime.tv_sec = (time_t) fileLine.time;
        relTime.tv_usec = (time_t) ((fileLine.time - relTime.tv_sec) * 1000000);
        position.xCoord_m = fileLine.xCoord;
        position.yCoord_m = fileLine.yCoord;
        position.isPositionValid = fileLine.zCoord != NULL;
        position.zCoord_m = position.isPositionValid ? *fileLine.zCoord : 0;
        position.heading_rad = fileLine.heading;
        position.isHeadingValid = true;
        speed.isLongitudinalValid = fileLine.longitudinalVelocity != NULL;
        speed.isLateralValid = fileLine.lateralVelocity != NULL;
        speed.longitudinal_m_s = fileLine.longitudinalVelocity != NULL ? *fileLine.longitudinalVelocity : 0;
        speed.lateral_m_s = fileLine.lateralVelocity != NULL ? *fileLine.lateralVelocity : 0;
        acceleration.isLongitudinalValid = fileLine.longitudinalAcceleration != NULL;
        acceleration.isLateralValid = fileLine.lateralAcceleration != NULL;
        acceleration.longitudinal_m_s2 =
            fileLine.longitudinalAcceleration != NULL ? *fileLine.longitudinalAcceleration : 0;

        acceleration.lateral_m_s2 = fileLine.lateralAcceleration != NULL ? *fileLine.lateralAcceleration : 0;
        if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
                                                   (float)fileLine.curvature, tmpvector.data(),
                                                   tmpvector.size(), debug)) == -1) {
            return -1;
        }
    }
    tmpvector.resize(printedBytes);
    for (auto val : tmpvector) buffer.push_back(val);


    return printedBytes;


}
