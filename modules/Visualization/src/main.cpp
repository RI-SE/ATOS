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
#include "tcphandler.h"
#include "iso22133.h"
#include "udphandler.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MODULE_NAME "Visualization"
#define RECV_MESSAGE_BUFFER 1024
#define MONR_BUFFER_LENGTH 1024
#define TCP_VISUAL_SERVER_PORT 53250
#define UDP_VISUAL_SERVER_PORT 53251 
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
/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/

int main(int argc, char const* argv[]){

	//std::vector<char> TCPBuffer(MONR_BUFFER_LENGTH);
	std::vector<char> UDPBuffer(MONR_BUFFER_LENGTH);
	std::vector<uint32_t> transmitterids(MONR_BUFFER_LENGTH);
	std::vector<char> chekingTCPconn(MONR_BUFFER_LENGTH);
	std::vector<char> chekingUDPconn(MONR_BUFFER_LENGTH);
	ObjectMonitorType monitorData;
	std::string IPaddr;
	
	COMMAND command = COMM_INV;
	COMMAND SendLater= COMM_INV;

    char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0,10000000};
	
	struct timespec remTime;
	struct timeval monitorDataReceiveTime;
	uint32_t transmitterId;
	uint8_t isoTransmitterID;

	int bytesent = 1;
	char debug = 0;
	int bytesread = 1;
	int objectErrorState = 0;
	uint32_t nOBJ;
	int recvDataLength = 0;
	unsigned long flagsock = O_NONBLOCK;
	int OnTCP = 0;	
	LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);

	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

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
	
	if (retval != READ_OK)
	{
		exit(EXIT_FAILURE);
	}
	
	std::cout << "Awaiting TCP connection..." << std::endl;
	while(!quit){
		
		TCPHandler TCPServerVisualizer(TCP_VISUAL_SERVER_PORT, "", "Server", 1, O_NONBLOCK);
		UDPHandler UDPServerVisualizer (UDP_VISUAL_SERVER_PORT,"",0,"Server");
		OnTCP = TCPServerVisualizer.ConnectionON;
		
		while(OnTCP <= 0)
		{
			TCPServerVisualizer.TCPHandlerAccept(5);
			
			OnTCP = TCPServerVisualizer.ConnectionON;
			if (OnTCP<0){
				break;
			}
			
			if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0)
			{
				util_error("Message bus receive error");
			}
			switch (command)
			{
			case COMM_CONNECT:
				SendLater = COMM_CONNECT; 
				break;
			
			default:
				break;
			}

		}
		
		
		
		std::cout<<"Connection recived"<<std::endl;
		/*
		if (SendLater == COMM_CONNECT)
		{
			//TO DO: check if transmitterid's are bound to ip in datadictonary
			
			std::cout <<"Sending trajectory to visualizer"<<std::endl;
			TrajectoryFileHeader fileHeader;
			TrajectoryFileLine fileLine;
			char trajPathDir[MAX_FILE_PATH];
		
			UtilGetTrajDirectoryPath(trajPathDir,sizeof(trajPathDir));
			std::string str(trajPathDir);
			
			for (auto& entry : std::experimental::filesystem::directory_iterator(trajPathDir)){
				memset(&fileHeader, 0, sizeof (fileHeader));
				memset(&fileLine, 0, sizeof (fileLine));
				char messageBuffer[TRAJECTORY_TX_BUFFER_SIZE];
				size_t remainingBufferSpace = sizeof (messageBuffer);
				char *messageBufferPosition = messageBuffer;
				
				if (UtilCheckTrajectoryFileFormat(entry.path().string().c_str(), strlen(entry.path().string().c_str())) == -1) {
					LogMessage(LOG_LEVEL_ERROR, "Incorrect trajectory file format - cannot proceed to send message");
					break;
				}

				std::ifstream infile(entry.path().string());
				std::string line;
				std::getline(infile, line);
				std::vector<char> cstr(line.c_str(), line.c_str() + line.size() + 1);
				if(UtilParseTrajectoryFileHeader(cstr.data(), &fileHeader) == -1){
					LogMessage(LOG_LEVEL_ERROR, "Failed to parse header of file <%s>", entry.path().string());
					break;
				}
				
				if ((encodeTRAJMessageHeader(fileHeader.ID > UINT16_MAX ? 0 : (uint16_t) fileHeader.ID,
					fileHeader.majorVersion, fileHeader.name,
					strlen(fileHeader.name), fileHeader.numberOfLines,
					messageBufferPosition, remainingBufferSpace, debug)) == -1) {
					LogMessage(LOG_LEVEL_ERROR, "Unable to encode trajectory message");
					break;
				}

				while (std::getline(infile, line)){
					std::vector<char> cstr(line.c_str(), line.c_str() + line.size() + 1);
					struct timeval relTime;
					CartesianPosition position;
					SpeedType speed;
					AccelerationType acceleration;
					if (UtilParseTrajectoryFileLine(cstr.data(), &fileLine) == -1) {
						// TODO: how to terminate an ISO message when an error has occurred?
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse line %u of trajectory file <%s>", i + 1, Filename);
						break;
					}
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

					// Print to buffer
					if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
															(float)fileLine.curvature, messageBufferPosition,
															remainingBufferSpace, debug)) == -1) {

						if (errno == ENOBUFS) {
							// Reached the end of buffer, send buffered data and
							// try again
							UtilSendTCPData(MODULE_NAME, messageBuffer, messageBufferPosition - messageBuffer, Socket,
											debug);

							messageBufferPosition = messageBuffer;
							remainingBufferSpace = sizeof (messageBuffer);
							if ((printedBytes =
								encodeTRAJMessagePoint(&relTime, position, speed, acceleration, fileLine.curvature,
														messageBufferPosition, remainingBufferSpace, debug)) == -1) {
								// TODO how to terminate an ISO message when an error has occurred?
								LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message point");
								fclose(fd);
								return -1;
							}
							messageBufferPosition += printedBytes;
							totalPrintedBytes += printedBytes;
							remainingBufferSpace -= (size_t) printedBytes;
						}
						else {
							// TODO how to terminate an ISO message when an error has occurred?
							LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message point");
							fclose(fd);
							return -1;
						}
					}
					else {
						totalPrintedBytes += printedBytes;
						messageBufferPosition += printedBytes;
						remainingBufferSpace -= (size_t) printedBytes;
					}


				}
				
			}

		}
		*/

		while(bytesread >= 0 && OnTCP > 0){

			
			if (iCommRecv(&command,mqRecvData, MQ_MSG_SIZE,nullptr) < 0)
			{
            	util_error("Message bus receive error");
        	}
			switch (command)
			{
			case COMM_CONNECT:
				//TODO send traj
				break;
			
			default:
				break;
			}

			if (command ==COMM_CONNECT)
			{
				//TO DO add so it sends traj with tcp to ar vis 				
			}
			
			bytesread = TCPServerVisualizer.receiveTCP(chekingTCPconn, 0);
			bytesent = UDPServerVisualizer.receiveUDP(chekingUDPconn);
			
			DataDictionaryGetNumberOfObjects(&nOBJ);
			DataDictionaryGetObjectTransmitterIDs(transmitterids.data(),transmitterids.size());
			transmitterids.resize(nOBJ);
			std::cout <<nOBJ<<std::endl;
			for (auto &transmitterID : transmitterids) {
			
				if (transmitterID <= 0){
					break;
				}
			
				DataDictionaryGetMonitorData(transmitterID, &monitorData);
				DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);// i am getting duplicates from this one
				//std::cout<<monitorDataReceiveTime.tv_sec<< "\nposvalid "<<monitorData.position.isPositionValid <<"\nheadingvalid" <<monitorData.position.isHeadingValid <<std::endl;
				if (monitorDataReceiveTime.tv_sec > 0&& monitorData.position.isPositionValid && monitorData.position.isHeadingValid) {
					// get transmitterid and encode monr to iso.
					isoTransmitterID = (uint8_t) transmitterID;
					setTransmitterID(isoTransmitterID);
					bytesent = UDPServerVisualizer.receiveUDP(chekingUDPconn);
					long retval = encodeMONRMessage(&monitorData.timestamp,monitorData.position,monitorData.speed,monitorData.acceleration,monitorData.drivingDirection,monitorData.state, monitorData.armReadiness, objectErrorState,UDPBuffer.data(), UDPBuffer.size(),debug);
					//std::cout << monitorData.position.yCoord_m<<std::endl;
					UDPBuffer.resize(static_cast<unsigned long>(retval));
					// Cheking so we still connected to visualizer

					bytesread = TCPServerVisualizer.receiveTCP(chekingTCPconn, 0);
					if (bytesread < 0) {
						std::cout<<"Error when reading from Maestro TCP socket"<<std::endl;
						break;

           			}
					//n sending message with udp.
					
					bytesent = UDPServerVisualizer.sendUDP(UDPBuffer);	
				}
				
			}
			nanosleep(&sleepTimePeriod,&remTime); // sleep might not be needed! but added it becouse i am not sure how much it will use sharedmemory resources
			
			
		}
		std::cout <<"Disconnected"<<std::endl;
        std::cout << "Awaiting TCP connection..." << std::endl;
		OnTCP = 0;
		bytesread = 1;
        TCPServerVisualizer.TCPHandlerclose();
		UDPServerVisualizer.UDPHandlerclose();
	}

	//ServerVisualizer.TCPHandlerclose();
}


void signalHandler(int signo) {
    if (signo == SIGINT) {
        LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
        quit = true;
		exit(EXIT_FAILURE);
    }
    else {
        LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
    }
}

