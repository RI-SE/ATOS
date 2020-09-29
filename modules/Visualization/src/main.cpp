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
int parseTraj(std::string line, std::vector<char> TCPbuffer);
/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/

int main(int argc, char const* argv[]){

	std::vector<char> TCPBuffer(MONR_BUFFER_LENGTH);
	std::vector<char> UDPBuffer(MONR_BUFFER_LENGTH);
	std::vector<uint32_t> transmitterids(MONR_BUFFER_LENGTH);
	std::vector<char> chekingTCPconn(MONR_BUFFER_LENGTH);
	std::vector<char> chekingUDPconn(MONR_BUFFER_LENGTH);
	std::vector<char> trajPath (PATH_MAX, '\0');
	

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

	//UtilGetTrajDirectoryPath (trajPath.data(), trajPath.size());
	//for (const auto & entry : std::experimental::filesystem::directory_iterator(trajPath.data())){
	//	/* TO DO:
	//		should have a checker 
	//		here to see that all the objects are connected
	//		before sending traj to visualizer*/
	//	
	//	std::ifstream file(entry.path());
	//	std::string line;
	//	int count;
	//	int retval = 0;
	//	while(std::getline(file,line)){
//
	//		retval = parseTraj(line,count, TCPBuffer)
	//		
	//		
	//	}
	//	//ssize_t encodeTRAJMessageHeader(const uint16_t trajectoryID, const uint16_t trajectoryVersion, const char * trajectoryName, const size_t nameLength, const uint32_t numberOfPointsInTraj, char * trajDataBuffer, const size_t bufferLength, const char debug);
	//	//ssize_t encodeTRAJMessagePoint(const struct timeval * pointTimeFromStart, const CartesianPosition position, const SpeedType speed, const AccelerationType acceleration, const float curvature, char * trajDataBufferPointer, const size_t remainingBufferLength, const char debug);
	//	//ssize_t encodeTRAJMessageFooter(char * trajDataBuffer, const size_t bufferLength, const char debug);
	//}	








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
		OnTCP = TCPServerVisualizer.getConnectionOn();
		
		while(OnTCP <= 0)
		{
			TCPServerVisualizer.TCPHandlerAccept(5);
			
			OnTCP = TCPServerVisualizer.getConnectionOn();
			if (OnTCP<0){
				break;
			}
			DataDictionaryGetNumberOfObjects(&nOBJ);

			for (int i = 0; i<nOBJ; i++ ){
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

		}
		
		
		
		std::cout<<"Connection recived"<<std::endl;
		
		/* So what do I need for the new system here? */
		
		/* fpro eller opro fpro skall jag tydligen skicka
			traj skall jag också skicka.

		*/

		//if (SendLater == COMM_CONNECT)
		//{
		//	UtilGetTrajDirectoryPath (trajPath.data(), trajPath.size());
		//	//UtilGetObjectDirectoryPath
		//	
		//	for (const auto & entry : std::experimental::filesystem::directory_iterator(trajPath.data())){
		//		/* TO DO:
		//			should have a checker 
		//			here to see that all the objects are connected
		//			before sending traj to visualizer*/
		//		
		//		std::ifstream file(entry.path());
		//		std::string line;
//
		//		while(std::getline(file,line,';')){
		//			std::cout<< line <<std::endl;
		//			
//
		//			// header check is TRAJECTORY, id, name, version, numberoflines
		//			// fotter for traj is ENDTRAJ
		//			
		//			// need the opro message to stich trajectorys to transmitter id:s
		//		} 		
		//	}
//
		//}
	

		while(bytesread >= 0 && OnTCP > 0){
			DataDictionaryGetNumberOfObjects(&nOBJ);
			DataDictionaryGetObjectTransmitterIDs(transmitterids.data(),transmitterids.size());
			
			for (int i = 0; i<nOBJ; i++ ){
				if (iCommRecv(&command,mqRecvData, MQ_MSG_SIZE, nullptr) < 0)
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
			}
			

			//if (command ==COMM_CONNECT)
			//{
			//	//TO DO add so it sends traj with tcp to ar vis 				
			//}
			
			bytesread = TCPServerVisualizer.receiveTCP(chekingTCPconn, 0);
			bytesent = UDPServerVisualizer.receiveUDP(chekingUDPconn);
			
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


//int parseTraj(std::string line,std::vector<char> buffer)
//{
//	struct timeval relTime;
//	CartesianPosition position;
//	SpeedType speed;
//	AccelerationType acceleration;
//	int count = 0;
//	TrajectoryFileHeader trajHeader;
//	char *dotToken;
//	int noOfLines = 0;
//	int retretvalval = 0;
//
//	std::stringstream ss (line);
//	std::string segement;
//	while (getline(ss,segement)){
//		
//		if(segement.compare("TRAJECTORY") == 0){
//			count = 1;
//		}
//		else if (segement.compare("LINE") == 0){
//			count = 5;
//		}
//		else{
//			count++
//		}
//		switch (count)
//		{
//		case 1/:
//			trajHeader->ID = static_cast<unsigned int> (atoi(segement));
//			break;
//		case 2/:
//			if (segement.length() > sizeof(header->name)){
//				LogMessage(LOG_LEVEL_ERROR, "Name field \"%s\" in trajectory too long", segement);	
//				retval = -1;
//			}
//			else{
//					strcpy(header->name, segement.c_str());
//			}
//			break;
//		case 3/:
//			header->majorVersion = (unsigned short)atoi(segement);
//			if ((dotToken = strchr(segement.c_str(), '.')) != NULL && *(dotToken + 1) != '\0') {
//				header->minorVersion = (unsigned short)atoi(dotToken + 1);
//			}
//			else {
//				header->minorVersion = 0;
//			}
//			break;
//		case 4/:
//			noOfLines = atoi(token);
//			if (noOfLines >= 0)
//				header->numberOfLines = (unsigned int)noOfLines;
//			else {
//				LogMessage(LOG_LEVEL_ERROR, "Found negative number of lines in trajectory");
//				retval = -1;
//			}
//			break;
//		
//			
//		
//		default:
//			break;
//		}
//
//	}
//
//
//}
//