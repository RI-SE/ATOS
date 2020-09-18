#include <systemd/sd-daemon.h>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <string>

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
		if (SendLater == COMM_CONNECT)
		{
			std::cout <<"Sending trajectory to visualizer"<<std::endl;
			// TO DO: swend traj with tcp
		}
		
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
			
			DataDictionaryGetNumberOfObjects(&nOBJ);
			DataDictionaryGetObjectTransmitterIDs(transmitterids.data(),transmitterids.size());
			transmitterids.resize(nOBJ);
			
			for (auto &transmitterID : transmitterids) {
			
				if (transmitterID <= 0){
					break;
				}
			
				DataDictionaryGetMonitorData(transmitterID, &monitorData);
				DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);// i am getting duplicates from this one
			
				if (monitorDataReceiveTime.tv_sec > 0&& monitorData.position.isPositionValid && monitorData.position.isHeadingValid) {
					// get transmitterid and encode monr to iso.
					isoTransmitterID = (uint8_t) transmitterID;
					setTransmitterID(isoTransmitterID);
					long retval = encodeMONRMessage(&monitorData.timestamp,monitorData.position,monitorData.speed,monitorData.acceleration,monitorData.drivingDirection,monitorData.state, monitorData.armReadiness, objectErrorState,UDPBuffer.data(), UDPBuffer.size(),debug);
					std::cout << monitorData.position.yCoord_m<<std::endl;
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

