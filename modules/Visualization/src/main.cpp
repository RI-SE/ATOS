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


/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MODULE_NAME "Visualization"
#define RECV_MESSAGE_BUFFER 1024
#define MONR_BUFFER_LENGTH  1024
#define VISUAL_SERVER_PORT  53250

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

	std::vector<char> TCPBuffer(MONR_BUFFER_LENGTH);
	std::vector<uint32_t> transmitterids(MONR_BUFFER_LENGTH);
	std::vector<char> chekingTCPconn(MONR_BUFFER_LENGTH);
	ObjectMonitorType monitorData;
	std::string IPaddr;
	COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0,10000000};
	const struct timespec sleepTimeTCP = {0,999999999};
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
		
		TCPHandler ServerVisualizer(VISUAL_SERVER_PORT, "", "Server", 1, O_NONBLOCK);
		OnTCP = ServerVisualizer.ConnectionON;
		
		while(OnTCP <= 0)
		{
			ServerVisualizer.TCPHandlerAccept();
			OnTCP = ServerVisualizer.ConnectionON;
			if (OnTCP<0){
				break;
			}
			
			if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0)
			{
				util_error("Message bus receive error");
			}
			

		}
		
		
		std::cout<<"Connection recived"<<std::endl;
		while(bytesread >= 0 && OnTCP > 0){

			if (iCommRecv(&command,mqRecvData, MQ_MSG_SIZE,nullptr) < 0)
			{
            	util_error("Message bus receive error");
        	}
			bytesread = ServerVisualizer.receiveTCP(chekingTCPconn, 0);
			//iCommRecv(nullptr,nullptr,0,nullptr);
			DataDictionaryGetNumberOfObjects(&nOBJ);
			DataDictionaryGetObjectTransmitterIDs(transmitterids.data(),transmitterids.size());
			transmitterids.resize(nOBJ);
			for (auto &transmitterID : transmitterids) {
				std::cout<<"transmitterid"<<transmitterID<<std::endl;
				if (transmitterID <= 0){
					break;
				}
				std::cout <<"Transmitter id "<< transmitterID <<std::endl;
				DataDictionaryGetMonitorData(transmitterID, &monitorData);
				DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);
				if (monitorDataReceiveTime.tv_sec > 0) {
					isoTransmitterID = (uint8_t) transmitterID;
					setTransmitterID(isoTransmitterID);
					long retval = encodeMONRMessage(&monitorData.timestamp,monitorData.position,monitorData.speed,monitorData.acceleration,monitorData.drivingDirection,monitorData.state, monitorData.armReadiness, objectErrorState,TCPBuffer.data(),TCPBuffer.size(),debug);
					TCPBuffer.resize(static_cast<unsigned long>(retval));
					bytesread = ServerVisualizer.receiveTCP(chekingTCPconn, 0);
					if (bytesread < 0) {
						std::cout<<"Error when reading from Maestro TCP socket"<<std::endl;
						break;

           			}
					bytesent = ServerVisualizer.sendTCP(TCPBuffer);	
				}
				
			}
			nanosleep(&sleepTimePeriod,&remTime);
			
		}
		std::cout <<"Disconnected"<<std::endl;
        std::cout << "Awaiting TCP connection..." << std::endl;
		OnTCP = 0;
		bytesread = 1;
        ServerVisualizer.TCPHandlerclose();
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

