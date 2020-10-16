
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
#include <experimental/filesystem>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "iso22133.h"

#include "FileHandler.hpp"
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

/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/

int main(int argc, char const* argv[]){

    std::vector<char> TCPBuffer;
    std::vector<char> UDPBuffer(MONR_BUFFER_LENGTH);
    std::vector<uint32_t> transmitterIDs(MONR_BUFFER_LENGTH);
    std::vector<char> trajPath (PATH_MAX, '\0');


    ObjectMonitorType monitorData;
    std::string IPaddr;

    COMMAND command = COMM_INV;
    COMMAND SendLater= COMM_INV;

    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
	// TO DO: fix the time here
    struct timespec remTime;
    struct timeval monitorDataReceiveTime;
    uint8_t isoTransmitterID;

    int bytesSent = 0;
    char debug = 0;
    int bytesRead = 0;
    int objectErrorState = 0;
    uint32_t nOBJ;
    int recvDataLength = 0;
    unsigned long socketFlags = O_NONBLOCK;
    int OnTCP = 0;


    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);

    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

    UtilGetTrajDirectoryPath (trajPath.data(), trajPath.size());

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

    LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
    while(!quit){

        TCPHandler TCPServerVisualizer(TCP_VISUALIZATION_SERVER_PORT , "", "Server", 1, socketFlags);
        UDPHandler UDPServerVisualizer (UDP_VISUALIZATION_SERVER_PORT,"",0,"Server");
        OnTCP = TCPServerVisualizer.getConnectionOn();

        while(OnTCP <= 0)
        {
            TCPServerVisualizer.TCPHandlerAccept(5);

            OnTCP = TCPServerVisualizer.getConnectionOn();
            if (OnTCP<0){
                break;
            }
            DataDictionaryGetNumberOfObjects(&nOBJ);

            for (int i = 0; i<nOBJ+1; i++ ){
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

         LogMessage(LOG_LEVEL_INFO, "Connection recived");

        /* So what do I need for the new system here? */

        /* fpro or opro fpro should be sent here
            traj should also be sent.

        */

        if (SendLater == COMM_CONNECT || command == COMM_CONNECT)
        {
            UtilGetTrajDirectoryPath (trajPath.data(), trajPath.size());

            for (const auto & entry : std::experimental::filesystem::directory_iterator(trajPath.data())){
                /* TO DO:
                should have a checker
                here to see that all the objects are connected
                before sending traj to visualizer*/

                std::ifstream file(entry.path());
                std::string line;
                int retval = 0;
                while(std::getline(file,line)){
                    retval = parseTraj(line, TCPBuffer);

                    if (retval == -1){

                        break;
                    }
                    else if ((MONR_BUFFER_LENGTH-retval)<TCPBuffer.size()){

                        //TCPServerVisualizer.sendTCP(TCPBuffer.data(),TCPBuffer.size());
                        TCPBuffer.clear();
                    }


                }

            }
            if (SendLater == COMM_CONNECT){
                SendLater =	COMM_INV;
            }


        }


        while(bytesRead >= 0 && OnTCP > 0){
            DataDictionaryGetNumberOfObjects(&nOBJ);
            DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(),transmitterIDs.size());

            for (int i = 0; i<nOBJ+1; i++ ){
                if (iCommRecv(&command,mqRecvData, MQ_MSG_SIZE, nullptr) < 0)
                {
                    util_error("Message bus receive error");
                }
                switch (command)
                {
                case COMM_CONNECT:
                    if (SendLater == COMM_CONNECT || command == COMM_CONNECT){
                        UtilGetTrajDirectoryPath (trajPath.data(), trajPath.size());

                        for (const auto & entry : std::experimental::filesystem::directory_iterator(trajPath.data())){
                            /* TO DO:
                            should have a checker
                            here to see that all the objects are connected
                            before sending traj to visualizer*/

                            std::ifstream file(entry.path());
                            std::string line;
                            int retval = 0;
                            while(std::getline(file,line)){
                                retval = parseTraj(line, TCPBuffer);

                                if (retval == -1){

                                    break;
                                }
                                else if ((MONR_BUFFER_LENGTH-retval*2)<TCPBuffer.size()){

                                    TCPServerVisualizer.sendTCP(TCPBuffer);
                                    TCPBuffer.clear();
                                }


                            }

                        }
                        if (SendLater == COMM_CONNECT){
                            SendLater =	COMM_INV;
                        }
                    }
                    break;

                default:
                    break;
                }
            }
			
            bytesRead = TCPServerVisualizer.receiveTCP(TCPBuffer, 0);
            bytesSent = UDPServerVisualizer.receiveUDP(UDPBuffer);
			UDPBuffer.clear();
			TCPBuffer.clear();

            transmitterIDs.resize(nOBJ);
            for (auto &transmitterID : transmitterIDs) {

                if (transmitterID == 0){
                    break;
                }

                DataDictionaryGetMonitorData(transmitterID, &monitorData);
                DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);// i am getting duplicates from this one
                if (monitorDataReceiveTime.tv_sec > 0&& monitorData.position.isPositionValid && monitorData.position.isHeadingValid) {
                    // get transmitterid and encode monr to iso.
                    isoTransmitterID = (uint8_t) transmitterID;
                    setTransmitterID(isoTransmitterID); // TO:Do voi message we now cheat wit transmitter id
                    bytesSent = UDPServerVisualizer.receiveUDP(UDPBuffer);
                    long retval = encodeMONRMessage(&monitorData.timestamp,monitorData.position,monitorData.speed,monitorData.acceleration,monitorData.drivingDirection,monitorData.state, monitorData.armReadiness, objectErrorState,UDPBuffer.data(), UDPBuffer.size(),debug);
                    if(retval>=0) {
						UDPBuffer.resize(static_cast<unsigned long>(retval));
						// Cheking so we still connected to visualizer

						bytesRead = TCPServerVisualizer.receiveTCP(TCPBuffer, 0);
						if (bytesRead < 0) {
							LogMessage(LOG_LEVEL_ERROR, "Error when reading from Maestro TCP socket");
							break;

						}
						//n sending message with udp.

						bytesSent = UDPServerVisualizer.sendUDP(UDPBuffer);
					}
				}

            }
            nanosleep(&sleepTimePeriod,&remTime); // sleep might not be needed! but added it becouse i am not sure how much it will use sharedmemory resources


        }
        LogMessage(LOG_LEVEL_INFO, "Disconnected");
        LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
        OnTCP = 0;
        bytesRead = 1;
        TCPServerVisualizer.TCPHandlerclose();
        UDPServerVisualizer.UDPHandlerclose();
    }
}

/**
 * @brief Cought keybord interuptions
 * 
 * @param signo 
 */
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