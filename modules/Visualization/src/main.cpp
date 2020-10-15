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
/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/

int main(int argc, char const* argv[]){

    std::vector<char> TCPBuffer;
    std::vector<char> UDPBuffer(MONR_BUFFER_LENGTH);
    std::vector<uint32_t> transmitterIDs(MONR_BUFFER_LENGTH);
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
    uint8_t isoTransmitterID;

    int bytesSent = 1;
    char debug = 0;
    int bytesRead = 1;
    int objectErrorState = 0;
    uint32_t nOBJ;
    int recvDataLength = 0;
    unsigned long flagsock = O_NONBLOCK;
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

        TCPHandler TCPServerVisualizer(TCP_VISUALIZATION_SERVER_PORT , "", "Server", 1, O_NONBLOCK);
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
            //UtilGetObjectDirectoryPath

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

            bytesRead = TCPServerVisualizer.receiveTCP(chekingTCPconn, 0);
            bytesSent = UDPServerVisualizer.receiveUDP(chekingUDPconn);

            transmitterIDs.resize(nOBJ);
            for (auto &transmitterID : transmitterIDs) {

                if (transmitterID <= 0){
                    break;
                }

                DataDictionaryGetMonitorData(transmitterID, &monitorData);
                DataDictionaryGetMonitorDataReceiveTime(transmitterID, &monitorDataReceiveTime);// i am getting duplicates from this one
                if (monitorDataReceiveTime.tv_sec > 0&& monitorData.position.isPositionValid && monitorData.position.isHeadingValid) {
                    // get transmitterid and encode monr to iso.
                    isoTransmitterID = (uint8_t) transmitterID;
                    setTransmitterID(isoTransmitterID); // TO:Do voi message we now cheat wit transmitter id
                    bytesSent = UDPServerVisualizer.receiveUDP(chekingUDPconn);

                    PeerObjectInjectionType podi;
                    podi.speed = monitorData.speed;
                    podi.state = monitorData.state;
                    podi.position = monitorData.position;
                    //podi.roll_rad =
                    //podi.pitch_rad =
                    //podi.isRollValid =
                    //podi.isPitchValid =
                    podi.foreignTransmitterID = transmitterID;
                    podi.dataTimestamp = monitorData.timestamp;

                    long retval = encodePODIMessage(&podi, UDPBuffer.data(), UDPBuffer.size(), debug);
                    UDPBuffer.resize(static_cast<unsigned long>(retval));
                    // Cheking so we still connected to visualizer

                    bytesRead = TCPServerVisualizer.receiveTCP(chekingTCPconn, 0);
                    if (bytesRead < 0) {
                         LogMessage(LOG_LEVEL_ERROR, "Error when reading from Maestro TCP socket");
                        break;

                    }
                    //n sending message with udp.

                    bytesSent = UDPServerVisualizer.sendUDP(UDPBuffer);
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
