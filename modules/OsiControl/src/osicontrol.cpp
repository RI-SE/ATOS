#include <iostream>
#include <unistd.h>
#include <vector>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "tcphandler.hpp"
#include "osi_handler.h"

#define MODULE_NAME "OSI"

int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    const struct timespec abortWaitTime = {1,0};
    struct timespec remTime;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());
    LogMessage(LOG_LEVEL_INFO, "This is OSI!");

    OBCState_t OBCState;
    DataDictionaryInitObjectData();
    TCPHandler tcp;
    std::vector<char> msg(1000);

 
    // Initialize message bus connection
    while(iCommInit())
    {
        nanosleep(&sleepTimePeriod,&remTime);
    }

    while(true)
    {
        if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0)
        {
            util_error("Message bus receive error");
        }

        switch (command) {
        case COMM_INV:
            nanosleep(&sleepTimePeriod,&remTime);
            break;
        case COMM_OBC_STATE:
            OBCState = (OBCState_t)mqRecvData[0];
            printf("OBCState = %d\n", OBCState);
            break;
        case COMM_CONNECT:
           // OBCState = (OBCState_t)mqRecvData[0];
            tcp.CreateClient(53250, "192.168.0.59");
            tcp.TCPHandlerclose();
            tcp.receiveTCP(msg, 1000);
            break;
        case COMM_STRT:
            nanosleep(&abortWaitTime,&remTime);
            //LogMessage(LOG_LEVEL_WARNING,"Sending ABORT");
            //iCommSend(COMM_ABORT,nullptr,0);
            break;
        case COMM_ABORT:
            tcp.TCPHandlerclose();
            nanosleep(&abortWaitTime,&remTime);
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
        


        if(OBCState == OBC_STATE_ARMED || OBCState == OBC_STATE_RUNNING){
                 uint32_t numberOfObjects;
           DataDictionaryGetNumberOfObjects(&numberOfObjects);
            if (numberOfObjects<=0){
                return 0;
            }
 
            ObjectMonitorType monrData;
            DataDictionaryGetMonitorData(8,&monrData);
            printf("y = %3.3f\n", monrData.position.yCoord_m);
            OsiHandler osi;
            std::string mystr;
            std::vector<char> myChar;
            osi.encodeSvGtMessage(myChar,10, &mystr, true);
            //encodeSvGtMessage(std::vector<char> msg, int msgSize, bool debug);
            //encodeOSI

            //tcp.transmitTCP();
        }

    }

    return 0;
}
