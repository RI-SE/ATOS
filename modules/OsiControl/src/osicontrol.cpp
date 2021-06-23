#include <iostream>
#include <unistd.h>
#include <vector>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "tcphandler.hpp"
#include "osi_handler.hpp"

#define MODULE_NAME "OSI"
#define OSI_RECEIVER_PORT 53250

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
    uint32_t nofObjects;
    uint32_t transmitterIDs[10];
    int32_t TCPSocketfdI32;
    C8 osiReceiverIP[] = {"192.168.0.59"};
    int32_t tcpRes = 0;
 
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
            DataDictionaryGetNumberOfObjects(&nofObjects);
            DataDictionaryGetObjectTransmitterIDs(transmitterIDs, nofObjects);
           // OBCState = (OBCState_t)mqRecvData[0];
            //tcp.CreateClient(53250, "192.168.0.59");

            tcpRes = UtilConnectTCPChannel((const C8*)MODULE_NAME, &TCPSocketfdI32, 
                                            osiReceiverIP, OSI_RECEIVER_PORT);
            break;
        case COMM_STRT:
            nanosleep(&abortWaitTime,&remTime);
            //LogMessage(LOG_LEVEL_WARNING,"Sending ABORT");
            //iCommSend(COMM_ABORT,nullptr,0);
            break;
        case COMM_ABORT:
            //tcp.TCPHandlerclose();
            nanosleep(&abortWaitTime,&remTime);
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
        


        if(OBCState == OBC_STATE_ARMED || OBCState == OBC_STATE_RUNNING){

            ObjectMonitorType monrData;
            for(int i = 0; i < nofObjects; i ++){
                DataDictionaryGetMonitorData(transmitterIDs[i],&monrData);
                
                OsiHandler osi;
                OsiHandler::GlobalObjectGroundTruth_t gt;
                std::chrono::system_clock::time_point ositime;
                gt.id = transmitterIDs[i];
                gt.pos_m.x = monrData.position.xCoord_m;
                gt.pos_m.y = monrData.position.yCoord_m;
                gt.pos_m.z = monrData.position.zCoord_m;
                std::string projstr;
                std::string sendstr;
                /*
                auto xVelocity = MonrUserData->velocity.longitudinal_m_s;
                auto yVelocity = MonrUserData->velocity.lateral_m_s;
                auto zVelocity = 0;
                auto xAcc = MonrUserData->acceleration.longitudinal_m_s2;
                auto yAcc = MonrUserData->acceleration.lateral_m_s2;
                auto zAcc = 0;
                auto secs = MonrUserData->timestamp.tv_sec;
                auto nanos = MonrUserData->timestamp.tv_usec*1000;
                */
                //auto ptr = reinterpret_cast<char*>(&monrData);
                //std::vector<char> buffer(ptr, ptr + sizeof(monrData));

                //std::string protoBuffed;
                sendstr = osi.encodeSvGtMessage(gt, ositime, projstr, true);

                //std::vector<char> inBuffer(protoBuffed.begin(), protoBuffed.end());
                //osi.decodeSvGtMessage(inBuffer, inBuffer.size(), true);

                auto ptr1 = reinterpret_cast<const C8*>(&sendstr);
                UtilSendTCPData((const C8*)MODULE_NAME, ptr1, sendstr.size(),
                           &TCPSocketfdI32, 0);
                //tcp.sendTCP(inBuffer);
            }
            
            
        }

    }

    return 0;
}
