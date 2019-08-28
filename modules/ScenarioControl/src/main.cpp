#include <iostream>
#include <unistd.h>

#include "braketrigger.h"
#include "trigger.h"

#include "scenario.h"
#include "logging.h"
#include "util.h"

#define MODULE_NAME "ScenarioControl"


/************************ Main task ******************************************/
int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    struct timespec remTime;
    Scenario scenario;
    TREOData treo;
    MonitorDataType monr;
    bool terminate = false;
    char configPath[MAX_FILE_PATH];
    UtilGetConfDirectoryPath(configPath, sizeof(configPath));
    strcat(configPath,TRIGGER_ACTION_FILE_NAME);
    enum {UNINITIALIZED, INITIALIZED, CONNECTED, RUNNING} state = UNINITIALIZED;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

    // Initialize message bus connection
    while(iCommInit())
    {
        nanosleep(&sleepTimePeriod,&remTime);
    }


    while(!terminate)
    {
        if (state == RUNNING)
        {
            // Make all active triggers cause their corresponding actions
            scenario.refresh();
            // Allow for retriggering on received TREO messages
            scenario.resetISOTriggers();
        }

        if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0)
        {
            util_error("Message bus receive error");
        }

        switch (command) {
        case COMM_INIT:
            if (state == UNINITIALIZED)
            {
                try {
                    LogMessage(LOG_LEVEL_INFO, "Initializing scenario");
                    scenario.initialize(configPath);
                    state = INITIALIZED;
                }
                catch (std::invalid_argument e) {
                    std::string errMsg = "Invalid scenario file format: " + std::string(e.what());
                    util_error(errMsg.c_str());
                }
                catch (std::ifstream::failure) {
                    std::string errMsg = "Unable to open scenario file <" + std::string(configPath) + ">";
                    util_error(errMsg.c_str());
                }
            }
            break;
        case COMM_OBJECTS_CONNECTED:
            if (state == INITIALIZED)
            {
                state = CONNECTED;
                LogMessage(LOG_LEVEL_INFO, "Distributing scenario configuration");
                scenario.sendConfiguration();
            }
            break;
        case COMM_OBC_STATE:
        case COMM_LOG:
            // Ignore the state of object control and logging messages
            break;
        case COMM_TREO:
            // Decode MQ data
            memcpy(&treo.triggerID, mqRecvData, sizeof(treo.triggerID));
            memcpy(&treo.timestamp_qmsow, mqRecvData+sizeof(treo.triggerID), sizeof(treo.timestamp_qmsow));
            memcpy(&treo.ip, mqRecvData+sizeof(treo.triggerID)+sizeof(treo.timestamp_qmsow), sizeof(treo.ip));

            if (state == RUNNING)
            {
                // Trigger corresponding trigger
                scenario.updateTrigger(treo.triggerID, treo);
            }
            break;
        case COMM_EXAC:
            LogMessage(LOG_LEVEL_ERROR, "Received unexpected execute action message");
            terminate = true;
            break;
        case COMM_TRCM:
            LogMessage(LOG_LEVEL_ERROR, "Received unexpected trigger configuration message");
            terminate = true;
            break;
        case COMM_ACCM:
            LogMessage(LOG_LEVEL_ERROR, "Received unexpected action configuration message");
            terminate = true;
            break;
        case COMM_EXIT:
            LogMessage(LOG_LEVEL_INFO, "Received exit command");
            terminate = true;
            break;
        case COMM_INV:
            nanosleep(&sleepTimePeriod,&remTime);
            break;
        case COMM_ABORT:
            if (state == RUNNING) state = CONNECTED;
            break;
        case COMM_STRT:
            if (state == CONNECTED)
            {
                state = RUNNING;
            }
            else LogMessage(LOG_LEVEL_ERROR, "Received unexpected START command (current state: %u)",static_cast<unsigned char>(state));
            break;
        case COMM_MONR:
            // Update triggers
            UtilPopulateMonitorDataStruct(reinterpret_cast<uint8_t*>(mqRecvData), sizeof(mqRecvData), &monr, 0);
            scenario.updateTrigger(monr);
            break;
        case COMM_MONI:
            // Ignore
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
    }

    return 0;
}

