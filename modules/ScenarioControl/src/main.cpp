#include <iostream>
#include <unistd.h>

#include "braketrigger.h"
#include "trigger.h"

#include "scenario.h"
#include "logging.h"
#include "util.h"

#define MODULE_NAME "ScenarioControl"

#define SCENARIO_FILE_PATH "path/to/file"

/************************ Main task ******************************************/
int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    struct timespec remTime;
    Scenario scenario;
    TREOData treo;
    bool terminate = false;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

    // Initialize message bus connection
    while(iCommInit())
    {
        nanosleep(&sleepTimePeriod,&remTime);
    }

    //exit(0);

    while(!terminate)
    {
        // Make all active triggers cause their corresponding actions
        scenario.refresh();
        // Allow for retriggering on received TREO messages
        scenario.resetISOTriggers();

        if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0)
        {
            util_error("Message bus receive error");
        }

        switch (command) {
        case COMM_INIT:
            try {
                LogMessage(LOG_LEVEL_INFO, "Initializing scenario");
                //scenario.initialize(SCENARIO_FILE_PATH);
                scenario.initialize("log/dummy_scenariofil.fil");
            }
            catch (std::invalid_argument) { util_error("Invalid scenario file format"); }
            catch (std::ifstream::failure) { util_error("Unable to open scenario file <" SCENARIO_FILE_PATH ">"); }
            break;
        case COMM_OBJECTS_CONNECTED:
            LogMessage(LOG_LEVEL_INFO, "Distributing scenario configuration");
            scenario.sendConfiguration();
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

            // Trigger corresponding trigger
            scenario.updateTrigger(treo.triggerID, treo);
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
        case COMM_STRT:

            // PLACEHOLDER
            LogMessage(LOG_LEVEL_INFO,"1");
            scenario.updateTrigger(1,false);
            scenario.refresh();
            LogMessage(LOG_LEVEL_INFO,"2");
            scenario.updateTrigger(1,false);
            scenario.refresh();
            LogMessage(LOG_LEVEL_INFO,"3");
            scenario.updateTrigger(1,true);
            scenario.refresh();
            LogMessage(LOG_LEVEL_INFO,"4");
            scenario.updateTrigger(1,true);
            scenario.refresh();
            LogMessage(LOG_LEVEL_INFO,"5");
            scenario.updateTrigger(1,false);
            scenario.refresh();
            LogMessage(LOG_LEVEL_INFO,"6");

        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
    }

    return 0;
}

