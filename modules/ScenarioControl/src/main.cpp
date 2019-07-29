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

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

    scenario.initialize("log/dummy_scenariofil.fil");

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
    exit(0);
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
        case COMM_INIT:
            try {
                LogMessage(LOG_LEVEL_INFO, "Initializing scenario");
                scenario.initialize(SCENARIO_FILE_PATH);
            }
            catch (std::invalid_argument) { util_error("Invalid scenario file format"); }
            catch (std::ifstream::failure) { util_error("Unable to open scenario file <" SCENARIO_FILE_PATH ">"); }
            break;
        case COMM_INV:
            nanosleep(&sleepTimePeriod,&remTime);
            break;
        case COMM_OBC_STATE:
            // Ignore the state of object control
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
    }

    return 0;
}

