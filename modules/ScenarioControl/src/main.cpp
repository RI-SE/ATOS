#include <iostream>
#include <unistd.h>

#include "logging.h"
#include "util.h"

#define MODULE_NAME "ScenarioControl"

int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    struct timespec remTime;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

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

            break;
        case COMM_INV:
            nanosleep(&sleepTimePeriod,&remTime);
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
        }
    }

    return 0;
}
