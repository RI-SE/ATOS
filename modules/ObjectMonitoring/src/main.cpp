#include <iostream>
#include <unistd.h>

#include "logging.h"
#include "util.h"

#define MODULE_NAME "ObjectMonitoring"

int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    const struct timespec abortWaitTime = {1,0};
    struct timespec remTime;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

    // Initialize message bus connection
	while(iCommInit()) {
        nanosleep(&sleepTimePeriod,&remTime);
    }

	while(true) {

    }

    return 0;
}
