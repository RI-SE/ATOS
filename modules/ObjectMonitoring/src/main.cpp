#include <iostream>
#include <unistd.h>
#include <typeindex>
#include <vector>

#include "logging.h"
#include "connectionhandler.h"
#include "protocoldata.h"
#include "ISO22133ProtocolData.h"
#include "util.h"

#define MODULE_NAME "ObjectMonitoring"
#define ISO22133_PORT 53240

using namespace std;

static void handleNewConnection(int socketDescriptor, vector<ConnectionHandler*> &handlers);

int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    const struct timespec abortWaitTime = {1,0};
	struct timespec remTime;
	vector<ConnectionHandler*> handlers;

	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

    // Initialize message bus connection
	while(iCommInit()) {
        nanosleep(&sleepTimePeriod,&remTime);
    }

	while(true) {
		// TODO: listen

		handleNewConnection(0, handlers);

    }

    return 0;
}

void handleNewConnection(int socketDescriptor, vector<ConnectionHandler*> &handlers) {
	struct sockaddr_in socketAddress;
	socklen_t addressLength = sizeof (socketAddress);
	getsockname(socketDescriptor, (struct sockaddr *)&socketAddress, &addressLength);

	switch(socketAddress.sin_port) {
	case ISO22133_PORT:
	{
		ISO22133ProtocolData protoData;
		handlers.push_back(new ConnectionHandler(socketDescriptor, protoData));
		break;
	}
	default:
		LogMessage(LOG_LEVEL_WARNING, "New connection made but no protocol specified for connected port: closing connection");
		close(socketDescriptor);
		return;
	}
}
