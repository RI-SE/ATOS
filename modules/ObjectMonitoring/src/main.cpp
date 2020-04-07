#include <iostream>
#include <unistd.h>
#include <typeindex>
#include <vector>
#include <algorithm>

#include <MQTTClient.h>

#include "logging.h"
#include "connectionhandler.h"
#include "protocoldata.h"
#include "ISO22133ProtocolData.h"
#include "mqttconnectionhandler.hpp"
#include "util.h"

#define MODULE_NAME "ObjectMonitoring"
#define ISO22133_PORT 53240

using namespace std;

static void handleNewConnection(int socketDescriptor, vector<RawConnectionHandler*> &handlers);
static void pruneTerminatedConnectionHandlers(vector<RawConnectionHandler*> &handlers);
static void listenForNewConnection(void);

int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE];
    const struct timespec sleepTimePeriod = {0,10000000};
    const struct timespec abortWaitTime = {1,0};
	struct timespec remTime;
	vector<RawConnectionHandler*> handlers;

	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

    // Initialize message bus connection
	while(iCommInit()) {
        nanosleep(&sleepTimePeriod,&remTime);
    }

	// TODO: start MQ communication handler

	// Initialize MQTT handler
	MQTTConnectionHandler mqttConnectionHandler("Maestro");
	mqttConnectionHandler.establishConnection();

	while(true) {

		listenForNewConnection();

		handleNewConnection(0, handlers);

		pruneTerminatedConnectionHandlers(handlers);
    }

    return 0;
}

void handleNewConnection(int socketDescriptor, vector<RawConnectionHandler*> &handlers) {
	struct sockaddr_in socketAddress;
	socklen_t addressLength = sizeof (socketAddress);
	getsockname(socketDescriptor, (struct sockaddr *)&socketAddress, &addressLength);

	switch(socketAddress.sin_port) {
	case ISO22133ProtocolData::TCP_PORT:
	{
		ISO22133ProtocolData protoData;
		handlers.push_back(new RawConnectionHandler(socketDescriptor, protoData));
		break;
	}
	default:
		LogMessage(LOG_LEVEL_WARNING, "New connection made but no protocol specified for connected port: closing connection");
		close(socketDescriptor);
		return;
	}
}

void pruneTerminatedConnectionHandlers(vector<RawConnectionHandler*> &handlers) {
	// Remove any connection handlers which are null pointers
	handlers.erase( std::remove(handlers.begin(), handlers.end(), nullptr), handlers.end() );
	// Remove any connection handlers which have finished their tasks
	handlers.erase( std::remove_if(handlers.begin(), handlers.end(),
								   [](const RawConnectionHandler* handler) { return handler->isTerminated(); }
									), handlers.end() );
}


void listenForNewConnection(void) {
	// TODO
}
