#include "connectionhandler.h"
#include "logging.h"
#include <system_error>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <vector>

typedef void* (*ThreadFunctionPointerType)(void *);

ConnectionHandler::ConnectionHandler(int openSocketDescriptor)
	: ConnectionHandler::ConnectionHandler(openSocketDescriptor, DefaultReadBufferSize) {}

ConnectionHandler::ConnectionHandler(int openSocketDescriptor, unsigned long readBufferSize) {
	int returnCode;
	if (openSocketDescriptor < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to create connection handler with invalid socket descriptor");
		return;
	}
	socketDescriptor = openSocketDescriptor;

	if (readBufferSize == 0) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to create connection handler with invalid socket descriptor");
		return;
	}
	this->readBufferSize = readBufferSize;

	LogMessage(LOG_LEVEL_INFO, "Creating thread to handle connection");
	returnCode = pthread_create(&readThread, nullptr, &ConnectionHandler::routineWrapper, this);
	if (returnCode) {
		throw std::system_error();
	}
}


void* ConnectionHandler::threadRoutine(void*) {
	struct sockaddr_in address;
	socklen_t addrlen = sizeof (address);
	std::vector<char> readBuffer(readBufferSize);
	std::vector<char> messageBuffer;
	ssize_t readBytes;

	socketDescriptor = accept(socketDescriptor, (struct sockaddr *)&address, static_cast<socklen_t*>(&addrlen));
	if (socketDescriptor < 0) {
		pthread_exit(static_cast<void*>(&socketDescriptor));
	}

	while ( (readBytes = read(socketDescriptor, readBuffer.data(), readBuffer.size()) ) > 0) {
		messageBuffer.insert(messageBuffer.end(), readBuffer.begin(), readBuffer.end());
		// TODO: Decode raw data into MQ friendly data

	}

	if (readBytes < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Socket read error");
		pthread_exit(static_cast<void*>(&readBytes));
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "Remote closed connection");
		pthread_exit(nullptr);
	}
}
