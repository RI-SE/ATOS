#include "connectionhandler.h"
#include "logging.h"

#include <system_error>
#include <unistd.h>
#include <sys/socket.h>
#include <vector>

ConnectionHandler::ConnectionHandler(int openSocketDescriptor, ProtocolData& data)
	: ConnectionHandler::ConnectionHandler(openSocketDescriptor, data, DefaultReadBufferSize) {}

ConnectionHandler::ConnectionHandler(int openSocketDescriptor, ProtocolData& data, unsigned long readBufferSize) : data(data) {
	int returnCode;

	if (openSocketDescriptor < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to create connection handler with invalid socket descriptor");
		return;
	}
	socketDescriptor = openSocketDescriptor;

	if (readBufferSize == 0) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to create connection handler with zero buffer size");
		return;
	}
	this->readBufferSize = readBufferSize;

	LogMessage(LOG_LEVEL_INFO, "Creating thread to handle connection");
	returnCode = pthread_create(&readThread, nullptr, &ConnectionHandler::routineWrapper, this);
	if (returnCode) {
		LogMessage(LOG_LEVEL_ERROR, "Error creating thread");
		close(socketDescriptor);
		terminated = true;
		throw std::system_error();
	}
}

ConnectionHandler::~ConnectionHandler() {
	close(socketDescriptor);
}


void* ConnectionHandler::threadRoutine(void*) {
	std::vector<char> readBuffer(readBufferSize);
	std::vector<char> messageBuffer;
	ssize_t readBytes;

	while ( (readBytes = read(socketDescriptor, readBuffer.data(), readBuffer.size()) ) > 0) {
		messageBuffer.insert(messageBuffer.end(), readBuffer.begin(), readBuffer.end());
		switch (data.decodeFrom(messageBuffer)) {
		case ProtocolData::DECODE_PARTIAL:
			break;
		case ProtocolData::DECODE_SUCCESSFUL:
			messageBuffer.clear();
			break;
		case ProtocolData::DECODE_ERROR:
			messageBuffer.clear();
			break;
		}
	}

	if (readBytes < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Socket read error");
		terminate(static_cast<void*>(&readBytes));
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "Remote closed connection");
		terminate(nullptr);
	}
}

void ConnectionHandler::terminate(void *retval) {
	terminated = true;
	pthread_exit(retval);
}
