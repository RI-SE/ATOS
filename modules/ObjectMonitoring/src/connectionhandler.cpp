#include "connectionhandler.h"
#include "logging.h"

#include <system_error>
#include <unistd.h>
#include <sys/socket.h>
#include <vector>

RawConnectionHandler::RawConnectionHandler(int openSocketDescriptor, ProtocolData& data)
	: RawConnectionHandler::RawConnectionHandler(openSocketDescriptor, data, DefaultReadBufferSize) {}

RawConnectionHandler::RawConnectionHandler(int openSocketDescriptor, ProtocolData& data, unsigned long readBufferSize) : data(data) {
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
	returnCode = pthread_create(&readThread, nullptr, &RawConnectionHandler::routineWrapper, this);
	if (returnCode) {
		LogMessage(LOG_LEVEL_ERROR, "Error creating thread");
		close(socketDescriptor);
		terminated = true;
		throw std::system_error();
	}
}

RawConnectionHandler::~RawConnectionHandler() {
	close(socketDescriptor);
}


void* RawConnectionHandler::threadRoutine(void*) {
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

void RawConnectionHandler::terminate(void *retval) {
	terminated = true;
	pthread_exit(retval);
}
