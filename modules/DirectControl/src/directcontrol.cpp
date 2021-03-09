#include <vector>
#include "directcontrol.hpp"
#include "util.h"
#include "tcphandler.hpp"
#include "logging.h"
#include "maestroTime.h"
#include "datadictionary.h"

#define TCP_BUFFER_SIZE 2048

DirectControl::DirectControl()
	: tcpHandler(commandPort, "", "off") {
}

int DirectControl::run() {
	const struct timespec sleepTimePeriod = {0, 100000000};
	struct timespec remTime;

	std::thread mqThread(&DirectControl::readMessageBus, this);
	std::thread receiveThread(&DirectControl::readSocketData, this);

	while(!this->quit) {
		nanosleep(&sleepTimePeriod, &remTime);
	}

	mqThread.join();
	receiveThread.join();

	return 0;
}

void DirectControl::readSocketData() {
	std::vector<char> data(TCP_BUFFER_SIZE);
	int recvData = 0;

	while (!this->quit) {
		// TODO set up TCP connection (wait until connected before continue)
		LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
		tcpHandler.CreateServer();
		if (tcpHandler.isConnected()){
			LogMessage(LOG_LEVEL_INFO, "Connected");
		}
		while (!this->quit && tcpHandler.isConnected()) {
			data.resize(TCP_BUFFER_SIZE);
			std::fill(data.begin(), data.end(), 0);
			recvData = tcpHandler.receiveTCP(data, 0);
			if (recvData == TCPHandler::TCPHANDLER_FAIL) {
				this->tcpHandler.TCPHandlerclose();
				break;
			}
			else if (recvData > 0) {
				try {
					this->handleISOMessage(data, static_cast<size_t>(recvData));
				} catch (std::invalid_argument& e) {
					LogMessage(LOG_LEVEL_ERROR, e.what());
					std::fill(data.begin(), data.end(), 0);
				}
			}
		}
	}
}

void DirectControl::handleISOMessage(
		std::vector<char>& byteData,
		size_t receivedBytes) {
	while (receivedBytes > 0) {
		ISOMessageID recvMessage = getISOMessageType(byteData.data(), byteData.size(), false);
		// TODO check for RDCI (optional)
		// TODO if RDCI, respond with DCTI (optional)
		switch (recvMessage) {
		case MESSAGE_ID_INVALID:
			throw std::invalid_argument("Received invalid ISO message");
		case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA:
			receivedBytes -= this->handleRDCAMessage(byteData);
			break;
		default:
			throw std::invalid_argument("Received unhandled ISO message");
		}
	}
}

size_t DirectControl::handleRDCAMessage(
		std::vector<char> &byteData) {
	RequestControlActionType recvMessage;
	struct timeval currentTime;
	TimeSetToCurrentSystemTime(&currentTime);
	ssize_t bytesRead = decodeRDCAMessage(byteData.data(), &recvMessage, byteData.size(), currentTime, false);
	if (bytesRead >= 0) {
		byteData.erase(byteData.begin(), byteData.begin() + bytesRead);
		DataDictionarySetRequestedControlAction(recvMessage.executingID, &recvMessage);
		return static_cast<size_t>(bytesRead);
	}
	else {
		// TODO respond with error (optional)
		throw std::invalid_argument("Failed to decode RDCA message");
	}
}

size_t DirectControl::handleUnknownMessage(
		std::vector<char> &byteData) {
	std::fill(byteData.begin(), byteData.end(), 0);
	return byteData.size();
}

void DirectControl::readMessageBus() {
	COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0, 10000000};
	struct timespec remTime;

	while (!this->quit) {
		if (iCommRecv(&command, mqRecvData, sizeof (mqRecvData), nullptr) < 0) {
			LogMessage(LOG_LEVEL_ERROR,"Message bus receive error");
			this->exit();
		}

		switch (command) {
		case COMM_INV:
			nanosleep(&sleepTimePeriod,&remTime);
			break;
		case COMM_EXIT:
			this->exit();
			break;
		default:
			break;
		}
	}
}

void DirectControl::exit() {
	this->quit = true;
	// Make the receive socket exit
	this->tcpHandler.TCPHandlerclose();
	//TCPHandler selfPipe(this->commandPort, "127.0.0.1", "Client");
	//selfPipe.TCPHandlerclose();
}
