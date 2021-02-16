#include <vector>
#include "directcontrol.hpp"
#include "util.h"
#include "tcphandler.hpp"
#include "logging.h"
#include "maestroTime.h"

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

		while (!this->quit && tcpHandler.isConnected()) {
			data.resize(TCP_BUFFER_SIZE);
			std::fill(data.begin(), data.end(), 0);
			recvData = tcpHandler.receiveTCP(data, 0);
			if (recvData == TCPHandler::TCPHANDLER_FAIL) {
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

			// TODO check for RDCI (optional)
			// TODO if RDCI, respond with DCTI (optional)

			// TODO if format valid, OBC correct state enter into shared memory
			// TODO if not, respond with error (optional)
		}
	}
}

void DirectControl::handleISOMessage(
		std::vector<char>& byteData,
		size_t receivedBytes) {
	while (receivedBytes > 0) {
		ISOMessageID recvMessage = getISOMessageType(byteData.data(), byteData.size(), false);
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
	if (bytesRead < 0) {
		throw std::invalid_argument("Failed to decode RDCA message");
	}
	byteData.erase(byteData.begin(), byteData.begin() + bytesRead);
	// TODO handle message
	std::cout << "Received RDCA data:" << std::endl
			  << "Timestamp: " << static_cast<double>(recvMessage.dataTimestamp.tv_sec) + recvMessage.dataTimestamp.tv_usec / 1000000.0
			  << " s " << std::endl << "Pinion angle: " << recvMessage.pinionAngle_rad << " rad" << std::endl;
	return static_cast<size_t>(bytesRead);
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
	quit = true;
	// Make the receive socket exit
	TCPHandler selfPipe(this->commandPort, "127.0.0.1", "Client");
	selfPipe.TCPHandlerclose();
}
