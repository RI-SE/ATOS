#include <vector>
#include "directcontrol.hpp"
#include "util.h"
#include "tcphandler.hpp"
#include "logging.h"

DirectControl::DirectControl() {

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
	std::vector<char> data(2048);
	int recvData = 0;

	while (!this->quit) {
		// TODO set up TCP connection (wait until connected before continue)
		LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
		TCPHandler tcpHandler(this->commandPort, "", "Server");

		while (!this->quit && tcpHandler.isConnected()) {
			recvData = tcpHandler.receiveTCP(data, 0);
			if (recvData == TCPHandler::TCPHANDLER_FAIL) {
				break;
			}
			else if (recvData > 0) {

				LogPrint("Received %d bytes!", recvData);
			}

			// TODO check for RDCI (optional)
			// TODO if RDCI, respond with DCTI (optional)

			// TODO accept RDCA
			// TODO if format valid, OBC correct state enter into shared memory
			// TODO if not, respond with error (optional)
		}
	}
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
	TCPHandler selfPipe(this->commandPort, "127.0.0.1", "Client", 1, SOCK_NONBLOCK);
}
