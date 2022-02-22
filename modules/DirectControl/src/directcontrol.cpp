#include <vector>
#include "directcontrol.hpp"
#include "util.h"
#include "tcphandler.hpp"
#include "logging.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "maestro_msgs/msg/dm_msg.hpp"

using maestro_msgs::msg::DmMsg;

#define TCP_BUFFER_SIZE 2048

void DirectControl::onAbortMessage(const Empty::SharedPtr){}

void DirectControl::onAllClearMessage(const Empty::SharedPtr){}

DirectControl::DirectControl() :
	Module("DirectControl"),
	tcpHandler(commandPort, "", "off", 1, O_NONBLOCK), 
	udpServer("127.0.0.1",UDPPort) {
	this->dmPub = this->create_publisher<DmMsg>("/dm",0);
}

int DirectControl::run() {
	const struct timespec sleepTimePeriod = {0, 100000000};
	struct timespec remTime;

	//std::thread mqThread(&DirectControl::readMessageBus, this);
	//std::thread receiveThread(&DirectControl::readSocketData, this);
	std::thread receiveThreadUDP(&DirectControl::readUDPSocketData, this);

	while(!this->quit) {
		nanosleep(&sleepTimePeriod, &remTime);
	}

	//mqThread.join();
	//receiveThread.join();
	receiveThreadUDP.join();

	return 0;
}

/*!
 * \brief transfers x bytes, on the interval from idx to idx+x, 
 *			from a vector of bytes into a variable
 *
 * \param x number of bytes to transfer
 * \param field variable to receive bytes
 * \param idx starting byte
 * \param bytes vector of (char) bytes
 */
template<typename T>
void decodeNextXBytes(int x, T& field, int& idx, const std::vector<char>& bytes){
	std::memcpy(&field, &(bytes[idx]), sizeof(T));
	if (sizeof(T) == 4){
		le32toh(field);
	}
	else if (sizeof(T) == 8){
		le64toh(field);
	} 
	idx+=x;
}
/*!
 * \brief Decodes a DM message from esmini compatible sender. 
 *			if the message is not of type inputMode=1 (DRIVER_INPUT)
 *			connecting to the message queue bus, setting up signal handers etc.
 *
 * \param bytes vector of (char) bytes
 * \param DMMsg the data structure to be populated with contents of the message 
 * \return 0 on success, 1 otherwise
 */
int decodeDMMessage(const std::vector<char>& bytes, DmMsg& DMMsg){
	int idx=0;
	decodeNextXBytes(4,DMMsg.version,idx,bytes);
	decodeNextXBytes(4,DMMsg.input_mode,idx,bytes);
	if (DMMsg.input_mode != 1) { DMMsg=DmMsg(); return 1;} // reset message and return error
	decodeNextXBytes(4,DMMsg.object_id,idx,bytes);
	decodeNextXBytes(4,DMMsg.frame_number,idx,bytes);
	decodeNextXBytes(8,DMMsg.throttle,idx,bytes);
	decodeNextXBytes(8,DMMsg.brake,idx,bytes);
	decodeNextXBytes(8,DMMsg.steering_angle,idx,bytes);
	return 0;
}

void DirectControl::readUDPSocketData() {
	std::pair<std::vector<char>, BasicSocket::HostInfo> message;
	while (1){
		message=udpServer.recvfrom();
		DmMsg dmmsg = DmMsg();
		decodeDMMessage(message.first,dmmsg);
		dmPub->publish(dmmsg);
	}
}


void DirectControl::readSocketData() {
	std::vector<char> data(TCP_BUFFER_SIZE);
	int recvData = 0;

	LogMessage(LOG_LEVEL_INFO, "Awaiting TCP connection...");
	this->tcpHandler.CreateServer();

	while (!this->quit) {
		// TODO set up TCP connection (wait until connected before continue)
		this->tcpHandler.TCPHandlerAccept(1000);

		if (this->tcpHandler.isConnected()){
			LogMessage(LOG_LEVEL_INFO, "Connected");
		
			while (!this->quit && tcpHandler.isConnected()) {
				data.resize(TCP_BUFFER_SIZE);
				std::fill(data.begin(), data.end(), 0);
				recvData = tcpHandler.receiveTCP(data, 0);
				if (recvData == TCPHandler::FAILURE) {
					this->tcpHandler.TCPHandlerclose();
					LogMessage(LOG_LEVEL_INFO, "TCP connection closed unexpectedly...");
					LogMessage(LOG_LEVEL_INFO, "Awaiting new TCP connection...");
					this->tcpHandler.CreateServer();
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
			LogMessage(LOG_LEVEL_INFO, "Exit received");
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
	if(this->tcpHandler.isConnected()) this->tcpHandler.TCPHandlerclose();
	//TCPHandler selfPipe(this->commandPort, "127.0.0.1", "Client");
	//selfPipe.TCPHandlerclose();
}
