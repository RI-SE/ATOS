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

bool DirectControl::shouldExit(){
	return this->quit;
}

DirectControl::DirectControl() :
	Module(module_name),
	tcpHandler(TCPPort, "", "off", 1, O_NONBLOCK), 
	udpServer("0.0.0.0",UDPPort) {
	// Publishers
	this->dmPub = this->create_publisher<DmMsg>("/dm",0);
}

void DirectControl::startThreads() {
	receiveThread=std::make_unique<std::thread>(&DirectControl::readTCPSocketData, this);
	receiveThreadUDP=std::make_unique<std::thread>(&DirectControl::readUDPSocketData, this);
}

void DirectControl::joinThreads(){
	//Tear down connections
	if(this->tcpHandler.isConnected()) this->tcpHandler.TCPHandlerclose();
	this->udpServer.close();
	//Join threads
	receiveThread->join();
	receiveThreadUDP->join();
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
 *			the function returns
 *
 * \param bytes vector of (char) bytes
 * \param DMMsg the data structure to be populated with contents of the message 
 * \return 1 on success, 0 otherwise
 */
int decodeDMMessage(const std::vector<char>& bytes, DmMsg& DMMsg){
	int idx=0;
	decodeNextXBytes(4,DMMsg.version,idx,bytes);
	decodeNextXBytes(4,DMMsg.input_mode,idx,bytes);
	if (DMMsg.input_mode != 1) { return 0;}
	decodeNextXBytes(4,DMMsg.object_id,idx,bytes);
	decodeNextXBytes(4,DMMsg.frame_number,idx,bytes);
	decodeNextXBytes(8,DMMsg.throttle,idx,bytes);
	decodeNextXBytes(8,DMMsg.brake,idx,bytes);
	decodeNextXBytes(8,DMMsg.steering_angle,idx,bytes);
	return 1;
}

/*!
 * \brief Listens for UDP data and sends DM message on ros topic when recevied
 */
void DirectControl::readUDPSocketData() {
	std::pair<std::vector<char>, BasicSocket::HostInfo> message;
	int wasSuccessful;

	LogMessage(LOG_LEVEL_INFO, "Listening on UDP port %d",UDPPort);
	
	while (!this->quit){
		try{
			message=udpServer.recvfrom();
		}
		catch(const SocketErrors::DisconnectedError& error){
			//If we get a disconnected exception when we do not intend to exit, propagate said exception
			if (!this->quit){
				throw error;
			}
		}
		DmMsg dmmsg = DmMsg();
		wasSuccessful = decodeDMMessage(message.first,dmmsg);
		if (wasSuccessful){
			dmPub->publish(dmmsg);
		}
	}
}


void DirectControl::readTCPSocketData() {
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

void DirectControl::onExitMessage(const Empty::SharedPtr){
	this->quit=true;
}

/*!
 * \brief initializeModule Initializes this module by creating log,
 *			connecting to the message queue bus, setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int DirectControl::initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;

	// Initialize log
	LogInit(module_name.c_str(), logLevel);
	LogMessage(LOG_LEVEL_INFO, "%s task running with PID: %d",module_name.c_str(), getpid());

	if (DataDictionaryInitObjectData() != READ_OK) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Preexisting data dictionary not found");
	}
	return retval;
}

void DirectControl::signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		this->quit = true;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}