#include <vector>
#include "directcontrol.hpp"
#include "util.h"
#include "tcphandler.hpp"
#include "logging.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "maestro_msgs/msg/control_signal_percentage.hpp"

using maestro_msgs::msg::ControlSignalPercentage;

#define TCP_BUFFER_SIZE 2048

/** @class DmMsg
 *  @brief Driver Model message, used in esmini to send
 * control signals to a vehicle from a driver model.
 */
class DmMsg{
public:
	DmMsg(){};

	ControlSignalPercentage toMaestroMsg(){
		ControlSignalPercentage cspmsg = ControlSignalPercentage();
		cspmsg.maestro_header.object_id = this->objectId;
		// Convert throttle brake and steering angle to integers between 0,100 and -100,100 respectively.  
		cspmsg.throttle = round(this->throttle * 100);
		cspmsg.brake = round(this->brake * 100);
		cspmsg.steering_angle = round(this->steeringAngle * 100);
		return cspmsg;
	}

	/*!
	* \brief Decodes a DM message from esmini compatible sender. 
	*			if the message is not of type inputMode=1 (DRIVER_INPUT)
	*			the function returns
	*
	* \param bytes vector of (char) bytes
	* \param DMMsg the data structure to be populated with contents of the message 
	* \return number of bytes on successfully parsed message, -1 otherwise
	*/
	int parseFromBytes(const std::vector<char>& bytes){
		int idx=0;
		decodeNextXBytes(4,this->version,idx,bytes);
		decodeNextXBytes(4,this->inputMode,idx,bytes);
		if (this->inputMode != 1) { return 0;}
		decodeNextXBytes(4,this->objectId,idx,bytes);
		decodeNextXBytes(4,this->frameNumber,idx,bytes);
		decodeNextXBytes(8,this->throttle,idx,bytes);
		decodeNextXBytes(8,this->brake,idx,bytes);
		decodeNextXBytes(8,this->steeringAngle,idx,bytes);
		return idx;
	}

private:
	unsigned int version;
	unsigned int inputMode;
	unsigned int objectId;
	unsigned int frameNumber;
	double throttle;       // range [0, 1]
	double brake;          // range [0, 1]
	double steeringAngle;  // range [-pi/2, pi/2]

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
		if (sizeof(T) == 2){
			le16toh(field);
		}
		else if (sizeof(T) == 4){
			le32toh(field);
		}
		else if (sizeof(T) == 8){
			le64toh(field);
		} 
		idx+=x;
	}

};

//! Message queue callbacks

void DirectControl::onAbortMessage(const Empty::SharedPtr) {}

void DirectControl::onAllClearMessage(const Empty::SharedPtr) {}

//! Class methods

DirectControl::DirectControl() :
	Module(module_name),
	tcpHandler(TCPPort, "", "off", 1, O_NONBLOCK), 
	udpServer("0.0.0.0",UDPPort) {
	//! Publishers
	this->controlSignalPercentagePub = this->create_publisher<ControlSignalPercentage>(topicNames[COMM_CONTROL_SIGNAL_PERCENTAGE],0);
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
 * \brief Listens for UDP data and sends a control signal on ros topic when recevied
 */
void DirectControl::readUDPSocketData() {
	RCLCPP_INFO(get_logger(),"Listening on UDP port %d",UDPPort);
	
	while (!this->quit){
		try{
			auto [data, remote] = udpServer.recvfrom();
			DmMsg dmMsg;

			auto bytesParsed = dmMsg.parseFromBytes(data);
			// TODO: add re-ordering with frame numbers? 
			// Have to think abt if/what kind of re-ordering makes sense for 
			// the application (driver model).
			if (bytesParsed != -1){
				ControlSignalPercentage cspmsg = dmMsg.toMaestroMsg();
				controlSignalPercentagePub->publish(cspmsg);
			}
		}
		catch(const SocketErrors::DisconnectedError& error){
			//If we get a disconnected exception when we do not intend to exit, propagate said exception
			if (!this->quit){
				throw error;
			}
		}
	}
}


void DirectControl::readTCPSocketData() {
	std::vector<char> data(TCP_BUFFER_SIZE);
	int recvData = 0;

	RCLCPP_INFO(get_logger(),"Awaiting TCP connection...");
	this->tcpHandler.CreateServer();

	while (!this->quit) {
		// TODO set up TCP connection (wait until connected before continue)
		this->tcpHandler.TCPHandlerAccept(1000);

		if (this->tcpHandler.isConnected()){
			RCLCPP_INFO(get_logger(),"Connected");
		
			while (!this->quit && tcpHandler.isConnected()) {
				data.resize(TCP_BUFFER_SIZE);
				std::fill(data.begin(), data.end(), 0);
				recvData = tcpHandler.receiveTCP(data, 0);
				if (recvData == TCPHandler::FAILURE) {
					this->tcpHandler.TCPHandlerclose();
					RCLCPP_INFO(get_logger(),"TCP connection closed unexpectedly...");
					RCLCPP_INFO(get_logger(),"Awaiting new TCP connection...");;
					this->tcpHandler.CreateServer();
					break;
				}
				else if (recvData > 0) {
					try {
						this->handleISOMessage(data, static_cast<size_t>(recvData));
					} catch (std::invalid_argument& e) {
						RCLCPP_ERROR(get_logger(),e.what());
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

/*!
 * \brief initializeModule Initializes this module by creating log,
 *			connecting to the message queue bus, setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int DirectControl::initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;

	RCLCPP_INFO(get_logger(), "%s task running with PID: %d",module_name.c_str(), getpid());
	if (requestDataDictInitialization()) {
		if (DataDictionaryInitObjectData() != READ_OK) {
			retval = -1;
			RCLCPP_ERROR(get_logger(), "Preexisting data dictionary not found");
		}
	}
	else{
		retval = -1;
		RCLCPP_ERROR(get_logger(), "Unable to initialize data dictionary");
	}
	return retval;
}