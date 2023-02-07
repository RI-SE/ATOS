#include "udpserver.hpp"

using namespace boost::asio;
using namespace rclcpp;

UDPServer::UDPServer(const std::string& address, const uint16_t& port, rclcpp::Logger& logger) : 
            Server(address, port, logger) {}


UDPServer::~UDPServer() {
  destroyServer();
}


/**
 * @brief Setup and opens a UDP server.
 * 
 */
void UDPServer::setupServer() {
  endpoint = std::make_shared<ip::udp::endpoint>(ip::make_address_v4(address), port);
  socket = std::make_shared<ip::udp::socket>(*ioContext);
  socket->connect(*endpoint);
  RCLCPP_DEBUG(logger, "Sending OSI-messages on %s:%d", endpoint->address().to_string().c_str(), endpoint->port());
}


/**
 * @brief Destroy the UDP server.
 * 
 */
void UDPServer::destroyServer() {
  socket->close();
}


/**
 * @brief Reset the UDP server.
 * 
 */
void UDPServer::resetServer() {
  destroyServer();
  setupServer();
}


/**
 * @brief Send data over UDP.
 * 
 * @param data Data to be sent
 * @param errorCode Error code
 */
void UDPServer::sendData(std::vector<char>& data, boost::system::error_code& errorCode) {
  socket->send_to(buffer(data), *endpoint, 0, errorCode);
}


/**
 * @brief Handle the error code if fails to send data.
 * 
 * @param errorCode The error code from trying to send data
 */
void UDPServer::handleError(boost::system::error_code& errorCode) {
  if (errorCode.value() != 0){
    resetServer();
  }
}
