#include "udpserver.hpp"

UDPServer::UDPServer(const std::string address, const uint16_t port, const std::string logger) : 
            Server(address, port, logger) {
  
  this->address = address;
  this->port = port;
  this->logger = logger;
}

UDPServer::~UDPServer() {
  destroyServer();
}


void UDPServer::setupServer() {
  endpoint = std::make_shared<ip::udp::endpoint>(ip::make_address_v4(address), port);
  socket = std::make_shared<ip::udp::socket>(*io_service);
  socket->open(ip::udp::v4());
  RCLCPP_DEBUG(get_logger(logger), "Sending OSI-messages on %s:%d", endpoint->address().to_string().c_str(), endpoint->port());
}


void UDPServer::destroyServer() {
  socket->close();
}


void UDPServer::resetServer() {
  destroyServer();
  setupServer();
}


void UDPServer::sendData(std::vector<char> data, boost::system::error_code errorCode) {
  socket->send_to(buffer(data), *endpoint, 0, errorCode);
}