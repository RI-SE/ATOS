#include "serverfactory.hpp"


/**
 * @brief Factory for creating TCP and UDP servers.
 * 
 * @param address Address
 * @param port Port
 * @param logger Logger for output
 */
ServerFactory::ServerFactory(const std::string& address, const uint16_t& port, rclcpp::Logger logger) : Loggable(logger) {
  this->address = address;
  this->port = port;
}


ServerFactory::~ServerFactory() {}


/**
 * @brief Creates either a TCPServer or a UDPServer.
 * 
 * @param protocol Which protocol to use. Either tcp or udp
 * @return std::unique_ptr<Server> Server object
 */
std::unique_ptr<Server> ServerFactory::createServer(const std::string protocol) {
  if (protocol == "tcp") {
    return std::make_unique<TCPServer>(this->address, this->port, logger);
  }
  else if (protocol == "udp") {
    return std::make_unique<UDPServer>(this->address, this->port, logger);
  }
  else {
    throw std::invalid_argument("Protocol must be either tcp or udp");
  }
}