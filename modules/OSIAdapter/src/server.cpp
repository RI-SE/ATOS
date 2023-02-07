#include "server.hpp"


Server::Server(const std::string& address, const uint16_t& port, rclcpp::Logger& logger) : Loggable(logger) {
  this->address = address;
  this->port = port;
  ioContext = std::make_shared<boost::asio::io_context>();
}


Server::~Server() {}