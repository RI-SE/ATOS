#include "server.hpp"


Server::Server(const std::string address, const uint16_t port, rclcpp::Logger logger) : Loggable(logger) {

  this->address = address;
  this->port = port;
  io_service = std::make_shared<boost::asio::io_service>();
}


Server::~Server() {}