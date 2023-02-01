#include "server.hpp"

Server::Server(const std::string address, const uint16_t port, const std::string logger) {

  this->address = address;
  this->port = port;
  this->logger = logger;
  io_service = std::make_shared<boost::asio::io_service>();
}


Server::~Server() {}