#include "serverfactory.hpp"


ServerFactory::ServerFactory(const std::string address, const uint16_t port, const std::string logger) {
  this->address = address;
  this->port = port;
  this->logger = logger;
}

ServerFactory::~ServerFactory() {}


std::unique_ptr<Server> ServerFactory::createServer(const std::string protocol) {
  if (protocol == "tcp") {
    return std::make_unique<TCPServer>(this->address, this->port, this->logger);
  }
  else if (protocol == "udp") {
    return std::make_unique<UDPServer>(this->address, this->port, this->logger);
  }
}