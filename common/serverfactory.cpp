#include "serverfactory.hpp"


/**
 * @brief Factory for creating TCP and UDP servers.
 * 
 * @param address Address
 * @param port Port
 * @param logger Logger for output
 */
ServerFactory::ServerFactory(const std::string& address, const uint16_t& port, const std::string protocol) {
  this->address = address;
  this->port = port;
  this->protocol = protocol;
}


ServerFactory::~ServerFactory() {
  destroyServer();
}


/**
 * @brief Creates either a TCPServer or a UDPServer.
 * 
 * @param protocol Which protocol to use. Either tcp or udp
 * @return std::unique_ptr<Server> Server object
 */
void ServerFactory::createServer() {
  if (protocol == "tcp") {
    tcpServer = std::make_unique<TCPServer>(this->address, this->port);
  }
  else if (protocol == "udp") {
    udpServer = std::make_unique<UDPServer>(this->address, this->port);
  }
  else {
    throw std::invalid_argument("Protocol must be either tcp or udp");
  }
}


void ServerFactory::setupServer() {
  if (protocol == "tcp") {
    socket = std::make_shared<Socket>(tcpServer->await(address, port));
  }
  else if (protocol == "udp") {
    auto pair = udpServer->recvfrom();
    hostInfo = pair.second;
  }
}


void ServerFactory::destroyServer() {
  if (protocol == "tcp") {
    tcpServer->close();
  }
  else if (protocol == "udp") {
    udpServer->close();
  }
}


void ServerFactory::resetServer() {
  destroyServer();
  setupServer();
}


void ServerFactory::sendData(const std::vector<char>& data) {
  if (protocol == "tcp") {
    std::cerr << "Cc\n";
    socket->send(data, data.size(), Socket::NOSIGNAL);
  }
  else if (protocol == "udp") {
    udpServer->sendto(std::make_pair(data, hostInfo));
  }
}