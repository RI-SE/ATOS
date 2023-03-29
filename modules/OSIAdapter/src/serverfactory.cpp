/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "serverfactory.hpp"


/**
 * @brief Factory for creating TCP and UDP servers.
 * 
 * @param address Address
 * @param port Port
 * @param protocol Either "tcp" or "udp"
 */
ServerFactory::ServerFactory(const std::string& address, const uint16_t& port, const std::string protocol) {
  this->address = address;
  this->port = port;
  this->protocol = protocol;
}


/**
 * @brief Destroy the Server Factory:: Server Factory object
 * 
 */
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


/**
 * @brief Setup the server. TCP: Wait for a connection on address and port. UDP: Wait for someone to connect
 *        in order to get address and port to send to,
 * 
 */
void ServerFactory::setupServer() {
  if (protocol == "tcp") {
    socket = std::make_shared<Socket>(tcpServer->await(address, port));
  }
  else if (protocol == "udp") {
    auto pair = udpServer->recvfrom();
    hostInfo = pair.second;
  }
}


/**
 * @brief Close the server.
 * 
 */
void ServerFactory::destroyServer() {
  if (protocol == "tcp") {
    tcpServer->close();
  }
  else if (protocol == "udp") {
    udpServer->close();
  }
}


/**
 * @brief Reset the server.
 * 
 */
void ServerFactory::resetServer() {
  destroyServer();
  setupServer();
}


/**
 * @brief Send data to the client.
 * 
 * @param data The data to be sent
 */
void ServerFactory::sendData(const std::vector<char>& data) {
  if (protocol == "tcp") {
    socket->send(data, Socket::NOSIGNAL);
  }
  else if (protocol == "udp") {
    udpServer->sendto(std::make_pair(data, hostInfo));
  }
}