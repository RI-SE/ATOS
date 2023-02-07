#include "tcpserver.hpp"

using namespace boost::asio;
using namespace rclcpp;

TCPServer::TCPServer(const std::string& address, const uint16_t& port, rclcpp::Logger& logger) : 
            Server(address, port, logger) {}


TCPServer::~TCPServer() {
  destroyServer();
}


/**
 * @brief Setup a TCP server. Opens the server and waits for someone to connect.
 * 
 */
void TCPServer::setupServer() {
  endpoint = std::make_shared<ip::tcp::endpoint>(ip::make_address_v4(address), port);
  acceptor = std::make_shared<ip::tcp::acceptor>(*ioContext, *endpoint);
  socket = std::make_shared<ip::tcp::socket>(*ioContext);
  boost::system::error_code ignored_error;

  RCLCPP_DEBUG(logger, "Waiting for connection on %s:%d", endpoint->address().to_string().c_str(), endpoint->port());
  
  acceptor->accept(*socket, ignored_error);
  while (ignored_error) {
    RCLCPP_DEBUG(logger, "Failed to accept the connection from %s:%d, retrying...", endpoint->address().to_string().c_str(), endpoint->port());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    acceptor->accept(*socket, ignored_error);
  }
  RCLCPP_INFO(logger, "Connection established with %s:%d", socket->remote_endpoint().address().to_string().c_str(), socket->remote_endpoint().port());
}


/**
 * @brief Destroy the TCP server.
 * 
 */
void TCPServer::destroyServer() {
  RCLCPP_DEBUG(logger, "Destroying TCP Server");
  if (acceptor) {
    acceptor->close();
    acceptor.reset();
  }
  if (socket) {
    socket->close();
    socket.reset();
  }
}


/**
 * @brief Reset TCP server.
 * 
 */
void TCPServer::resetServer() {
  destroyServer();
  setupServer();
}


/**
 * @brief Send data over TCP.
 * 
 * @param data Data to be sent
 * @param errorCode Error code
 */
void TCPServer::sendData(std::vector<char>& data, boost::system::error_code& errorCode) {
  write(*socket, buffer(data), errorCode);
}