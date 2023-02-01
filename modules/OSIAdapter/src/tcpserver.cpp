#include "tcpserver.hpp"


TCPServer::TCPServer(const std::string address, const uint16_t port, const std::string logger) : 
            Server(address, port, logger) {
  
  this->address = address;
  this->port = port;
  this->logger = logger;
}


TCPServer::~TCPServer() {}

void TCPServer::setupServer() {
  endpoint = 
  acceptor = std::make_shared<ip::tcp::acceptor>(*io_service, endpoint);
  socket = std::make_shared<ip::tcp::socket>(*io_service);
  boost::system::error_code ignored_error;

  RCLCPP_INFO(get_logger(logger), "Waiting for connection on %s:%d", endpoint.address().to_string().c_str(), endpoint.port());
  
  acceptor->accept(*socket, ignored_error);
  while (ignored_error) {
    RCLCPP_DEBUG(get_logger(logger), "Failed to accept the connection from %s:%d, retrying...", endpoint.address().to_string().c_str(), endpoint.port());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    acceptor->accept(*socket, ignored_error);
  }
  RCLCPP_INFO(get_logger(logger), "Connection established with %s:%d", socket->remote_endpoint().address().to_string().c_str(), socket->remote_endpoint().port());

}

void TCPServer::destroyServer() {
  RCLCPP_DEBUG(get_logger(logger), "Destroying TCP Server");
  if (acceptor) {
    acceptor->close();
    acceptor.reset();
  }
  if (socket) {
    socket->close();
    socket.reset();
  }
}

void TCPServer::resetServer() {
  destroyServer();
  setupServer();
}

void TCPServer::sendData(std::vector<char> data, boost::system::error_code errorCode) {
  write(*socket, buffer(data), errorCode);
}
