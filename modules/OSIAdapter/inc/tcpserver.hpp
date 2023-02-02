#pragma once

#include "server.hpp"

using namespace boost::asio;
using namespace rclcpp;

class TCPServer : public Server {

  public:
    TCPServer(const std::string address, const uint16_t port, rclcpp::Logger logger);
    ~TCPServer();


  private:
    void setupServer();
    void destroyServer();
    void resetServer();
    void sendData(std::vector<char> data, boost::system::error_code errorCode);

    std::shared_ptr<ip::tcp::endpoint> endpoint;
    std::shared_ptr<ip::tcp::acceptor> acceptor;
    std::shared_ptr<ip::tcp::socket> socket;
};