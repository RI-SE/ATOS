#pragma once

#include "server.hpp"


class TCPServer : public Server {

  public:
    TCPServer(const std::string& address, const uint16_t& port, rclcpp::Logger& logger);
    ~TCPServer();


  private:
    void setupServer();
    void destroyServer();
    void resetServer();
    void sendData(std::vector<char>& data, boost::system::error_code& errorCode);
    void handleError(boost::system::error_code& errorCode);

    std::shared_ptr<boost::asio::ip::tcp::endpoint> endpoint;
    std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
    std::shared_ptr<boost::asio::ip::tcp::socket> socket;
};