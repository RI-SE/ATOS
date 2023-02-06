#pragma once

#include "server.hpp"



class UDPServer : public Server {

  public:
    UDPServer(const std::string address, const uint16_t port, rclcpp::Logger logger);
    ~UDPServer();


  private:
    void setupServer();
    void destroyServer();
    void resetServer();
    void sendData(std::vector<char> data, boost::system::error_code errorCode);

    std::shared_ptr<boost::asio::ip::udp::endpoint> endpoint;
    std::shared_ptr<boost::asio::ip::udp::socket> socket;
};