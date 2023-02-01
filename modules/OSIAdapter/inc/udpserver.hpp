#pragma once

#include "server.hpp"

using namespace boost::asio;
using namespace rclcpp;


class UDPServer : public Server {

  public:
    UDPServer(const std::string address, const uint16_t port, const std::string logger);
    ~UDPServer();


  private:
    void setupServer();
    void destroyServer();
    void resetServer();
    void sendData(std::vector<char> data, boost::system::error_code errorCode);

    std::shared_ptr<ip::udp::endpoint> endpoint;
    std::shared_ptr<ip::udp::socket> socket;

    std::string address;
    uint16_t port;
    std::string logger;

};