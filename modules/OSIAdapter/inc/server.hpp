#pragma once

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>


class Server {

  public:
    Server(const std::string address, const uint16_t port, const std::string logger);
    ~Server();

    virtual void setupServer() = 0;
    virtual void destroyServer() = 0;
    virtual void resetServer() = 0;
    virtual void sendData(std::vector<char> data, boost::system::error_code errorCode) = 0;
    

  protected:
    std::shared_ptr<boost::asio::io_service> io_service;
    std::string address;
    uint16_t port;
    std::string logger;

  private:

};