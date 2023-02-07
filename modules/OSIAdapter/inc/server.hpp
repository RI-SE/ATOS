#pragma once

#include <boost/asio.hpp>

#include "loggable.hpp"



class Server : public Loggable {

  public:
    Server(const std::string address, const uint16_t port, rclcpp::Logger logger);
    ~Server();

    virtual void setupServer() = 0;
    virtual void destroyServer() = 0;
    virtual void resetServer() = 0;
    virtual void sendData(std::vector<char>& data, boost::system::error_code& errorCode) = 0;
    

  protected:
    std::string address;
    uint16_t port;
    std::shared_ptr<boost::asio::io_context> ioContext;
};