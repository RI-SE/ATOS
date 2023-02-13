#pragma once

#include "server.hpp"
#include "loggable.hpp"


class ServerFactory : public Loggable {

  public:
    ServerFactory(const std::string& address, const uint16_t& port, rclcpp::Logger logger);
    ~ServerFactory();

    std::unique_ptr<Server> createServer(const std::string protocol);


  private:
    std::string address;
    uint16_t port;
};