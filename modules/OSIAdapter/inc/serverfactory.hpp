#pragma once

#include "tcpserver.hpp"
#include "udpserver.hpp"


class ServerFactory {

  public:
    ServerFactory(const std::string address, const uint16_t port, const std::string logger);
    ~ServerFactory();

  private:
    std::unique_ptr<Server> createServer(const std::string protocol);

    std::string address;
    uint16_t port;
    std::string logger;

    std::unique_ptr<Server> server;

};