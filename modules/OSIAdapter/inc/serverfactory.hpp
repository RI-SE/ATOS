#pragma once

#include <memory>
#include "server.hpp"


class ServerFactory {

  public:
    ServerFactory(const std::string& address, const uint16_t& port, const std::string protocol);
    ~ServerFactory();

    void createServer();
    void setupServer();
    void destroyServer();
    void resetServer();
    void sendData(const std::vector<char>& data);


  private:
    std::string address;
    uint16_t port;
    std::string protocol;
    std::unique_ptr<TCPServer> tcpServer;
    std::unique_ptr<UDPServer> udpServer;
    std::shared_ptr<Socket> socket;
    BasicSocket::HostInfo hostInfo;
};
