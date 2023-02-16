#pragma once
#include "socket.hpp"

class Server : public BasicSocket {
public:
	Server(const SocketType type, const bool debug = false);
	Server(const SocketType type, const Address& localAddr, const Port port, const bool debug = false);

protected:
	void bind(const SocketType type, const sockaddr_in localAddr);
};

class TCPServer : public Server {
public:
	TCPServer(const bool debug = false);
	TCPServer(const Address& localAddr, const Port port, const bool debug = false);

	virtual Socket await();
	virtual Socket await(const Address& localAddr, const Port port);
protected:
	virtual void listen();
	virtual Socket accept();
private:
	static const unsigned int MAX_QUEUED_CONNECTIONS = 10;
};

class UDPServer : public Server {
public:
	UDPServer(const bool debug = false);
	UDPServer(const Address& localAddr, const Port port, const bool debug = false);

	void bind(const HostInfo& localEndpoint);

	std::pair<std::vector<char>, HostInfo> recvfrom();
	void sendto(const std::pair<const std::vector<char>, const HostInfo>& data);
	void sendto(const std::pair<const std::vector<char>, const HostInfo>& data, const size_t nBytes);
private:
	static const int IO_BUFFER_SIZE = 4096;
	std::vector<char> recvBuffer = std::vector<char>(IO_BUFFER_SIZE);
	std::vector<char> sendBuffer = std::vector<char>(IO_BUFFER_SIZE);
};
