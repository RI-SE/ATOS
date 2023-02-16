#pragma once
#include "socket.hpp"
class Client : public Socket {
public:
	Client(const SocketType type, const bool debug = false);
	Client(const SocketType type, const Address& remoteAddr, const Port port, const bool debug = false);

	void connect();
	void connect(const Address&, const Port port);
	int disconnect();
};

class TCPClient : public Client {
public:
	TCPClient(const bool debug = false);
	TCPClient(const Address& remoteAddr, const Port port, const bool debug = false);
};

class UDPClient : public Client {
public:
	UDPClient(const bool debug = false);
	UDPClient(const Address& remoteAddr, const Port port, const bool debug = false);
};
