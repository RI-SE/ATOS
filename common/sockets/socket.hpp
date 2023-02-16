/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once
#define SOCKET_DEBUG

#include "socketexceptions.hpp"
#include <cstdint>
#include <chrono>
#include <map>
#include <vector>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <unistd.h>

class BasicSocket
{
public:
	typedef enum {
		STREAM = SOCK_STREAM,
		DATAGRAM = SOCK_DGRAM
	} SocketType;
	typedef uint16_t Port;
	typedef std::string Address;
	typedef struct {
		Address address;
		Port port;
	} HostInfo;
	typedef enum {
		NO_OPTION = 0,
		NOSIGNAL = MSG_NOSIGNAL
	} MessageOption;

	BasicSocket();
	BasicSocket(const SocketType type, const bool debug = false);
	BasicSocket(const int sockfd, const bool debug = false);
	virtual ~BasicSocket();
	BasicSocket(const BasicSocket& other);
	BasicSocket(BasicSocket&& other);
	BasicSocket& operator=(const BasicSocket& other);
	BasicSocket& operator=(BasicSocket&& other);

	void setDebug(const bool enable = true);
	void setReuseAddr(const bool reuseAddr = true);
	void setKeepAlive(const bool keepAlive = true);
	void setLinger(const bool linger = true);
	void setLingerSeconds(const int);
	void setBlocking(const bool blocking = true);

	bool getBlocking() const;

	SocketType getType() const;
	Address getRemoteIP() const;
	Address getLocalIP() const {throw std::runtime_error("Not implemented");}
	Port getRemotePort() const;
	Port getLocalPort() const;

	void close();
	void open(const SocketType type);
protected:
	typedef enum {
		DEBUG = SO_DEBUG,
		REUSEADDR = SO_REUSEADDR,
		KEEPALIVE = SO_KEEPALIVE,
		LINGER = SO_LINGER,
		TYPE = SO_TYPE
	} SocketOption;
	typedef enum {
		NONBLOCKING = O_NONBLOCK
	} FileOption;
	std::map<SocketOption,std::string> mSocketOptionNames = {
		{DEBUG, "SO_DEBUG"},
		{REUSEADDR, "SO_REUSEADDR"},
		{KEEPALIVE, "SO_KEEPALIVE"},
		{LINGER, "SO_LINGER"},
		{TYPE, "SO_TYPE"}
	};
	std::map<MessageOption, std::string> mMessageOptionNames = {
		{NOSIGNAL, "MSG_NOSIGNAL"}
	};
	std::map<FileOption,std::string> mFileOptionNames = {
		{NONBLOCKING, "O_NONBLOCK"}
	};

	void setOption(const SocketOption option, const int value);
	void setOption(const SocketOption option, const struct linger value);
	int getOption(const SocketOption option) const;
	void setOption(const FileOption option, const bool value);
	int getOption(const FileOption option) const;

	sockaddr_in getRemoteAddr() const;
	sockaddr_in getLocalAddr() const;

	static sockaddr_in toSockaddr(const Address& addr, const Port port);
	static sockaddr_in toSockaddr(const HostInfo& host);
	static HostInfo fromSockaddr(const struct sockaddr_in&);

	int mSockfd = -1;
	bool mDebug = false;

private:
	bool haveInput(const double timeout);
	int preCloseFlush(const std::chrono::system_clock::duration& timeout);
};

class Socket : public BasicSocket
{
public:

	using BasicSocket::BasicSocket;
	Socket(const Socket& other){}
	Socket(Socket&& other);
	Socket& operator=(const Socket& other);
	Socket& operator=(Socket&& other);

	std::vector<char> recv(const MessageOption option = NO_OPTION);
	void send(const std::vector<char>& data, const MessageOption option = NO_OPTION);
	void send(const std::vector<char>& data, const size_t nBytes, const MessageOption option = NO_OPTION);

private:
	static const int IO_BUFFER_SIZE = 4096;
	std::vector<char> recvBuffer = std::vector<char>(IO_BUFFER_SIZE);
	std::vector<char> sendBuffer = std::vector<char>(IO_BUFFER_SIZE);
};



