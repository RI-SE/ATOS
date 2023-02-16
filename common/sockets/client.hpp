/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

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
