#include "client.hpp"
#include <cstring>

using namespace SocketErrors;

Client::Client(const SocketType type, const bool debug) : Socket(type, debug) {}
Client::Client(
		const SocketType type,
		const Address& remoteAddr,
		const Port port,
		const bool debug)
	: Client(type, debug) {
	connect(remoteAddr, port);
}

TCPClient::TCPClient(const bool debug) : Client(STREAM, debug) {}
TCPClient::TCPClient(
		const Address& remoteAddr,
		const Port port,
		const bool debug)
	: Client(STREAM, remoteAddr, port, debug) {
}

UDPClient::UDPClient(const bool debug) : Client(DATAGRAM, debug) {}
UDPClient::UDPClient(
		const Address& remoteAddr,
		const Port port,
		const bool debug)
	: Client(DATAGRAM, remoteAddr, port, debug) {
}

/*!
 * \brief Client::connect Connect the client to a remote host.
 * \param remoteAddr Address of the remote host.
 * \param port Port on the remote host.
 */
void Client::connect(
		const Address& remoteAddr,
		const Port port) {
	if (remoteAddr.empty() || port == 0) {
		throw ArgumentError("Empty address or port specified for connect call");
	}
	auto addr = toSockaddr(remoteAddr, port);
	if (::connect(mSockfd, reinterpret_cast<struct sockaddr *>(&addr),
				  sizeof (addr)) < 0) {
		throw SocketConnectError(errno);
	}
}

