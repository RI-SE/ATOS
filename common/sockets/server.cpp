#include "server.hpp"
#include "socketexceptions.hpp"
#include <cstring>

using namespace SocketErrors;

Server::Server(
		const SocketType type,
		const bool debug)
	: BasicSocket(type, debug) {
	setReuseAddr(true);
}

Server::Server(
		const SocketType type,
		const Address& localAddr,
		const Port port,
		const bool debug) : Server(type, debug) {
	bind(type, toSockaddr(localAddr, port));
}

/*!
 * \brief Server::bind Assigns a name to the server, causing it to be
 *			locally bound.
 * \param localAddr Local address to which server shall be bound.
 */
void Server::bind(const SocketType type,
		const sockaddr_in localAddr) {

	if (::bind(mSockfd, reinterpret_cast<const struct sockaddr*>(&localAddr),
			   sizeof (localAddr)) >= 0) {
		return;
	}
	if (errno == EBADF) {
		// Socket was not previously initialised
		if ((mSockfd = ::socket(AF_INET, type, 0)) < 0) {
			throw SocketCreateError(errno);
		}
		if (::bind(mSockfd, reinterpret_cast<const struct sockaddr*>(&localAddr),
				   sizeof (localAddr)) < 0) {
			throw SocketBindError(errno);
		}
	}
	else if (errno == EINVAL) {
		if (localAddr.sin_port == getLocalAddr().sin_port
				&& localAddr.sin_addr.s_addr == getLocalAddr().sin_addr.s_addr) {
			// Socket already bound and address same, do nothing
			return;
		}
		else {
			// Socket already bound and address different, rebind
			close();
			return bind(type, localAddr);
		}
	}
	else {
		throw SocketBindError(errno);
	}
}


/*!
 * \brief TCPServer::await Await and accept a connection attempt from
 *			remote client. Assumes that the local address has been
 *			previously set.
 * \return A new ::Socket for the accepted connection, connected
 *			to a single remote client.
 */
Socket TCPServer::await() {
	if (getLocalAddr().sin_port == 0) {
		throw ArgumentError("Empty port specified for await call");
	}
	listen();
	return accept();
}

/*!
 * \brief TCPServer::await Await and accept a connection attempt from
 *			remote client, using the specified local listening address.
 * \param localAddr Listening address to use.
 * \param port Listening port to use.
 * \return A new ::Socket for the accepted connection, connected
 *			to a single remote client.
 */
Socket TCPServer::await(
		const Address& localAddr,
		const Port port) {
	auto saddr = toSockaddr(localAddr, port);
	bind(STREAM, saddr);
	return await();
}

/*!
 * \brief TCPServer::listen Opens the server for incoming connections.
 */
void TCPServer::listen() {
	if (mSockfd == -1) {
		throw DisconnectedError();
	}
	if (::listen(mSockfd, MAX_QUEUED_CONNECTIONS) < 0) {
		throw SocketListenError(errno);
	}
}

/*!
 * \brief TCPServer::accept Accepts the next incoming connection,
 *			performing e.g. a TCP handshake. This call can be made
 *			repeatedly to accept several TCP connections on the same
 *			port. One new connection is returned for each accept call.
 * \return A new Socket representing the created connection.
 */
Socket TCPServer::accept() {
	if (mSockfd == -1) {
		throw DisconnectedError();
	}
	sockaddr_in cliaddr;
	socklen_t addrlen = sizeof (cliaddr);
	auto conn = ::accept(mSockfd, reinterpret_cast<struct sockaddr*>(&cliaddr),
						 &addrlen);
	if (conn < 0) {
		throw SocketAcceptError(errno);
	}
	return Socket(conn, mDebug);
}

/*!
 * \brief UDPServer::bind Assigns the specified local address to
 *			the server. This also opens the port for incoming data.
 * \param localEndpoint Local address to which the server shall bind.
 */
void UDPServer::bind(
		const HostInfo &localEndpoint) {
	auto saddr = toSockaddr(localEndpoint);
	return Server::bind(DATAGRAM, saddr);
}

/*!
 * \brief UDPServer::recvfrom Receives the next incoming data packet
 *			and returns the data as well as information on the sender.
 * \return A std::pair containing the data and host info.
 */
std::pair<std::vector<char>, BasicSocket::HostInfo> UDPServer::recvfrom() {
	sockaddr_in addr;
	socklen_t socksize = sizeof (addr);
	auto bytesRead = ::recvfrom(mSockfd, recvBuffer.data(), recvBuffer.size(),
								0, reinterpret_cast<struct sockaddr*>(&addr),
								&socksize);
	if (bytesRead < 0) {
		if (errno == EBADF) {
			throw DisconnectedError();
		}
		throw SocketRecvFromError(errno);
	}
	if (bytesRead == 0 && mSockfd == -1) {
		throw DisconnectedError();
	}
	auto first = recvBuffer.cbegin();
	auto last = recvBuffer.cbegin()+bytesRead;
	return {std::vector<char>(first, last), fromSockaddr(addr)};
}

/*!
 * \brief UDPServer::sendto Sends data to the specified address. Sends
 *			the entire contents of the data vector.
 * \param data A std::pair of data and remote host info.
 */
void UDPServer::sendto(
		const std::pair<const std::vector<char>, const HostInfo> &data) {
	return sendto(data, data.first.size());
}

/*!
 * \brief UDPServer::sendto Sends data to the specified address.
 * \param data A std::pair of data and remote host info.
 * \param nBytes Number of bytes from the start of the data vector to send.
 */
void UDPServer::sendto(
		const std::pair<const std::vector<char>, const HostInfo> &data,
		const size_t nBytes) {
	if (nBytes > data.first.size()) {
		throw ArgumentError("Bytes to send exceeds data size");
	}
	auto saddr = toSockaddr(data.second);
	auto bytesSent = ::sendto(mSockfd, data.first.data(), nBytes, 0,
							  reinterpret_cast<struct sockaddr*>(&saddr),
							  sizeof (saddr));
	if (bytesSent < 0) {
		if (errno == EBADF) {
			throw DisconnectedError();
		}
		throw SocketSendToError(errno);
	}
}

TCPServer::TCPServer(const bool debug) : Server(STREAM, debug) {}
TCPServer::TCPServer(
		const Address& localAddr,
		const Port port,
		const bool debug)
	: Server(STREAM, localAddr, port, debug) {
}
UDPServer::UDPServer(const bool debug) : Server(DATAGRAM, debug) {}
UDPServer::UDPServer(
		const Address& localAddr,
		const Port port,
		const bool debug)
	: Server(DATAGRAM, localAddr, port, debug) {
}
