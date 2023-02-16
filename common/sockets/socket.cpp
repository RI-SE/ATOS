#include "socket.hpp"
#include <fcntl.h>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>

using namespace SocketErrors;

BasicSocket::BasicSocket() {
}

BasicSocket::BasicSocket(
		const SocketType type,
		const bool debug) {
	if ((mSockfd = socket(AF_INET, type, 0)) < 0) {
		throw SocketCreateError(errno);
	}
	setDebug(debug);
}

BasicSocket::BasicSocket(
		const int sockfd,
		const bool debug) {
	mSockfd = sockfd;
	setDebug(debug);
}

BasicSocket::~BasicSocket() {
	close();
}

BasicSocket::BasicSocket(
		const BasicSocket& other) {
	mSockfd = dup(other.mSockfd);
	mDebug = other.mDebug;
}

BasicSocket::BasicSocket(BasicSocket&& other) {
	mSockfd = other.mSockfd;
	mDebug = other.mDebug;
	other.mSockfd = -1;
}
BasicSocket& BasicSocket::operator=(
		const BasicSocket& other) {
	if (this != &other) {
		mSockfd = dup(other.mSockfd);
		mDebug = other.mDebug;
	}
	return *this;
}

BasicSocket& BasicSocket::operator=(BasicSocket &&other) {
	if (this != &other) {
		mSockfd = other.mSockfd;
		other.mSockfd = -1;
	}
	return *this;
}

Socket::Socket(Socket&& other) : BasicSocket(std::move(other)) {
	recvBuffer = other.recvBuffer;
	sendBuffer = other.sendBuffer;
}

Socket& Socket::operator=(Socket &&other) {
	if (this != &other) {
		BasicSocket::operator=(std::move(other));
		recvBuffer = other.recvBuffer;
		sendBuffer = other.sendBuffer;
	}
	return *this;
}


/*!
 * \brief BasicSocket::getLocalAddr Returns the current address to which the socket
 *		is bound.
 * \return sockaddr_in struct containing the bound local address.
 */
sockaddr_in BasicSocket::getLocalAddr() const {
	sockaddr_in ret;
	socklen_t addrlen = sizeof (ret);
	if (getsockname(mSockfd, reinterpret_cast<struct sockaddr*>(&ret),
					&addrlen) < 0) {
		throw SocketGetSockNameError(errno);
	}
	return ret;
}

/*!
 * \brief BasicSocket::getRemoteAddr Returns the address of the peer connected
 *		to the socket.
 * \return sockaddr_in struct containing the connected remote address.
 */
sockaddr_in BasicSocket::getRemoteAddr() const {
	sockaddr_in ret;
	socklen_t addrlen;
	if (getpeername(mSockfd, reinterpret_cast<struct sockaddr*>(&ret),
					&addrlen) < 0) {
		throw SocketGetPeerNameError(errno);
	}
	return ret;
}

/*!
 * \brief BasicSocket::setOption Manipulate options for the socket.
 * \param option Option to manipulate.
 * \param value Value to set on the option.
 */
void BasicSocket::setOption(
		const SocketOption option,
		const int value) {
	if (setsockopt(mSockfd, SOL_SOCKET, option,
				   &value, sizeof (value)) < 0) {
		throw SetSockOptError(mSocketOptionNames[option], errno);
	}
}

/*!
 * \brief BasicSocket::setOption Manipulate options for the socket.
 * \param option Option to manipulate.
 * \param value Value to set on the option.
 */
void BasicSocket::setOption(
		const SocketOption option,
		const struct linger value) {
	if (setsockopt(mSockfd, SOL_SOCKET, static_cast<int>(option),
				   &value, sizeof (value)) < 0) {
		throw SetSockOptError(mSocketOptionNames[option], errno);
	}
}

/*!
 * \brief BasicSocket::setOption Manipulate options for the socket.
 * \param option Option to manipulate.
 * \param value Value to set on the option.
 */
void BasicSocket::setOption(
		const FileOption option,
		const bool value) {
	auto opt = fcntl(mSockfd, F_GETFL);
	if (opt < 0) {
		throw FileControlGetError(mFileOptionNames[option], errno);
	}
	if (fcntl(mSockfd, F_SETFL, value ? opt | option : opt & ~option) < 0) {
		throw FileControlSetError(mFileOptionNames[option], errno);
	}
}

/*!
 * \brief BasicSocket::getOption Fetch options for the socket.
 * \param option Option to fetch.
 * \return Value of the option.
 */
int BasicSocket::getOption(const SocketOption option) const {
	int retval = 0;
	socklen_t retsize = sizeof (retval);
	if (getsockopt(mSockfd, SOL_SOCKET, option, &retval, &retsize) < 0) {
		throw GetSockOptError(mSocketOptionNames.at(option), errno);
	}
	return retval & option;
}

/*!
 * \brief BasicSocket::getOption Fetch options for the socket.
 * \param option Option to fetch.
 * \return Value of the option.
 */
int BasicSocket::getOption(const FileOption option) const {
	auto opt = fcntl(mSockfd, F_GETFL);
	if (opt < 0) {
		throw FileControlGetError(mFileOptionNames.at(option), errno);
	}
	return opt & option;
}

/*!
 * \brief BasicSocket::setDebug Manipulate the debug option for the
 *			socket.
 * \param enable Enable or disable debugging (defaults to true).
 */
void BasicSocket::setDebug(
		const bool enable) {
	mDebug = enable;
}

/*!
 * \brief BasicSocket::setReuseAddr Manipulate the reuse address option
 *			for the socket.
 * \param reuseAddr Enable or disable reuse of address (defaults to true).
 */
void BasicSocket::setReuseAddr(
		const bool reuseAddr) {
	return setOption(REUSEADDR, reuseAddr);
}

/*!
 * \brief BasicSocket::setKeepAlive Manipulate the keepalive option
 *			for the socket.
 * \param keepAlive Enable or disable sending of keep-alive messages
 *			on connection-oriented sockets (defaults to true).
 */
void BasicSocket::setKeepAlive(
		const bool keepAlive) {
	return setOption(KEEPALIVE, keepAlive);
}

/*!
 * \brief BasicSocket::setKeepAlive Manipulate the linger option
 *			for the socket. When enabled, close will not return
 *			until all queued messages for the socket have been
 *			successfully sent or the linger timeout has been reached.
 * \param linger Enable or disable lingering for the socket
 *			(defaults to true).
 */
void BasicSocket::setLinger(
		const bool linger) {
	struct linger lingerOption;
	lingerOption.l_onoff = linger;
	return setOption(LINGER, lingerOption);
}

/*!
 * \brief BasicSocket::setLingerSeconds Manipulate the linger option
 *			for the socket. Set the linger timeout. If greater
 *			than zero, this option also enables the linger option
 *			for the socket.
 * \param seconds Timeout to set.
 */
void BasicSocket::setLingerSeconds(
		const int seconds) {
	struct linger lingerOption;
	lingerOption.l_linger = std::max(seconds, 0);
	lingerOption.l_onoff = seconds > 0;
	return setOption(LINGER, lingerOption);
}

/*!
 * \brief BasicSocket::setBlocking Manipulate the blocking option of
 *			the socket.
 * \param blocking Enable or disable blocking operation (defaults to true).
 */
void BasicSocket::setBlocking(
		const bool blocking) {
	setOption(NONBLOCKING, !blocking);
}

/*!
 * \brief BasicSocket::getBlocking Fetch the blocking option of the
 *			socket.
 * \return Value showing if socket is blocking.
 */
bool BasicSocket::getBlocking() const {
	return !getOption(NONBLOCKING);
}

/*!
 * \brief BasicSocket::getType Get the type of the socket (stream or
 *			datagram).
 * \return Value representing the socket type.
 */
Socket::SocketType BasicSocket::getType() const {
	return static_cast<SocketType>(getOption(TYPE));
}

/*!
 * \brief BasicSocket::close Close the socket.
 */
void BasicSocket::close() {
	if (mSockfd >= 0) {
		if (::close(mSockfd) < 0) {
#ifdef SOCKET_DEBUG
			perror("close");
#endif
		}
	}
	mSockfd = -1;
}

/*!
 * \brief BasicSocket::open If you are using this, there is likely
 *			a better class to use.
 * \param type
 */
void BasicSocket::open(const SocketType type) {
	if ((mSockfd = socket(AF_INET, type, 0)) < 0) {
		throw SocketCreateError(errno);
	}
}

/*!
 * \brief BasicSocket::getRemoteIP Fetch a string representation
 *			of the connected remote address.
 * \return String with remote IP.
 */
BasicSocket::Address BasicSocket::getRemoteIP() const {
	char ipStr[INET_ADDRSTRLEN];
	const auto remoteAddr = getRemoteAddr();
	if (inet_ntop(remoteAddr.sin_family, &remoteAddr.sin_addr,
				  ipStr, sizeof (ipStr)) == nullptr) {
		throw NtopError(errno);
	}
	return Address(ipStr);
}

/*!
 * \brief BasicSocket::getRemotePort Fetch the port used on the
 *			connected remote address.
 * \return Port of the remote.
 */
BasicSocket::Port BasicSocket::getRemotePort() const {
	return ntohs(getRemoteAddr().sin_port);
}

/*!
 * \brief BasicSocket::getLocalPort Fetches the bound local port
 *			of the socket.
 * \return Bound local port.
 */
Socket::Port BasicSocket::getLocalPort() const {
	return ntohs(getLocalAddr().sin_port);
}

/*!
 * \brief BasicSocket::toSockaddr Convert IP address and port into
 *			a sockaddr struct.
 * \param addr IP to convert.
 * \param port Port to convert.
 * \return Struct representation of a socket address.
 */
sockaddr_in BasicSocket::toSockaddr(
		const Address &addr,
		const Port port) {
	sockaddr_in saddr;
	saddr.sin_family = AF_INET;
	saddr.sin_port = htons(port);
	if (addr.empty()) {
		saddr.sin_addr.s_addr = INADDR_ANY;
	}
	else {
		if (inet_pton(saddr.sin_family, addr.c_str(), &saddr.sin_addr) < 0) {
			throw PtonError(errno);
		}
	}
	return saddr;
}

/*!
 * \brief BasicSocket::toSockaddr Convert IP address and port into
 *			a sockaddr struct.
 * \param info Host info to convert.
 * \return Struct representation of a socket address.
 */
sockaddr_in BasicSocket::toSockaddr(const HostInfo& info) {
	return toSockaddr(info.address, info.port);
}

/*!
 * \brief BasicSocket::fromSockaddr Convert from sockaddr struct
 *			to string containing IP and port.
 * \param addr Struct representation of socket address.
 * \return String representation of socket address.
 */
BasicSocket::HostInfo BasicSocket::fromSockaddr(
		const struct sockaddr_in &addr) {
	char ipAddr[INET6_ADDRSTRLEN];
	if (inet_ntop(addr.sin_family, &addr.sin_addr, ipAddr, sizeof (ipAddr)) == nullptr) {
		throw NtopError(errno);
	}
	return {std::string(ipAddr), ntohs(addr.sin_port)};
}

/*!
 * \brief Socket::recv Receive data from socket.
 * \return Vector containing the received data.
 */
std::vector<char> Socket::recv(const MessageOption option) {
	auto bytesRead = ::recv(mSockfd, recvBuffer.data(), recvBuffer.size(), option);
	if (bytesRead > 0) {
		auto first = recvBuffer.cbegin();
		auto last = recvBuffer.cbegin()+bytesRead;
		return std::vector<char>(first, last);
	}
	else if (bytesRead == 0) {
		try {
			// Zero length datagrams are allowed in UDP
			if (getType() == DATAGRAM || recvBuffer.size() == 0) {
				return std::vector<char>();
			}
		} catch (GetSockOptError&) {}
		close();
		throw DisconnectedError();
	}
	else {
		if (errno == EBADF) {
			throw DisconnectedError();
		}
		else if (!getBlocking() && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			return std::vector<char>();
		}
		throw SocketRecvError(errno);
	}
}

/*!
 * \brief Socket::send Transmit data on socket.
 * \param data Data to transmit.
 * \param nBytes Number of bytes to transmit from start of data vector.
 */
void Socket::send(
		const std::vector<char>& data,
		const size_t nBytes,
		const MessageOption option) {
	if (nBytes > data.size()) {
		throw ArgumentError("Bytes to send exceeds data size");
	}

	auto bytesSent = ::send(mSockfd, data.data(), nBytes, option);

	if (bytesSent < 0) {
		if (errno == EBADF) {
			throw DisconnectedError();
		}
		throw SocketSendError(errno);
	}
}

/*!
 * \brief Socket::send Transmit data on socket. Transmits the entire
 *			data buffer passed as parameter.
 * \param data Data to transmit.
 */
void Socket::send(const std::vector<char> &data, const MessageOption option) {
	return send(data, data.size(), option);
}

/*!
 * \brief BasicSocket::haveInput Checks if there is input available on the
 *			socket.
 * \param timeout Time to wait (seconds) before returning no data present.
 * \return True if data is present, false if not.
 */
bool BasicSocket::haveInput(
		const double timeout) {
	int status;
	fd_set fds;
	struct timeval tv;
	FD_ZERO(&fds);
	FD_SET(mSockfd, &fds);
	tv.tv_sec  = static_cast<long>(timeout);
	tv.tv_usec = static_cast<long>((timeout - tv.tv_sec) * 1000000);

	while (true) {
		if (!(status = select(mSockfd + 1, &fds, nullptr, nullptr, &tv))) {
			return false;
		}
		else if (status > 0 && FD_ISSET(mSockfd, &fds)) {
			return true;
		}
		else if (status > 0) {
			throw RuntimeError("???", errno);
		}
		else if (errno != EINTR) {
			throw SocketSelectError(errno);
		}
	}
}

/*!
 * \brief BasicSocket::preCloseFlush Flushes the socket data buffer
 *			in preparation for closing. Usually not necessary.
 * \param timeout Time to spend flushing incoming data.
 * \return 0 if successful, -1 otherwise.
 */
int BasicSocket::preCloseFlush(
		const std::chrono::system_clock::duration& timeout) {
	const auto start = std::chrono::system_clock::now();
	char discard[99];
	if (shutdown(mSockfd, SHUT_WR) != -1) {
		while (std::chrono::system_clock::now() < start + timeout) {
			while (haveInput(0.01)) {
				if (!::read(mSockfd, discard, sizeof (discard))) {
					return 0;
				}
			}
		}
	}
	return -1;
}
