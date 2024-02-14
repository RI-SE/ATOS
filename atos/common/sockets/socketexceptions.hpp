/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once
#include <stdexcept>
#include <cstring>
#include <arpa/inet.h>
#include <cstdio>
#include <iostream>

namespace SocketErrors {

class StdErrorPrinter {
protected:
	inline void perror(const char* s) const {
#ifdef SOCKET_DEBUG
		::perror(s);
#endif
	}
};

class ArgumentError : public std::invalid_argument, public StdErrorPrinter {
public:
	ArgumentError() : std::invalid_argument("") {}
	ArgumentError(const std::string& msg)
		: std::invalid_argument(msg) {}
	ArgumentError(const std::string& msg, const int errorNo)
		: std::invalid_argument(msg + " (" + strerror(errorNo) + ")") {}
	virtual ~ArgumentError() {}
};
class NtopError final : public ArgumentError {
public:
	NtopError(const int errorNo) : ArgumentError("Failed to stringify IP address", errorNo) { perror("inet_ntop"); }
};
class PtonError final : public ArgumentError {
public:
	PtonError(const int errorNo) : ArgumentError("Failed to parse IP string", errorNo) { perror("inet_pton"); }
};

class RuntimeError : public std::runtime_error, public StdErrorPrinter {
public:
	RuntimeError() : std::runtime_error("") {}
	RuntimeError(const std::string& msg)
		: std::runtime_error(msg) {}
	RuntimeError(const std::string& msg, const int errorNo)
		: std::runtime_error(msg + " (" + strerror(errorNo) + ")") {
		mErrorNo = errorNo;
	}
	virtual ~RuntimeError() {}

	int getErrorNo() const { return mErrorNo; }
private:
	int mErrorNo = 0;
};

class DisconnectedError : public RuntimeError {
public:
	DisconnectedError() {}
};

/*!
 * \brief The ConfigurationError class represents a category of errors relating to configuration
 *			of sockets.
 */
class ConfigurationError : public RuntimeError {
public:
	ConfigurationError(const std::string& msg, const int errorNo) : RuntimeError(msg, errorNo) {}
};
class GetSockOptError final : public ConfigurationError {
public:
	GetSockOptError(const std::string& optionName, const int errorNo)
		: ConfigurationError("Failed to get socket option " + optionName, errorNo) { perror("getsockopt"); }
};
class SetSockOptError final : public ConfigurationError {
public:
	SetSockOptError(const std::string& optionName, const int errorNo)
		: ConfigurationError("Failed to set socket option " + optionName, errorNo) { perror("setsockopt"); }
};
class FileControlError : public ConfigurationError {
public:
	FileControlError(const std::string& msg, const std::string& optionName, const int errorNo) : ConfigurationError(msg + " for option " + optionName, errorNo) { perror("fcntl"); }
};
class FileControlGetError final : public FileControlError {
public:
	FileControlGetError(const std::string& optionName, const int errorNo) : FileControlError("Failed to get file status flags", optionName, errorNo) {}
};
class FileControlSetError final : public FileControlError {
public:
	FileControlSetError(const std::string& optionName, const int errorNo) : FileControlError("Failed to set file status flags", optionName, errorNo) {}
};
class SocketGetPeerNameError final : public ConfigurationError {
public:
	SocketGetPeerNameError(const int errorNo) : ConfigurationError("Failed to get remote socket address", errorNo) { perror("getpeername"); }
};
class SocketGetSockNameError final : public ConfigurationError {
public:
	SocketGetSockNameError(const int errorNo) : ConfigurationError("Failed to get bound local socket address", errorNo) { perror("getsockname"); }
};

/*!
 * \brief The SocketOperationError class represents a category of errors relating to operations
 *			on sockets.
 */
class SocketOperationError : public RuntimeError {
public:
	SocketOperationError(const std::string& opName, const int errorNo)
		: RuntimeError("Failed at " + opName + " call", errorNo) { perror(opName.c_str()); }
};
class SocketCreateError final : public SocketOperationError {
public:
	SocketCreateError(const int errorNo) : SocketOperationError("socket", errorNo) {}
};
class SocketRecvError final : public SocketOperationError {
public:
	SocketRecvError(const int errorNo) : SocketOperationError("recv", errorNo) {}
};
class SocketRecvFromError final : public SocketOperationError {
public:
	SocketRecvFromError(const int errorNo) : SocketOperationError("recvfrom", errorNo) {}
};
class SocketSendError final : public SocketOperationError {
public:
	SocketSendError(const int errorNo) : SocketOperationError("send", errorNo) {}
};
class SocketSendToError final : public SocketOperationError {
public:
	SocketSendToError(const int errorNo) : SocketOperationError("sendto", errorNo) {}
};
class SocketSelectError final : public SocketOperationError {
public:
	SocketSelectError(const int errorNo) : SocketOperationError("select", errorNo) {}
};
class SocketConnectError final : public SocketOperationError {
public:
	SocketConnectError(const int errorNo) : SocketOperationError("connect", errorNo) {}
};
class SocketDuplicateError final : public SocketOperationError {
public:
	SocketDuplicateError(const int errorNo) : SocketOperationError("dup", errorNo) {}
};
class SocketBindError final : public SocketOperationError {
public:
	SocketBindError(const int errorNo) : SocketOperationError("bind", errorNo) {}
};
class SocketListenError final : public SocketOperationError {
public:
	SocketListenError(const int errorNo) : SocketOperationError("listen", errorNo) {}
};
class SocketAcceptError final : public SocketOperationError {
public:
	SocketAcceptError(const int errorNo) : SocketOperationError("accept", errorNo) {}
};

}
