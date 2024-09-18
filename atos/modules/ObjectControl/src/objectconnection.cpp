/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectconnection.hpp"

void ObjectConnection::connect(
		std::shared_future<void> stopRequest,
		const std::chrono::milliseconds retryPeriod) {
	try {
		this->cmd.connect(stopRequest, retryPeriod);
		this->mntr.connect(stopRequest, retryPeriod);
	} catch (std::runtime_error& e) {
		this->disconnect();
		throw std::runtime_error(std::string("Failed to establish ISO 22133 connection. Reason: \n") + e.what());
	}

	return;
}

bool ObjectConnection::isConnected() const {
	if (!isValid()) {
		return false;
	}
	pollfd fds[2];
	fds[0].fd = mntr.socket;
	fds[0].events = POLLIN | POLLOUT;
	fds[1].fd = cmd.socket;
	fds[1].events = POLLIN | POLLOUT;
	return poll(fds, 2, 0) >= 0;
}

bool ObjectConnection::isValid() const {
	return this->cmd.isValid() && this->mntr.isValid();
}

void ObjectConnection::disconnect() {
	this->cmd.disconnect();
	this->mntr.disconnect();
	close(this->interruptionPipeFds[0]);
	close(this->interruptionPipeFds[1]);
}

ISOMessageID ObjectConnection::pendingMessageType(bool awaitNext) {
	if (awaitNext) {
		if (!isValid()) {
			throw std::invalid_argument("Attempted to check pending message type for unconnected object");
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(interruptionPipeFds[0], &fds);
		FD_SET(mntr.socket, &fds);
		FD_SET(cmd.socket, &fds);
		auto result = select(std::max({mntr.socket,cmd.socket,interruptionPipeFds[0]})+1,
							 &fds, nullptr, nullptr, nullptr);
		if (result < 0) {
			throw std::runtime_error(std::string("Failed socket operation (select: ") + strerror(errno) + ")"); // TODO clearer
		}
		else if (!isValid()) {
			throw std::invalid_argument("Connection invalidated during select call");
		}
		else if (FD_ISSET(interruptionPipeFds[0], &fds)){
			close(interruptionPipeFds[0]);
			throw std::range_error("Select call was interrupted");
		}
		else if (FD_ISSET(mntr.socket, &fds)) {
			return this->mntr.pendingMessageType();
		}
		else if (FD_ISSET(cmd.socket, &fds)) {
			return this->cmd.pendingMessageType();
		}
		throw std::logic_error("Call to select returned unexpectedly: " + std::to_string(result));
	}
	else {
		auto retval = this->mntr.pendingMessageType();
		return retval != MESSAGE_ID_INVALID ? retval : this->cmd.pendingMessageType();
	}
}