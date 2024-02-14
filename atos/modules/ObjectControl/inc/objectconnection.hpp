/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <future>
#include <chrono>
#include <netinet/in.h>
#include "iso22133.h"
#include "loggable.hpp"
#include "channel.hpp"

/*!
 * \brief The ObjectConnection class holds network connection data for
 *			a single object, i.e. the two channels for command and
 *			safety data.
 */
class ObjectConnection : public Loggable {
public:
	Channel cmd;
	Channel mntr;

	ObjectConnection(rclcpp::Logger log, int id)
		: Loggable(log),
			cmd(SOCK_STREAM, log, id),
			mntr(SOCK_DGRAM, log, id) {
			pipe(interruptionPipeFds);
		}

	bool isValid() const;
	bool isConnected() const;
	void connect(std::shared_future<void> stopRequest,
				 const std::chrono::milliseconds retryPeriod);
	void disconnect();
	ISOMessageID pendingMessageType(bool awaitNext = false);
	void interruptSocket() {
		int i = 1;
		write(interruptionPipeFds[1], &i, sizeof(i));
		close(interruptionPipeFds[1]);
	}
private:
	int interruptionPipeFds[2];
};