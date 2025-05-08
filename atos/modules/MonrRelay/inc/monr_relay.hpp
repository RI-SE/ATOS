/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <thread>

#include <std_srvs/srv/set_bool.hpp>

#include "module.hpp"
#include "roschannels/monitorchannel.hpp"
#include "server.hpp"

namespace monr_relay {

/*!
 * \brief MonrRelay
 */
class MonrRelayModule : public Module {
public:
	static inline std::string const moduleName = "monr_relay";
	MonrRelayModule();

private:
	void relayMonitorMessage(ROSChannels::Monitor::message_type::SharedPtr message);
	ROSChannels::Monitor::SubAll monrSub;
	UDPServer udp_server;
	BasicSocket::HostInfo remote_host{};
};

} // namespace monr_relay
