#include "monr_relay.hpp"

#include "rclcpp/wait_for_message.hpp"
#include "roschannels/commandchannels.hpp"

namespace monr_relay {

MonrRelayModule::MonrRelayModule() :
  Module(moduleName),
  monrSub(*this, [this](auto&& param) { MonrRelayModule::relayMonitorMessage(std::forward<decltype(param)>(param)); }) {
	BasicSocket::Address client_host{};
	declare_parameter("client_host", "127.0.0.1");
	get_parameter("client_host", client_host);

	BasicSocket::Port client_port{};
	declare_parameter("client_port", 51234);
	get_parameter("client_port", client_port);

	remote_host = {client_host, client_port};

	std::string server_host{};
	declare_parameter("server_host", "127.0.0.1");
	get_parameter("server_host", server_host);

	std::uint16_t server_port{};
	declare_parameter("server_port", 51122);
	get_parameter("server_port", server_port);

	udp_server = UDPServer(server_host, server_port);
}

void MonrRelayModule::relayMonitorMessage(ROSChannels::Monitor::message_type::SharedPtr message) {
	std::vector<char> out;
	out.reserve(message->raw_data.size());
	std::copy(message->raw_data.begin(), message->raw_data.end(), std::back_inserter(out));
	udp_server.sendto({out, remote_host});
}

} // namespace monr_relay
