#include "monr_relay.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto sm = std::make_shared<monr_relay::MonrRelayModule>();
	rclcpp::spin(sm);
	rclcpp::shutdown();
	return 0;
}
