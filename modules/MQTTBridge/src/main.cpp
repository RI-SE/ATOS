
#include "mqttbridge.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);

	auto mb = std::make_shared<MqttBridge>();
	if(rclcpp::ok()) {
		rclcpp::spin(mb);
	}
	rclcpp::shutdown();
	return 0;
}
