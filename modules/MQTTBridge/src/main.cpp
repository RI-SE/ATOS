
#include "mqttbridge.hpp"

static std::shared_ptr<MqttBridge> mqttBridge;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	mqttBridge = MqttBridge::instance();
	auto retval = mqttBridge->initializeModule();
	if (!retval){
		rclcpp::spin(mqttBridge);
	}
	rclcpp::shutdown();
	return 0;
}
