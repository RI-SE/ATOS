
#include "mqttbridge.hpp"

static std::shared_ptr<MqttBridge> mqttBridge;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	mqttBridge = MqttBridge::instance();
	mqttBridge->initializeModule();
	rclcpp::spin(mqttBridge);
	rclcpp::shutdown();
}
