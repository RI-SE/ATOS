#include "objectcontrol.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto obc = std::make_shared<ObjectControl>();
	rclcpp::spin(obc);
	rclcpp::shutdown();

	return 0;
}
