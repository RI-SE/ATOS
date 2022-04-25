#include "util.h"
#include "objectcontrol.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto rk = std::make_shared<ObjectControl>();
	rclcpp::spin(rk);
	rclcpp::shutdown();

	return 0;
}
