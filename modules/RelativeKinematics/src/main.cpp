#include "logging.h"
#include "util.h"
#include "RelativeKinematicsModule.hpp"

int main(int argc, char **argv) {
	const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;
	rclcpp::init(argc,argv);
	auto rk = std::make_shared<RelativeKinematicsModule>(logLevel);
	rclcpp::spin(rk);
	rclcpp::shutdown();

	return 0;
}
