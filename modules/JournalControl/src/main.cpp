#include "logging.h"
#include "util.h"
#include "rclcpp/rclcpp.hpp"
#include "journalcontrol.hpp"

int main(int argc, char **argv) {
	const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;
	rclcpp::init(argc,argv);
	auto node = std::make_shared<JournalControl>(logLevel);
	rclcpp::spin(node);
	return 0;
}