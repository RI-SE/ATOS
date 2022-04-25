#include "logging.h"
#include "util.h"
#include "rclcpp/rclcpp.hpp"
#include "journalcontrol.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto node = std::make_shared<JournalControl>();
	rclcpp::spin(node);
	return 0;
}