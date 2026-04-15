#include "CotReceiver.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<CotReceiver> cot_reciver_node = std::make_shared<CotReceiver>();
	rclcpp::spin(cot_reciver_node);
	rclcpp::shutdown();
	return 0;
}
