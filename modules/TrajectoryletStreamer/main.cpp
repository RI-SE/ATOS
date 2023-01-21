#include <rclcpp/rclcpp.hpp>
#include "trajectoryletstreamer.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto ts = std::make_shared<ATOS::TrajectoryletStreamer>();
	rclcpp::spin(ts);
	rclcpp::shutdown();

	return 0;
}
