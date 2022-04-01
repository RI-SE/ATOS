#include <rclcpp/rclcpp.hpp>
#include "trajectorystreamer.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto ts = std::make_shared<maestro::TrajectoryStreamer>();
	rclcpp::spin(ts);
	rclcpp::shutdown();

	return 0;
}
