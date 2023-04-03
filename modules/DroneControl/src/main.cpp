#include "rclcpp/rclcpp.hpp"
#include "dronecontrol.hpp"


int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	auto dc = std::make_shared<DroneControl>();
	rclcpp::spin(dc);
	rclcpp::shutdown();
	return 0;
}
