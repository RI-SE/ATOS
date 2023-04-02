#include "rclcpp/rclcpp.hpp"
#include "dronecontrol.hpp"


int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	auto sm = std::make_shared<DroneControl>();
	rclcpp::spin(sm);
	rclcpp::shutdown();
	return 0;
}
