#include "rclcpp/rclcpp.hpp"
#include "samplemodule.hpp"


int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	auto sm = std::make_shared<SampleModule>();
	sm->initializeModule();
	rclcpp::spin(sm);
	rclcpp::shutdown();
	return 0;
}
