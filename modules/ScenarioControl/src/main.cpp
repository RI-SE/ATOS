#include "rclcpp/rclcpp.hpp"
#include "scenariocontrol.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<maestro::ScenarioControl>();
	rclcpp::spin(node);
	return 0;
}
