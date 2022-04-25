#include "maestrobase.hpp"


int main(int argc, char** argv)
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<MaestroBase>();
	rclcpp::spin(node);
	return 0;
}
