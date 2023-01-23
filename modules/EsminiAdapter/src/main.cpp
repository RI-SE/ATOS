#include "esminiadapter.hpp"


static std::shared_ptr<EsminiAdapter> esminiAdapter;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	esminiAdapter = EsminiAdapter::instance();
	esminiAdapter->initializeModule();
	rclcpp::spin(esminiAdapter);
	rclcpp::shutdown();
	return 0;
}
