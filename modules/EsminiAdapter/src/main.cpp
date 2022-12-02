#include "esminiadapter.hpp"

using namespace std::chrono;

static std::shared_ptr<EsminiAdapter> esminiAdapter;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	esminiAdapter = EsminiAdapter::instance();
	esminiAdapter->initializeModule(LOG_LEVEL_DEBUG);
	rclcpp::spin(esminiAdapter);
	rclcpp::shutdown();
	return 0;
}
