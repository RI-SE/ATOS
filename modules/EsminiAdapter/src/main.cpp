#include "esminiadapter.hpp"
#include "util.h"

using namespace std::chrono;

static std::shared_ptr<EsminiAdapter> esminiAdapter;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	esminiAdapter = EsminiAdapter::instance(UtilGetOscDirectoryPath() + "ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.xosc");
	esminiAdapter->initializeModule(LOG_LEVEL_DEBUG);
	rclcpp::spin(esminiAdapter);
	rclcpp::shutdown();
	return 0;
}
