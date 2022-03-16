#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "directcontrol.hpp"

using namespace std::chrono;

static std::shared_ptr<DirectControl> dc;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	dc = std::make_shared<DirectControl>();
	//rclcpp::executors::SingleThreadedExecutor executor;
	dc->initializeModule(LOG_LEVEL_DEBUG);
	dc->startThreads();
	rclcpp::spin(dc);
	/*while (rclcpp::ok() && !dc->shouldExit()){
		executor.spin_node_once(dc,duration<int64_t,nanoseconds::period>(100000000));
	}*/
	dc->joinThreads();
	rclcpp::shutdown();
	return 0;
}
