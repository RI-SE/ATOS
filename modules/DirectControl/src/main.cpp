#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "directcontrol.hpp"

using namespace std::chrono;

static std::shared_ptr<DirectControl> dc;

int main() {
	rclcpp::init(0,nullptr);
	dc = std::make_shared<DirectControl>();
	// Set up signal handlers
	auto f = [](int signo) -> void {dc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR){
		util_error("Unable to initialize signal handler");
	}

	rclcpp::executors::SingleThreadedExecutor executor;

	dc->initializeModule(LOG_LEVEL_DEBUG);
	dc->startThreads();
	while (rclcpp::ok() && !dc->shouldExit()){
		executor.spin_node_once(dc,duration<int64_t,nanoseconds::period>(100000000));
	}
	dc->joinThreads();
	rclcpp::shutdown();
	return 0;
}
