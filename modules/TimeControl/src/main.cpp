/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "timecontrol.hpp"
#include "rclcpp/executor.hpp"
#include <sys/mman.h> 

using namespace std::chrono;

static std::shared_ptr<TimeControl> tc;
static void timecontrol_task(TimeType * GPSTime, GSDType * GSD);

int main(int argc, char** argv){
	TimeType *GPSTime;
	GSDType *GSD;
	GPSTime = reinterpret_cast<TimeType*>(mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
	GSD = reinterpret_cast<GSDType*>(mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
	timecontrol_task(GPSTime,GSD);
	return 0;
}

/*!
 * \brief Initializes the ROS bus and TimeControl. 
 *	Runs the main task of TimeControl until shut off.
 */
void timecontrol_task(TimeType * GPSTime, GSDType * GSD){
	// Initialize ROS node
	rclcpp::init(0,nullptr);
	tc = std::make_shared<TimeControl>();
	tc->initialize(GPSTime,GSD);

	// Set up signal handlers
	auto f = [](int signo) -> void {tc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR){
		util_error("Unable to initialize signal handler");
	}

	// Create an executor for executing the callback functions of the node when messages arrive
	rclcpp::executors::SingleThreadedExecutor executor;
	
	while (rclcpp::ok() && !tc->shouldExit()){
		//Do some work
		tc->calibrateTime(GPSTime, GSD);
		
		// spin_node_once() adds node to executor, spins and then removes node from executor.
		// If message exists in queue, it instantly processed callback, else sleeps at most POLL_SLEEP_TIME for a message to arrive.
		executor.spin_node_once(tc,duration<int64_t,nanoseconds::period>(tc->getQueueEmptyPollPeriod())); 
	}
	RCLCPP_INFO(tc->get_logger(),"Time control exiting");
	rclcpp::shutdown();
}

