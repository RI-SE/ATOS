#include "systemcontrol.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono;

/* decl */
std::shared_ptr<SystemControl> sc;
void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);

int main(int argc, char** argv){
	TimeType *GPSTime;
	GSDType *GSD;
	GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	systemcontrol_task(GPSTime,GSD,LOG_LEVEL_INFO);
	return 0;
}

/*
Initializes Logs, Shared memory and mode, then runs the main loop
*/
void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel){
	// Initialize ROS node
	rclcpp::init(0,nullptr);
	sc = std::make_shared<SystemControl>(MODULE_NAME);
	sc->initialize(GPSTime, GSD, logLevel);
	
	// Set up signal handlers
	auto f = [](int signo) -> void {sc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	rclcpp::executors::SingleThreadedExecutor executor;

	while (rclcpp::ok() && !sc->iExit){
		sc->mainTask(GPSTime, GSD, logLevel);
		sc->handleCommand(GPSTime, GSD, logLevel);
		sc->sendTimeMessages(GPSTime, GSD, logLevel);
		

		// If SystemControl is not in work state and clientResult < 0 then wait at most POLL_SLEEP_TIME for a msg
		// else keep running at full speed.
		int64_t sleeptime=(sc->SystemControlState != SERVER_STATE_INWORK && (sc->ClientResult < 0)) ? QUEUE_EMPTY_POLL_PERIOD : 0;
		executor.spin_node_once(sc,duration<int64_t,nanoseconds::period>(sleeptime));
	}
	LogMessage(LOG_LEVEL_INFO, "System control exiting");
	rclcpp::shutdown();
}
