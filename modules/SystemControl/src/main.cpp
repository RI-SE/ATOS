#include "systemcontrol.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono;

/* decl */
static std::shared_ptr<SystemControl> sc;
static void systemcontrol_task(TimeType * GPSTime, LOG_LEVEL logLevel);

int main(int argc, char** argv){
	TimeType *GPSTime;
	GPSTime = (TimeType*) mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	systemcontrol_task(GPSTime,LOG_LEVEL_INFO);
	return 0;
}

/*
Initializes Logs, Shared memory and mode, then runs the main loop
*/
void systemcontrol_task(TimeType * GPSTime, LOG_LEVEL logLevel){
	// Initialize ROS node
	rclcpp::init(0,nullptr);
	sc = std::make_shared<SystemControl>();
	sc->initialize(logLevel);
	
	// Set up signal handlers
	auto f = [](int signo) -> void {sc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR){
		util_error("Unable to initialize signal handler");
	}

	rclcpp::executors::SingleThreadedExecutor executor;

	while (rclcpp::ok() && !sc->shouldExit()){
		sc->receiveUserCommand(GPSTime, logLevel);
		sc->processUserCommand(GPSTime, logLevel);
		sc->sendUnsolicitedData(GPSTime, logLevel);
		

		// If SystemControl is not in work state and clientResult < 0 then wait at most QUEUE_EMPTY_POLL_PERIOD for a msg
		// else keep running at full speed.
		const int64_t sleeptime=sc->isWorking() ? 0 : sc->getQueueEmptyPollPeriod();
		executor.spin_node_once(sc,duration<int64_t,nanoseconds::period>(sleeptime));
	}
	ReadWriteAccess_t dataDictOperationResult = DataDictionaryDestructor();
	if (dataDictOperationResult != WRITE_OK && dataDictOperationResult != READ_WRITE_OK) {
		util_error("Unable to clear shared memory space");
	}
	RCLCPP_INFO(sc->get_logger(), "System control exiting");
	rclcpp::shutdown();
}
