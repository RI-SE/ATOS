#include "systemcontrol.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono;

/* decl */
static std::shared_ptr<SystemControl> sc;
static void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);

int main(int argc, char** argv){
	TimeType *GPSTime;
	GSDType *GSD;
	GPSTime = (TimeType*) mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	GSD = (GSDType*) mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	systemcontrol_task(GPSTime,GSD,LOG_LEVEL_INFO);
	return 0;
}

/*
Initializes Logs, Shared memory and mode, then runs the main loop
*/
void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel){
	// Initialize ROS node
	rclcpp::init(0,nullptr);
	sc = std::make_shared<SystemControl>();
	sc->initialize(GPSTime, GSD, logLevel);
	
	// Set up signal handlers
	auto f = [](int signo) -> void {sc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR){
		util_error("Unable to initialize signal handler");
	}

	rclcpp::executors::SingleThreadedExecutor executor;

	while (rclcpp::ok() && !sc->shouldExit()){
		sc->receiveUserCommand(GPSTime, GSD, logLevel);
		sc->processUserCommand(GPSTime, GSD, logLevel);
		sc->sendUnsolicitedData(GPSTime, GSD, logLevel);
		

		// If SystemControl is not in work state and clientResult < 0 then wait at most QUEUE_EMPTY_POLL_PERIOD for a msg
		// else keep running at full speed.
		int64_t sleeptime=sc->isWorking() ? 0 : QUEUE_EMPTY_POLL_PERIOD;
		executor.spin_node_once(sc,duration<int64_t,nanoseconds::period>(sleeptime));
	}
	ReadWriteAccess_t dataDictOperationResult = DataDictionaryDestructor();
	if (dataDictOperationResult != WRITE_OK && dataDictOperationResult != READ_WRITE_OK) {
		util_error("Unable to clear shared memory space");
	}
	LogMessage(LOG_LEVEL_INFO, "System control exiting");
	rclcpp::shutdown();
}
