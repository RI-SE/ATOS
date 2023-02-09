#include "systemcontrol.hpp"
#include "rclcpp/executor.hpp"

using namespace std::chrono;

/* decl */
static std::shared_ptr<SystemControl> sc;
static void systemcontrol_task(int argc, char** argv);

int main(int argc, char** argv){
	systemcontrol_task(argc, argv);
	return 0;
}

/*
Initializes Logs, Shared memory and mode, then runs the main loop
*/
void systemcontrol_task(int argc, char** argv){
	// Initialize ROS node
	rclcpp::init(argc, argv);
	sc = std::make_shared<SystemControl>();
	sc->initialize();
	
	// Set up signal handlers
	auto f = [](int signo) -> void {sc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR){
		util_error("Unable to initialize signal handler");
	}

	rclcpp::executors::SingleThreadedExecutor executor;

	while (rclcpp::ok() && !sc->shouldExit()){
		sc->receiveUserCommand();
		sc->processUserCommand();
		sc->sendUnsolicitedData();
		sc->pollModuleStatus();

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
