#include "systemcontrol.hpp"
#include "rclcpp/executor.hpp"

/* decl */
static std::shared_ptr<SystemControl> sc;
void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);

int main(int argc, char** argv){
	TimeType *GPSTime;
	GSDType *GSD;
	GPSTime = mmap(NULL, sizeof *GPSTime, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	GSD = mmap(NULL, sizeof *GSD, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	systemcontrol_task(GPSTime,GSD,LOG_LEVEL_INFO);
	return 0;
}

void signalHandlerWrapper(std::shared_ptr<SystemControl> sc, int signo)
{
  sc->signalHandler(signo);
}

/*
Initializes Logs, Shared memory and mode, then runs the main loop
*/
void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel){
	// Initialize ROS node
	rclcpp::init(0,nullptr);
	sc = std::make_shared<SystemControl>("SystemControl");

	LogInit("SystemControl", logLevel);
	LogMessage(LOG_LEVEL_INFO, "System control task running with PID: %i", getpid());

	// Set up signal handlers
	auto f = [](int signo) -> void {sc->signalHandler(signo);};
	if (std::signal(SIGINT,f) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	DataDictionaryGetRVSSConfigU32(&sc->RVSSConfigU32);
	LogMessage(LOG_LEVEL_INFO, "RVSSConfigU32 = %d", sc->RVSSConfigU32);

	DataDictionaryGetRVSSRateU8(&sc->RVSSRateU8);
	LogMessage(LOG_LEVEL_INFO, "Real-time variable subscription service rate set to %u Hz", sc->RVSSRateU8);

	if (sc->ModeU8 == 0) {

	}
	else if (sc->ModeU8 == 1) {
		sc->SessionData.SessionIdU32 = 0;
		sc->SessionData.UserIdU32 = 0;
		sc->SessionData.UserTypeU8 = 0;

		sc->PollRateU64 = SYSTEM_CONTROL_SERVICE_POLL_TIME_MS;
		sc->CurrentTimeU64 =
			(uint64_t) sc->CurrentTimeStruct.tv_sec * 1000 + (uint64_t) sc->CurrentTimeStruct.tv_usec / 1000;
		sc->OldTimeU64 = sc->CurrentTimeU64;

	}
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(sc);

	while (rclcpp::ok() && !sc->iExit){
		sc->mainTask(GPSTime, GSD, logLevel);
		sc->handleCommand(GPSTime, GSD, logLevel);
		sc->sendTimeMessages(GPSTime, GSD, logLevel);
		
		executor.spin_node_once(sc);

		nanosleep((const struct timespec[]){{0, 5000L}}, NULL); // temporary fix, sleep 5000 ns
	}
}

