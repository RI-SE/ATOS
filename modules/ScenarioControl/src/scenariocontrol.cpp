#include "scenariocontrol.hpp"

ScenarioControl::ScenarioControl()
	: Module(ScenarioControl::moduleName),
	initSub(*this, std::bind(&ScenarioControl::onInitMessage, this, std::placeholders::_1)),
	armSub(*this, std::bind(&ScenarioControl::onArmMessage, this, std::placeholders::_1)),
	startSub(*this, std::bind(&ScenarioControl::onStartMessage, this, std::placeholders::_1)),
	abortSub(*this, std::bind(&ScenarioControl::onAbortMessage, this, std::placeholders::_1)),
	exitSub(*this, std::bind(&ScenarioControl::onExitMessage, this, std::placeholders::_1)),
	getStatusSub(*this, std::bind(&ScenarioControl::onGetStatusMessage, this, std::placeholders::_1)),
	objectsConnectedSub(*this, std::bind(&ScenarioControl::onObjectsConnectedMessage, this, std::placeholders::_1)),
	disconnectSub(*this, std::bind(&ScenarioControl::onDisconnectMessage, this, std::placeholders::_1)),
	triggerEventSub(*this, std::bind(&ScenarioControl::onTriggerEventMessage, this, std::placeholders::_1))
{
	initialize();
}

ScenarioControl::~ScenarioControl(){
	actionExecutorThread->join();
}

void ScenarioControl::initialize()
{
	int retval = 0;
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d", get_name(), getpid());

	if (requestDataDictInitialization()) {
		if (DataDictionaryInitObjectData() != READ_OK) {
			retval = -1;
			RCLCPP_ERROR(get_logger(), "Preexisting data dictionary not found");
		}
	}
	else{
		retval = -1;
		RCLCPP_ERROR(get_logger(), "Unable to initialize data dictionary");
	}
	JournalInit(get_name());
	actionExecutorThread=std::make_unique<std::thread>(&ScenarioControl::mainLoop, this);
}

void ScenarioControl::onInitMessage(const Empty::SharedPtr){
	if (state == UNINITIALIZED) {
		try {
			RCLCPP_INFO(get_logger(), "Initializing scenario");
			scenario.initialize(configPath);
			state = INITIALIZED;
		}
		catch (std::invalid_argument e) {
			std::string errMsg = "Invalid scenario file format: " + std::string(e.what());
			util_error(errMsg.c_str());
		}
		catch (std::ifstream::failure) {
			std::string errMsg = "Unable to open scenario file <" + std::string(configPath) + ">";
			util_error(errMsg.c_str());
		}
	}
	else{
		RCLCPP_WARN(get_logger(), "Received unexpected initialize command (current state: %u)",static_cast<unsigned char>(state));
	}
}

void ScenarioControl::onObjectsConnectedMessage(const Empty::SharedPtr){
	if (state == INITIALIZED) {
		state = CONNECTED;
		RCLCPP_INFO(get_logger(), "Distributing scenario configuration");
		scenario.sendConfiguration();
	}
	else {
		RCLCPP_WARN(get_logger(), "Received unexpected objects connected command (current state: %u)",static_cast<unsigned char>(state));
	}
}

void ScenarioControl::onTriggerEventMessage(const TriggerEvent::SharedPtr){
	if (state == RUNNING) {
		// Trigger corresponding trigger
		scenario.updateTrigger(treo.triggerID, treo);
	}
	else {
		RCLCPP_WARN(get_logger(), "Received unexpected trigger action command (current state: %u)",static_cast<unsigned char>(state));
	}
}

void ScenarioControl::onAbortMessage(const Empty::SharedPtr){
	RCLCPP_INFO(get_logger(), "Received abort command");
	if (state == RUNNING) {
		state = CONNECTED;
	}
}
void ScenarioControl::onStartMessage(const Empty::SharedPtr){
	if (state == CONNECTED) {
		RCLCPP_INFO(get_logger(), "Received start message - transitioning to running state");
		state = RUNNING;
		// Update the triggers immediately on transition
		// to ensure they are always in a valid state when
		// they are checked for the first time
		updateObjectCheckTimer(&nextSHMEMreadTime, SHMEM_READ_RATE_HZ);
		updateTriggers(scenario);
	}
	else{
		RCLCPP_WARN(get_logger(), "Received unexpected start command (current state: %u)",static_cast<unsigned char>(state));
	}
}
void ScenarioControl::onArmMessage(const Empty::SharedPtr){
	RCLCPP_INFO(get_logger(), "Resetting scenario");
	scenario.reset();
}

void ScenarioControl::onExitMessage(const Empty::SharedPtr){
	LogMessage(LOG_LEVEL_INFO, "Received exit command");
	quit=true;
	rclcpp::shutdown();
}

void ScenarioControl::onDisconnectMessage(const Empty::SharedPtr){
	LogMessage(LOG_LEVEL_INFO,"Received disconnect command");
	state = UNINITIALIZED;
}


void ScenarioControl::executeScenario(){
	while (!quit){
		usleep(10000000);
		if (state == RUNNING) {
			scenario.executeTriggeredActions();
			// Allow for retriggering on received TREO messages
			scenario.resetISOTriggers();
		
if (std::chono::steady_clock::now() > nextShmemReadTime) {
  nextShmemReadTime += shmemReadInterval;
  doStuff();
  }
			TimeSetToCurrentSystemTime(&tvTime);
			if (timercmp(&tvTime, &nextSHMEMreadTime, >)) {
				updateObjectCheckTimer(&nextSHMEMreadTime, SHMEM_READ_RATE_HZ);
				updateTriggers(scenario);
			}
		}
	}
}


/*!
 * \brief updateTriggers reads monr messages from the shared memory and passes the iformation along to scenario which handles trigger updates.
 *			with the rate parameter
 * \param Scenario scenario object keeping information about which trigger is linked to which action and the updating and parsing of the same.
 */
int ScenarioControl::updateTriggers(Scenario& scenario) {

	std::vector<uint32_t> transmitterIDs;
	uint32_t numberOfObjects;
	ObjectDataType monitorData;

	// Get number of objects present in shared memory
	if (DataDictionaryGetNumberOfObjects(&numberOfObjects) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR,
				   "Data dictionary number of objects read error - Cannot update triggers");
		return -1;
	}
	if (numberOfObjects == 0) {
		return 0;
	}

	transmitterIDs.resize(numberOfObjects, 0);
	// Get transmitter IDs for all connected objects
	if (DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size()) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR,
				   "Data dictionary transmitter ID read error - Cannot update triggers");
		return -1;
	}


	for (const uint32_t &transmitterID : transmitterIDs) {
		if (DataDictionaryGetMonitorData(transmitterID, &monitorData.MonrData) && DataDictionaryGetObjectIPByTransmitterID(transmitterID, &monitorData.ClientIP) != READ_OK) {
			LogMessage(LOG_LEVEL_ERROR,
					   "Data dictionary monitor data read error for transmitter ID %u",
					   transmitterID);
			return -1;
		}
		else {
			scenario.updateTrigger(monitorData);
		}

	}
	return 0;
}


/*!
 * \brief updateObjectCheckTimer Adds a time interval onto the specified time struct in accordance
 *			with the rate parameter
 * \param currentSHMEMReadTime Struct containing the timewhen at when SHMEM was last accessed. After this
 *			function has been executed, the struct contains the time at which the shared memory will be accessed is to be
 *			accessed next time.
 * \param SHMEMReadRate_Hz Rate at which SHMEM is read - if this parameter is 0 the value
 *			is clamped to 1 Hz
 */
void ScenarioControl::updateObjectCheckTimer(struct timeval *currentSHMEMReadTime, uint8_t SHMEMReadRate_Hz) {
	struct timeval SHMEMTimeInterval, timeDiff, currentTime;

	SHMEMReadRate_Hz = SHMEMReadRate_Hz == 0 ? 1 : SHMEMReadRate_Hz;	// Minimum frequency 1 Hz
	SHMEMTimeInterval.tv_sec = static_cast<long>(1.0 / SHMEMReadRate_Hz);
	SHMEMTimeInterval.tv_usec = static_cast<long>((1.0 / SHMEMReadRate_Hz - SHMEMTimeInterval.tv_sec) * 1000000.0);

	// If there is a large difference between the current time and the time at which time at which shared memory was updated, update based
	// on current time instead of last send time to not spam messages until caught up
	TimeSetToCurrentSystemTime(&currentTime);
	timersub(&currentTime, currentSHMEMReadTime, &timeDiff);
	if (timercmp(&timeDiff, &SHMEMTimeInterval, <)) {
		timeradd(currentSHMEMReadTime, &SHMEMTimeInterval, currentSHMEMReadTime);
	}
	else {
		timeradd(&currentTime, &SHMEMTimeInterval, currentSHMEMReadTime);
	}
}

