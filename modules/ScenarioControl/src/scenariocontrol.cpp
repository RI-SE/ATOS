#include <thread>

#include "scenariocontrol.hpp"


#include "xodr.h"
#include "xosc.h"

using namespace std::chrono;
using namespace maestro;
using namespace ROSChannels;
using std::placeholders::_1;
	
ScenarioControl::ScenarioControl()
	: Module(ScenarioControl::moduleName),
	initSub(*this, std::bind(&ScenarioControl::onInitMessage, this, _1)),
	armSub(*this, std::bind(&ScenarioControl::onArmMessage, this, _1)),
	startSub(*this, std::bind(&ScenarioControl::onStartMessage, this, _1)),
	abortSub(*this, std::bind(&ScenarioControl::onAbortMessage, this, _1)),
	exitSub(*this, std::bind(&ScenarioControl::onExitMessage, this, _1)),
	getStatusSub(*this, std::bind(&ScenarioControl::onGetStatusMessage, this, _1)),
	objectsConnectedSub(*this, std::bind(&ScenarioControl::onObjectsConnectedMessage, this, _1)),
	disconnectSub(*this, std::bind(&ScenarioControl::onDisconnectMessage, this, _1)),
	triggerEventSub(*this, std::bind(&ScenarioControl::onTriggerEventMessage, this, _1))
{
	initialize();
}

ScenarioControl::~ScenarioControl(){
	quit=true;
	manageTriggersThread->join();
}

int ScenarioControl::initialize()
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
	//! Start thread
	manageTriggersThread=std::make_unique<std::thread>(&ScenarioControl::manageTriggers, this);
	//! Load configuration
	/*
	xosc _xosc;
	_xosc.load("/home/victor/lane_change_simple.xosc");
	_xosc.parse();
	RCLCPP_INFO(get_logger(), "string: %s",_xosc.m_OpenSCENARIO->m_FileHeader->description.m_string.c_str());*/
	UtilGetConfDirectoryPath(configPath, sizeof(configPath));
	strcat(configPath,TRIGGER_ACTION_FILE_NAME);
	return retval;
}

void ScenarioControl::onInitMessage(const ROSChannels::Init::message_type::SharedPtr){
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
		RCLCPP_WARN(get_logger(), "Received unexpected initialize command (current state: %s)",stateToString.at(state));
	}
}
void ScenarioControl::onObjectsConnectedMessage(const ObjectsConnected::message_type::SharedPtr msg){
	static int recursionDepth = 0;
	if (state == INITIALIZED) {
		state = CONNECTED;
		RCLCPP_INFO(get_logger(), "Distributing scenario configuration");
		scenario.sendConfiguration();
	}
	else { // if not initialized, try to initialize once and then send conf to objects.
		if (recursionDepth == 0){
			recursionDepth=1;
			RCLCPP_INFO(get_logger(), "Received objects connected message when not in initialized state, trying to initialize");
			onInitMessage(msg);
			onObjectsConnectedMessage(msg);
		}
	}
	recursionDepth=0;
}


void ScenarioControl::onTriggerEventMessage(const ROSChannels::TriggerEvent::message_type::SharedPtr treo){
	if (state == RUNNING) {
		// Trigger corresponding trigger
		scenario.updateTrigger(treo->trigger_id, treo);
	}
	else {
		RCLCPP_WARN(get_logger(), "Received unexpected trigger action command (current state: %s)",stateToString.at(state));
	}
}

void ScenarioControl::onAbortMessage(const Abort::message_type::SharedPtr){
	RCLCPP_INFO(get_logger(), "Received abort command");
	if (state == RUNNING) {
		state = CONNECTED;
	}
}
void ScenarioControl::onStartMessage(const Start::message_type::SharedPtr){
	if (state == CONNECTED) {
		RCLCPP_INFO(get_logger(), "Received start message - transitioning to running state");
		state = RUNNING;
		// Update the triggers immediately on transition
		// to ensure they are always in a valid state when
		// they are checked for the first time
		updateTriggers(scenario);
		nextShmemReadTime = steady_clock::now() + shmemReadPeriod;
	}
	else{
		RCLCPP_WARN(get_logger(), "Received unexpected start command (current state: %s)",stateToString.at(state));
	}
}
void ScenarioControl::onArmMessage(const Arm::message_type::SharedPtr){
	RCLCPP_INFO(get_logger(), "Resetting scenario");
	scenario.reset();
}

void ScenarioControl::onExitMessage(const Exit::message_type::SharedPtr){
	LogMessage(LOG_LEVEL_INFO, "Received exit command");
	quit=true;
	rclcpp::shutdown();
}

void ScenarioControl::onDisconnectMessage(const Disconnect::message_type::SharedPtr){
	LogMessage(LOG_LEVEL_INFO,"Received disconnect command");
	state = UNINITIALIZED;
}


/*!
* \brief Executes triggered actions in a scenario, also resets ISO triggers and updates triggers.
* Triggers are updated and executed at different frequencies (determined by scenarioDuration and shmemReadDuration).
*/
void ScenarioControl::manageTriggers() {
	while (!quit){
		auto end_time = steady_clock::now() + scenarioCheckPeriod;
		if (state == RUNNING) {
			scenario.executeTriggeredActions();

			// Allow for retriggering on received TREO messages
			scenario.resetISOTriggers();

			auto now = steady_clock::now();
			if (now > nextShmemReadTime){
				nextShmemReadTime = getNextReadTime(now);
				updateTriggers(scenario);
			}
		}
		std::this_thread::sleep_until(end_time);
	}
}


time_point<steady_clock> ScenarioControl::getNextReadTime(time_point<steady_clock> now) {
	if (( now - nextShmemReadTime ) > shmemReadPeriod){
		nextShmemReadTime += shmemReadPeriod;
	}
	else{
		nextShmemReadTime = now + shmemReadPeriod;
	}
	return nextShmemReadTime;
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