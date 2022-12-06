#include "esminiadapter.hpp"
#include "esmini/esminiLib.hpp"
#include "esmini/esminiRMLib.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "trajectory.hpp"
#include "datadictionary.h"

using namespace ROSChannels;

std::shared_ptr<EsminiAdapter> EsminiAdapter::me = NULL;
std::unordered_map<int,int> EsminiAdapter::objectIdToIndex = std::unordered_map<int, int>();

/*!
 * \brief Creates an instance and initialize esmini if none exists, otherwise returns the existing instance.
 * \return the sole EsminiAdapter instance.
 */
std::shared_ptr<EsminiAdapter> EsminiAdapter::instance() {
	if (me == nullptr) {
		me = std::shared_ptr<EsminiAdapter>(new EsminiAdapter());
		me->InitializeEsmini(oscFilePath);
	}
	return me;
}

EsminiAdapter::EsminiAdapter() : Module(moduleName) {
	// Get the file path of xosc file
	try{
		char path[MAX_FILE_PATH];
		UtilGetOscDirectoryPath(path, MAX_FILE_PATH);
		oscFilePath = std::string(path) + oscFileName;
	}
	catch (std::exception& e) {
		throw std::runtime_error("EsminiAdapter: Could not read xosc file path");
	}
}

//! Message queue callbacks

void EsminiAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
	SE_Close();
}

void EsminiAdapter::onAllClearMessage(const AllClear::message_type::SharedPtr) {}

/*!
 * \brief Callback to be executed by esmini when story board state changes.
 * \param name Name of the StoryBoardElement whose state has changed.
 * \param type Possible values: STORY = 1, ACT = 2, MANEUVER_GROUP = 3, MANEUVER = 4, EVENT = 5, ACTION = 6, UNDEFINED_ELEMENT_TYPE = 0.
 * \param state new state, possible values: STANDBY = 1, RUNNING = 2, COMPLETE = 3, UNDEFINED_ELEMENT_STATE = 0.
 */
void EsminiAdapter::onEsminiStoryBoardStateChange(const char* name, int type, int state){
	//Placeholder, TODO: Implement
	RCLCPP_INFO(me->get_logger(), "Esmini Storyboard State Change Name: %s, Type: %d, State: %d", name, type, state);
}

/*!
 * \brief Callback to be executed by esmini when a condition is triggered.
 * \param name Name of the condition that was triggered.
 * \param timestamp Timestamp when the condition triggered.
 */
void EsminiAdapter::onEsminiConditionTriggered(const char* name, double timestamp){
	//Placeholder, TODO: Implement
	RCLCPP_INFO(me->get_logger(), "Esmini Condition Trigger Name %s", name);
}

/*!
 * \brief Utility function to convert a ROS MONR message to Esmini representation
 *			and report the object position to Esmini
 * \param monr ROS Monitor message of an object
 * \param id The object ID to which the monr belongs
 */

void EsminiAdapter::reportObjectPosition(const Monitor::message_type::SharedPtr monr, uint32_t id){
	// Conversions from ROS to Esmini
	auto ori = monr->pose.pose.orientation;
	auto quat = tf2::Quaternion(ori.x, ori.y, ori.z, ori.w);
	tf2::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	auto pos = monr->pose.pose.position;

	// Reporting to Esmini
	int timestamp = 0; // Not really used according to esmini documentation
	SE_ReportObjectPos(id, timestamp, pos.x, pos.y, pos.z, yaw, pitch, roll);
}


void EsminiAdapter::onMonitorMessage(const Monitor::message_type::SharedPtr monr, uint32_t id) {
	if (me->objectIdToIndex.find(id) != me->objectIdToIndex.end()){
		reportObjectPosition(monr, id); // Report object position to esmini
		SE_Step(); // Advance the "simulation world"-time
	}
	else{
		RCLCPP_WARN(me->get_logger(), "Received MONR message for object with ID %d, but no such object exists in the scenario", id);
	}
}

/*!
 * \brief Given a vector of object states, creates a trajectory for each vector
 * \return A vector of trajectories, one for each object in the scenario
 */

void EsminiAdapter::getTrajectories(std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states) {
}

/*!
 * \brief Dump object positions for each timestep.
 *	Inspired by ScenarioGateway::WriteStatesToFile from esmini lib.
 * \param oscFilePath Path to the xosc file
 * \param timeStep Time step to use for generating the trajectories
 * \param endTime End time of the simulation
 * \return A map of 
 */

void EsminiAdapter::getObjectStates(const std::string& oscFilePath, double timeStep, double endTime, std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states) {
	SE_Init(oscFilePath.c_str(), 0, 0, 0, 0); // Run the ScenarioEngine
	double accumTime = 0;
	while (accumTime < endTime) {
		SE_StepDT(timeStep);
		accumTime += timeStep;
		states[timeStep] = std::vector<SE_ScenarioObjectState>();
		for (int j = 0; j < SE_GetNumberOfObjects(); j++){
			SE_ScenarioObjectState* state;
			SE_GetObjectState(SE_GetId(j), state);
			states.at(SE_GetId(j)).push_back(*state); // Copy state into vector
		}
	}
	SE_Close(); // Stop ScenarioEngine
}

void EsminiAdapter::extractTrajectories(const std::string& oscFilePath, double timeStep, double endTime){
	// Get object states
	std::map<uint32_t,std::vector<SE_ScenarioObjectState>> states;
	getObjectStates(oscFilePath, timeStep, endTime, states);

	// Extract trajectories
	getTrajectories(states);

}

/*!
 * \brief Initialize the esmini simulator and perform subsequent setup tasks
 */
void EsminiAdapter::InitializeEsmini(std::string& oscFilePath){
	// Extract trajectory data for each object
	// TODO
	me->extractTrajectories(oscFilePath, 0.1, 40.0);
	
	std::string openScenarioFilePath = "/home/victor/.maestro/conf/ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.xosc"; //placeholder
	SE_Init(oscFilePath.c_str(),0,0,0,0);
	SE_Step(); // Make sure that the scenario is started
	RCLCPP_DEBUG(me->get_logger(), "Esmini initialized");

	// Inject Meastro as controller for the DefaultControlled entities
	// TODO

	// Handle triggers and story board element changes
	SE_RegisterConditionCallback(&onEsminiConditionTriggered);
	SE_RegisterStoryBoardElementStateChangeCallback(&onEsminiStoryBoardStateChange);

	// Update the map tracking Object ID -> esmini index
	for (int j = 0; j < SE_GetNumberOfObjects(); j++){
		me->objectIdToIndex[SE_GetId(j)] = j;
	}

}

/*!
 * \brief initializeModule Initializes this module by creating log,
 *			connecting to the message queue bus, setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */

int EsminiAdapter::initializeModule(const LOG_LEVEL logLevel) {
	int retval = 0;

	RCLCPP_INFO(me->get_logger(), "%s task running with PID: %d",moduleName.c_str(), getpid());
	if (me->requestDataDictInitialization()) {
		if (DataDictionaryInitObjectData() != READ_OK) {
			retval = -1;
			RCLCPP_ERROR(me->get_logger(), "Preexisting data dictionary not found");
		}
	}
	else{
		retval = -1;
		RCLCPP_ERROR(me->get_logger(), "Unable to initialize data dictionary");
	}
	return retval;
}
