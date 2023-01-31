#include "esminiadapter.hpp"
#include "esmini/esminiLib.hpp"
#include "esmini/esminiRMLib.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

#include "trajectory.hpp"
#include "datadictionary.h"

using namespace ROSChannels;

std::shared_ptr<EsminiAdapter> EsminiAdapter::me = NULL;
std::unordered_map<int,int> EsminiAdapter::objectIdToIndex = std::unordered_map<int, int>();
std::shared_ptr<rclcpp::Client<atos_interfaces::srv::GetTestOrigin>> EsminiAdapter::testOriginClient = nullptr;

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
	char path[MAX_FILE_PATH];
	declare_parameter("open_scenario_file");
	UtilGetOscDirectoryPath(path, MAX_FILE_PATH);

	std::string result;
	auto success = get_parameter("open_scenario_file", result);
	if (!success) {
		throw std::runtime_error("Could not read parameter open_scenario_file");
	}
	oscFilePath = std::string(path) + result;
}

//! Message queue callbacks

void EsminiAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
	SE_Close();
}

void EsminiAdapter::onAllClearMessage(const AllClear::message_type::SharedPtr) {}

void EsminiAdapter::onInitMessage(const Init::message_type::SharedPtr) {
	InitializeEsmini(oscFilePath);
}

void EsminiAdapter::onExitMessage(const Exit::message_type::SharedPtr){
	RCLCPP_DEBUG(me->get_logger(),"Received exit command");
	rclcpp::shutdown();
}

void EsminiAdapter::onStartMessage(const Start::message_type::SharedPtr) {
	if (SE_Init(oscFilePath.c_str(),0,0,0,0) == -1){
		RCLCPP_ERROR(me->get_logger(), "Failed to initialize esmini, aborting");
		exit(1);
	}
	SE_Step(); // Make sure that the scenario is started
	RCLCPP_INFO(me->get_logger(), "Esmini started");
}

/*!
 * \brief Callback to be executed by esmini when story board state changes.
 * \param name Name of the StoryBoardElement whose state has changed.
 * \param type Possible values: STORY = 1, ACT = 2, MANEUVER_GROUP = 3, MANEUVER = 4, EVENT = 5, ACTION = 6, UNDEFINED_ELEMENT_TYPE = 0.
 * \param state new state, possible values: STANDBY = 1, RUNNING = 2, COMPLETE = 3, UNDEFINED_ELEMENT_STATE = 0.
 */
void EsminiAdapter::onEsminiStoryBoardStateChange(const char* name, int type, int state){
	//Placeholder, TODO: Implement
	RCLCPP_DEBUG(me->get_logger(), "Esmini Storyboard State Change Name: %s, Type: %d, State: %d", name, type, state);
}

/*!
 * \brief Callback to be executed by esmini when a condition is triggered.
 * \param name Name of the condition that was triggered.
 * \param timestamp Timestamp when the condition triggered.
 */
void EsminiAdapter::onEsminiConditionTriggered(const char* name, double timestamp){
	//Placeholder, TODO: Implement
	RCLCPP_DEBUG(me->get_logger(), "Esmini Condition Trigger Name %s", name);
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

/*!
 * \brief Callback for MONR messages, reports the object position to esmini and advances the simulation time
 * \param monr ROS Monitor message of an object
 * \param id The object ID to which the monr belongs
*/
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
 * \brief Given a vector of object states at different timesteps,
 *	creates a trajectory consisting of trajectory points, one for each timestep.
 * Note: If there is no difference between consecutive states, the trajectory point is not added.
 * \param id The object ID to which the trajectory belongs
 * \param states A vector of object states
 * \return A trajectory consisting of trajectory points, one for each state.
 */
ATOS::Trajectory EsminiAdapter::getTrajectory(uint32_t id,std::vector<SE_ScenarioObjectState>& states) {
	ATOS::Trajectory trajectory;
	trajectory.name = "Esmini Trajectory for object " + std::to_string(id);
	auto saveTp = [&](auto& state){
		ATOS::Trajectory::TrajectoryPoint tp;
		tp.setXCoord(state.x);
		tp.setYCoord(state.y);
		tp.setZCoord(state.z);
		tp.setHeading(state.h);
		tp.setTime(state.timestamp);
		tp.setCurvature(0); // TODO: implement support for different curvature, now only support straight lines
		tp.setLongitudinalVelocity(state.speed * cos(state.wheel_angle));
		tp.setLateralAcceleration(state.speed * sin(state.wheel_angle));
		trajectory.points.push_back(tp);
	};
	for (auto it = states.begin()+1; it != states.end(); ++it) {
		if (it->x == (it-1)->x && it->y == (it-1)->y && // Nothing interesting happens within 1 timestep, skip
			it->z == (it-1)->z && it->h == (it-1)->h) {
			continue;
		}
		saveTp(*(it-1)); // Next timestep is different, save current one.
	}
	auto startTime = trajectory.points.front().getTime();

	// Subtract start time from all timesteps
	for (auto& tp : trajectory.points){
		tp.setTime(tp.getTime() - startTime);
	}
	return trajectory;
}

/*!
 * \brief Returns object states for each timestep.
 *	Inspired by ScenarioGateway::WriteStatesToFile from esmini lib.
 * \param oscFilePath Path to the xosc file
 * \param timeStep Time step to use for generating the trajectories
 * \param endTime End time of the simulation
 * \param states The return map of ids mapping to the respective object states at different timesteps
 * \return A map of object states, where the key is the object ID and the value is a vector of states
 */
void EsminiAdapter::getObjectStates(const std::string& oscFilePath, double timeStep, double endTime, std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states) {
	if (!SE_Init(oscFilePath.c_str(),0,0,0,0) == -1){
		RCLCPP_ERROR(me->get_logger(), "Failed to initialize esmini");
		exit(1);
	}
	// Populate States map with empty vector for each object
	for (int j = 0; j < SE_GetNumberOfObjects(); j++){
		states[SE_GetId(j)] = std::vector<SE_ScenarioObjectState>();
	}
	double accumTime = 0;
	while (accumTime < endTime) {
		SE_StepDT(timeStep);
		accumTime += timeStep;
		for (int j = 0; j < SE_GetNumberOfObjects(); j++){
			SE_ScenarioObjectState state;
			SE_GetObjectState(SE_GetId(j), &state);
			state.timestamp = accumTime; // Inject time since esmini does not do this
			states.at(SE_GetId(j)).push_back(state); // Copy state into vector
		}
	}
	RCLCPP_INFO(me->get_logger(), "Finished esmini simulation");
	SE_Close(); // Stop ScenarioEngine
}


/*!
 * \brief Runs the esmini simulator with the xosc file and returns the trajectories for each object
 * \param oscFilePath Path to the xosc file
 * \param timeStep Time step to use for generating the trajectories
 * \param endTime End time of the simulation TODO: not nessescary if xosc has a stop trigger at the end of the scenario
 * \param idToTraj The return map of ids mapping to the respective trajectories
 * \return A map of ids mapping to the respective trajectories
 */
std::map<uint32_t,ATOS::Trajectory> EsminiAdapter::extractTrajectories(const std::string& oscFilePath, double timeStep, double endTime, std::map<uint32_t,ATOS::Trajectory>& idToTraj){
	// Get object states
	std::map<uint32_t,std::vector<SE_ScenarioObjectState>> idToStates;
	getObjectStates(oscFilePath, timeStep, endTime, idToStates);

	// Extract trajectories
	for (auto& os : idToStates){
		auto id = os.first;
		auto objectStates = os.second;
		idToTraj[id] = getTrajectory(id, objectStates);
	}
	return idToTraj;
}

/*!
 * \brief Initialize the esmini simulator and perform subsequent setup tasks
 * \param oscFilePath Path to the xosc file
 */
void EsminiAdapter::InitializeEsmini(std::string& oscFilePath){
	std::map<uint32_t,ATOS::Trajectory> idToTraj;
	me->extractTrajectories(oscFilePath, 0.1, 14.0, idToTraj);
	RCLCPP_INFO(me->get_logger(), "Extracted %d trajectories", idToTraj.size());
	for (auto& it : idToTraj){
		auto id = it.first;
		auto traj = it.second;
		RCLCPP_DEBUG(me->get_logger(), "Trajectory for object %d has %d points", id, traj.points.size());
		// below is for dumping the trajectory points to the console
		/*for (auto& tp : traj.points){
			RCLCPP_INFO(me->get_logger(), "Trajectory point: %lf, %lf, %lf, %lf, %ld", tp.getXCoord(), tp.getYCoord(), tp.getZCoord(), tp.getHeading(), tp.getTime().count());
		}*/
	}

	// Inject Meastro as controller for the DefaultControlled entities
	// TODO

	// Handle triggers and story board element changes
	SE_RegisterConditionCallback(&onEsminiConditionTriggered);
	SE_RegisterStoryBoardElementStateChangeCallback(&onEsminiStoryBoardStateChange);

	// Populate the map tracking Object ID -> esmini index
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

	// Set up services
	using TestOriginSrv = atos_interfaces::srv::GetTestOrigin;
	using namespace std::chrono;
	me->testOriginClient = me->nTimesWaitForService<TestOriginSrv>(3, 1s, ServiceNames::getTestOrigin);
	TestOriginSrv::Response::SharedPtr response;

	// Call services TODO: Call only when needed
	me->callService(1s ,me->testOriginClient, response);

	RCLCPP_DEBUG(me->get_logger(), "Test origin: %lf, %lf, %lf", response->origin.position.latitude, response->origin.position.longitude, response->origin.position.altitude);
	return retval;
}