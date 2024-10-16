/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esminiadapter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <functional>
#include <chrono>
#include <cmath>
#include <chrono>
#include <regex>

#include "atos_interfaces/msg/cartesian_trajectory.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "trajectory.hpp"
#include "string_utility.hpp"


using namespace ROSChannels;
using TestOriginSrv = atos_interfaces::srv::GetTestOrigin;
using ObjectTrajectorySrv = atos_interfaces::srv::GetObjectTrajectory;
using ObjectTriggerSrv = atos_interfaces::srv::GetObjectTriggerStart;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;




std::shared_ptr<EsminiAdapter> EsminiAdapter::me = nullptr;
std::unordered_map<int, std::string> EsminiAdapter::atosIDToObjectName = std::unordered_map<int, std::string>();
std::unordered_map<std::string, int> EsminiAdapter::objectNameToAtosId = std::unordered_map<std::string, int>();
std::map<uint32_t,ATOS::Trajectory> EsminiAdapter::atosObjectIdToTraj = std::map<uint32_t,ATOS::Trajectory>();
std::unordered_map<int, int> EsminiAdapter::atosIdToEsminiId = std::unordered_map<int, int>();

std::unordered_map<uint32_t,std::shared_ptr<ROSChannels::Monitor::Sub>> EsminiAdapter::monrSubscribers = std::unordered_map<uint32_t,std::shared_ptr<Monitor::Sub>>();
std::shared_ptr<rclcpp::Service<ObjectTrajectorySrv>> EsminiAdapter::objectTrajectoryService = std::shared_ptr<rclcpp::Service<ObjectTrajectorySrv>>();
std::shared_ptr<rclcpp::Service<TestOriginSrv>> EsminiAdapter::testOriginService = std::shared_ptr<rclcpp::Service<TestOriginSrv>>();
geographic_msgs::msg::GeoPose EsminiAdapter::testOrigin = geographic_msgs::msg::GeoPose();

EsminiAdapter::EsminiAdapter() : Module(moduleName),
	storyBoardElementStateChangePub(*this),
	connectedObjectIdsSub(*this, &EsminiAdapter::onConnectedObjectIdsMessage),
	exitSub(*this, &EsminiAdapter::onStaticExitMessage),
	stateChangeSub(*this, &EsminiAdapter::onStaticStateChangeMessage),
	applyTrajTransform(false),
	testOriginSet(false)
 {
	oscFilePathClient_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	objectIdsClient_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	oscFilePathClient_ = create_client<atos_interfaces::srv::GetOpenScenarioFilePath>(ServiceNames::getOpenScenarioFilePath, rmw_qos_profile_services_default, oscFilePathClient_cb_group_);
	objectIdsClient_ = create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds, rmw_qos_profile_services_default, objectIdsClient_cb_group_);
	declare_parameter("timestep", 0.1);

}

/*!
 * \brief Fetches the open drive file path from the open scenario file parameter
 * \return Configured path
*/
std::filesystem::path EsminiAdapter::getOpenDriveFile()
{
	std::filesystem::path odrFilePath;
	if (SE_GetODRFilename() != nullptr) {
		odrFilePath = std::filesystem::path(SE_GetODRFilename()); 
		RCLCPP_INFO(me->get_logger(), "Got ODR file %s from scenario", odrFilePath.string().c_str());
	}
	else {
		RCLCPP_DEBUG(me->get_logger(), "No ODR file found");
	}

	if (odrFilePath.is_absolute()) {
		return odrFilePath;
	}
	else {
		// Look for the file relative the scenario file (ie root openx dir)
		auto odrCatalog = std::filesystem::path(me->oscFilePath).parent_path();
		std::filesystem::path joinedPath = odrCatalog / odrFilePath;
		if (!std::filesystem::exists(joinedPath)) {
			throw std::runtime_error("ODR file " + joinedPath.string() + " does not exist");
		}
		else {
			RCLCPP_INFO(me->get_logger(), "Found ODR file at %s", joinedPath.string().c_str());
		}
		return joinedPath;
	}

}

/*!
 * \brief Creates an instance and initialize esmini if none exists, otherwise returns the existing instance.
 * \return the sole EsminiAdapter instance.
 */
std::shared_ptr<EsminiAdapter> EsminiAdapter::instance() {
	if (me == nullptr) {
		me = std::shared_ptr<EsminiAdapter>(new EsminiAdapter());
		// Start listening to connected object ids
		me->connectedObjectIdsSub = ROSChannels::ConnectedObjectIds::Sub(*me,&EsminiAdapter::onConnectedObjectIdsMessage);
		me->exitSub = ROSChannels::Exit::Sub(*me,&EsminiAdapter::onStaticExitMessage);
		me->stateChangeSub = ROSChannels::StateChange::Sub(*me,&EsminiAdapter::onStaticStateChangeMessage);
		
	}
	return me;
}

/*!
 * \brief This callback is executed when objects are connected, and creates Monitor subscribers for all connected objects.
 * \param msg Array of object IDs
 */
void EsminiAdapter::onConnectedObjectIdsMessage(const ConnectedObjectIds::message_type::SharedPtr msg) {
	for (uint32_t id : msg->ids) {
		if (me->monrSubscribers.find(id) == me->monrSubscribers.end()){
			me->monrSubscribers[id] = std::make_shared<Monitor::Sub>(*me, id, std::bind(&EsminiAdapter::onMonitorMessage, me, _1, id));	
		}
	}
}

/**
 * @brief To ensure that EsminiAdapter follows the states, we execute actions only when going from IDLE to INITIALIZED,
 * from ARMED to RUNNING and from any state to ABORTING. We do this instead of subscribing to "/init", "/start", etc.,
 * because we only want to execute the actions once when changing to these states.
 * 
 * @param msg StateChange message
 */
void EsminiAdapter::onStaticStateChangeMessage(const ROSChannels::StateChange::message_type::SharedPtr msg) {
	int prevState = msg->prev_state;
	int currentState = msg->current_state;

	if (prevState == OBCState_t::OBC_STATE_ARMED && currentState == OBCState_t::OBC_STATE_RUNNING) {
		me->handleStartCommand();
	}
	else if (currentState == OBCState_t::OBC_STATE_ABORTING) {
		me->handleAbortCommand();
	}
}


void EsminiAdapter::handleAbortCommand() {
	SE_Close();
	RCLCPP_INFO(me->get_logger(), "Esmini ScenarioEngine stopped due to Abort");
}

void EsminiAdapter::fetchOSCFilePath()
{
	// Get the file path of xosc file
	std::promise<bool> done;
	auto callback = [&](rclcpp::Client<atos_interfaces::srv::GetOpenScenarioFilePath>::SharedFuture future) {
		auto response = future.get();
		me->oscFilePath = response->path;
		if (response->md5hash != me->scenarioFileMd5hash) {
			me->scenarioFileMd5hash = response->md5hash;
			me->runSimulation = true;
		}
		me->scenarioFileMd5hash = response->md5hash;
		done.set_value(true);
	};

	auto request = std::make_shared<atos_interfaces::srv::GetOpenScenarioFilePath::Request>();
	auto future = me->oscFilePathClient_->async_send_request(request, std::move(callback));
	if (done.get_future().wait_for(250ms) == std::future_status::timeout) {
		RCLCPP_ERROR(me->get_logger(), "Failed to fetch open scenario file path");
	}
	
}

void EsminiAdapter::onStaticExitMessage(const ROSChannels::Exit::message_type::SharedPtr)
{
	SE_Close();
	RCLCPP_DEBUG(me->get_logger(),"Received exit command");
	rclcpp::shutdown();
}

void EsminiAdapter::handleStartCommand()
{
	if (SE_Init(me->oscFilePath.c_str(),0,0,0,0) < 0) {
		throw std::runtime_error("Failed to initialize esmini with scenario file " + me->oscFilePath.string());
	}
	// Handle triggers and story board element changes
	SE_RegisterStoryBoardElementStateChangeCallback(&handleStoryBoardElementChange);

	SE_Step(); // Make sure that the scenario is started
	RCLCPP_INFO(me->get_logger(), "Esmini ScenarioEngine started");
}

/*!
 * \brief Callback to be executed by esmini when story board state changes.
 * 		If story board element is an action, and the action is supported, the action is run.
 * \param name Name of the StoryBoardElement whose state has changed.
 * \param type Possible values: STORY_BOARD = 1, STORY = 2, ACT = 3, MANEUVER_GROUP = 4, MANEUVER = 5, EVENT = 6, ACTION = 7, UNDEFINED_ELEMENT_TYPE = 0.
 * \param state new state, possible values: STANDBY = 1, RUNNING = 2, COMPLETE = 3, UNDEFINED_ELEMENT_STATE = 0.
 */
void EsminiAdapter::handleStoryBoardElementChange(
	const char *name,
	int type,
	int state,
	const char *full_path)
{
	RCLCPP_INFO(me->get_logger(), "Storyboard state changed! Name: %s, Type: %d, State: %d, Full path: %s", name, type, state, full_path);

	atos_interfaces::msg::StoryBoardElementStateChange msg;
	msg.name = name;
	msg.type = type;
	msg.state = state;
	msg.full_path = full_path;

	me->storyBoardElementStateChangePub.publish(msg);
}

/*!
 * \brief Utility function to convert a ROS MONR message to Esmini representation
 *			and report the object position to Esmini
 * \param monr ROS Monitor message of an object
 * \param id The object ID to which the monr belongs
 */
void EsminiAdapter::reportObjectPosition(const Monitor::message_type::SharedPtr monr, uint32_t esminiObjectId){
	// Conversions from ROS to Esmini
	auto ori = monr->pose.pose.orientation;
	auto quat = tf2::Quaternion(ori.x, ori.y, ori.z, ori.w);
	tf2::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	geometry_msgs::msg::Point pos = monr->pose.pose.position;
	if (me->applyTrajTransform) {
		me->crsTransformation->apply(pos, PJ_INV);
	}
	auto speed = monr->velocity.twist.linear;

	// Reporting to Esmini
	int timestamp = 0; // Not really used according to esmini documentation
	SE_ReportObjectPos(esminiObjectId, timestamp, pos.x, pos.y, pos.z, yaw, pitch, roll);
	SE_ReportObjectSpeed(esminiObjectId, speed.x);
}

/*!
 * \brief Callback for MONR messages, reports the object position to esmini and advances the simulation time
 * \param monr ROS Monitor message of an object
 * \param id The object ID to which the monr belongs
*/
void EsminiAdapter::onMonitorMessage(const Monitor::message_type::SharedPtr monr, uint32_t ATOSObjectId) {
	if (auto idMapping = atosIdToEsminiId.find(ATOSObjectId); idMapping != atosIdToEsminiId.end()) {
		reportObjectPosition(monr, idMapping->first); // Report object position to esmini
		SE_Step(); // Advance the "simulation world"-time
	} 
	else {
		RCLCPP_WARN(me->get_logger(), "Received MONR message for object with ATOS Object ID %d, but no such object exists in the scenario", ATOSObjectId);
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
ATOS::Trajectory EsminiAdapter::getTrajectoryFromObjectState(
	uint32_t id,
	std::vector<SE_ScenarioObjectState>& states)
{
	ATOS::Trajectory trajectory(me->get_logger());
	trajectory.name = "Esmini Trajectory for object " + std::to_string(id);
	if (states.empty()) {
		return trajectory;
	}

	RCLCPP_DEBUG(me->get_logger(), "Creating trajectory for object %d", id);
	auto saveTp = [&](auto& state, auto& prevState) {
		ATOS::Trajectory::TrajectoryPoint tp(me->get_logger());
		double currLonVel = state.speed * cos(state.wheel_angle);
		double currLatVel = state.speed * sin(state.wheel_angle);
		double prevLonVel = prevState.speed * cos(prevState.wheel_angle);
		double prevLatVel = prevState.speed * sin(prevState.wheel_angle);

		tp.setXCoord(state.x);
		tp.setYCoord(state.y);
		tp.setZCoord(state.z);
		tp.setHeading(state.h);
		tp.setTime(state.timestamp);
		tp.setCurvature(0); // TODO: implement support for different curvature, now only support straight lines
		tp.setLongitudinalVelocity(currLonVel);
		tp.setLateralVelocity(currLatVel);
		if (state.timestamp != prevState.timestamp) {
			tp.setLongitudinalAcceleration((currLonVel - prevLonVel) / (state.timestamp - prevState.timestamp));
			tp.setLateralAcceleration((currLatVel - prevLatVel) / (state.timestamp - prevState.timestamp));
		}
		else {
			tp.setLongitudinalAcceleration(0);
			tp.setLateralAcceleration(0);
		}

		trajectory.points.push_back(tp);
	};

	if (states.size() > 1) {
		for (auto it = states.begin()+1; it != states.end(); ++it) {
			if (it->x == (it-1)->x && it->y == (it-1)->y && // Nothing interesting happens within 1 timestep, skip
				it->z == (it-1)->z && it->h == (it-1)->h) {
				continue;
			}
			saveTp(*it,*(it-1)); // Next timestep is different, save current one.
		}
	}
	if (trajectory.points.size() == 0) {
		saveTp(states.front(), states.front()); // Only one state or no points, save it.
	}
	auto startTime = trajectory.points.front().getTime();

	// Subtract start time from all timesteps
	for (auto& tp : trajectory.points){
		tp.setTime(tp.getTime() - startTime);
	}
	return trajectory;
}

/*!
 * \brief Given a RM_georeference converts into a proj string
 * \param geoRef The geo reference to convert
 * \return The proj string
 */
std::string EsminiAdapter::projStrFromGeoReference(RM_GeoReference& geoRef) {
	std::string projStringFrom = "+proj=";
	if (strlen(geoRef.proj_) != 0) {
		projStringFrom += std::string(geoRef.proj_) + " ";
	}
	else {
		throw std::runtime_error("No projection found in geo reference");
	}
	if (!std::isnan(geoRef.lat_0_)) {
		projStringFrom += "+lat_0=" + std::to_string(geoRef.lat_0_) + " ";
	}
	if (!std::isnan(geoRef.lon_0_)) {
		projStringFrom += "+lon_0=" + std::to_string(geoRef.lon_0_) + " ";
	}
	if (!std::isnan(geoRef.k_)) {
		projStringFrom += "+k=" + std::to_string(geoRef.k_) + " ";
	}
	if (!std::isnan(geoRef.k_0_)) {
		projStringFrom += "+k_0=" + std::to_string(geoRef.k_0_) + " ";
	}
	if (!std::isnan(geoRef.x_0_)) {
		projStringFrom += "+x_0=" + std::to_string(geoRef.x_0_) + " ";
	}
	if (!std::isnan(geoRef.y_0_)) {
		projStringFrom += "+y_0=" + std::to_string(geoRef.y_0_) + " ";
	}
	if (strlen(geoRef.ellps_) != 0) {
		projStringFrom += "+ellps=" + std::string(geoRef.ellps_) + " ";
	}
	if (strlen(geoRef.units_) != 0) {
		projStringFrom += "+units=" + std::string(geoRef.units_) + " ";
	}
	if (strlen(geoRef.vunits_) != 0) {
		projStringFrom += "+vunits=" + std::string(geoRef.vunits_) + " ";
	}
	if (strlen(geoRef.datum_) != 0) {
		projStringFrom += "+datum=" + std::string(geoRef.datum_) + " ";
	}
	if (strlen(geoRef.geo_id_grids_) != 0) {
		projStringFrom += "+geoidgrids=" + std::string(geoRef.geo_id_grids_) + " ";
	}
	if (!std::isnan(geoRef.zone_)) {
		projStringFrom += "+zone=" + std::to_string(geoRef.zone_) + " ";
	}
	if (geoRef.towgs84_ != 0) {
		projStringFrom += "+towgs84=" + std::to_string(geoRef.towgs84_) + " ";
	}
	if (strlen(geoRef.axis_) != 0) {
		projStringFrom += "+axis=" + std::string(geoRef.axis_) + " ";
	}
	if (!std::isnan(geoRef.lon_wrap_)) {
		projStringFrom += "+lon_wrap=" + std::to_string(geoRef.lon_wrap_) + " ";
	}
	if (!std::isnan(geoRef.over_)) {
		projStringFrom += "+over=" + std::to_string(geoRef.over_) + " ";
	}
	if (strlen(geoRef.pm_) != 0) {
		projStringFrom += "+pm=" + std::string(geoRef.pm_) + " ";
	}
	projStringFrom += "+no_defs";
	RCLCPP_DEBUG(me->get_logger(), "Created proj string: %s", projStringFrom.c_str());
	return projStringFrom;
}

/*!
 * \brief Returns object states for each timestep by simulating the loaded scenario.
 *  The simulation is stopped if there is no vehicle movement and at least 
 * 	MIN_SCENARIO_TIME has passed or if more than MAX_SCENARIO_TIME has passed.
 *	Inspired by ScenarioGateway::WriteStatesToFile from esmini lib.
 *  
 * \param timeStep Time step to use for generating the trajectories
 * \param endTime End time of the simulation
 * \param states The return map of ids mapping to the respective object states at different timesteps
 * \return A map of object states, where the key is the object ID and the value is a vector of states
 */
void EsminiAdapter::getObjectStates(
	double timeStep,
	std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states)
{
	double accumTime = 0.0;
	auto pushCurrentState = [&](auto& st, auto& index) {
		auto id = SE_GetId(index);
		//SE_SetAlignModeZ(id, 0); // Disable Z-alignment, not implemented in esmini yet
		SE_ScenarioObjectState s;
		SE_GetObjectState(id, &s);
		s.timestamp = accumTime;
		st.at(id).push_back(s);
	};

	// Populate States map with vector for each object containing the initial state
	SE_StepDT(timeStep);
	accumTime += timeStep;
	for (int j = 0; j < SE_GetNumberOfObjects(); j++){
		states[SE_GetId(j)] = std::vector<SE_ScenarioObjectState>();
		pushCurrentState(states, j);
	}
	constexpr double MIN_SCENARIO_TIME = 10.0;
	constexpr double MAX_SCENARIO_TIME = 3600.0;
	bool stopSimulation = false;
	while (!stopSimulation) {
		if (SE_GetQuitFlag() != 0) {
			break;
		}
		
		SE_StepDT(timeStep);
		accumTime += timeStep;
		for (int j = 0; j < SE_GetNumberOfObjects(); j++){
			pushCurrentState(states, j);
		}
		bool noMovement = std::all_of(states.begin(), states.end(), [&](auto& pair) {
			return pair.second.back().speed < 0.1;
		});
		bool atLeastMinTimePassed = accumTime > MIN_SCENARIO_TIME;
		bool moreThanMaxTimePassed = accumTime > MAX_SCENARIO_TIME;
		stopSimulation = (noMovement && atLeastMinTimePassed) || moreThanMaxTimePassed;
	}
	if (accumTime > MAX_SCENARIO_TIME) {
		RCLCPP_WARN(me->get_logger(), "Scenario time limit reached, stopping simulation");
	}
	else if (accumTime < MIN_SCENARIO_TIME + timeStep) {
		RCLCPP_WARN(me->get_logger(), "Ran scenario for the minimum time %.2f, possibly no movement in scenario", MIN_SCENARIO_TIME);
	}
	RCLCPP_INFO(me->get_logger(), "Finished %f s simulation", accumTime);
}


/*!
 * \brief Runs the esmini simulator with the xosc file and returns the trajectories for each object
 * \param timeStep Time step to use for generating the trajectories
 * \return A map of ids mapping to the respective trajectories
 */
std::map<uint32_t, ATOS::Trajectory> EsminiAdapter::extractTrajectories(
	double timeStep)
{
	std::map<uint32_t,ATOS::Trajectory> esminiObjectIdToTraj = std::map<uint32_t,ATOS::Trajectory>();

	// Get object states
	std::map<uint32_t,std::vector<SE_ScenarioObjectState>> esminiIdToStates;
	getObjectStates(timeStep, esminiIdToStates);

	// Extract trajectories
	for (auto& os : esminiIdToStates) {
		auto id = os.first;
		auto objectStates = os.second;
		auto traj = getTrajectoryFromObjectState(id, objectStates);
		// Apply CRS transform if OpenDrive CRS Transformation is defined
		if (me->applyTrajTransform){
			RCLCPP_DEBUG(me->get_logger(), "Applying CRS transformation to trajectory for object %d", id);
			me->crsTransformation->apply(traj.points);
		}
		esminiObjectIdToTraj.insert(std::pair<uint32_t,ATOS::Trajectory>(id, traj));
	}
	return esminiObjectIdToTraj;
}

/*!
 * \brief Initialize the esmini simulator and perform subsequent setup tasks.
 * Can be called many times, each time the test is initialized. 
 */
void EsminiAdapter::runEsminiSimulation()
{
	me->atosObjectIdToTraj.clear();
	me->atosIdToEsminiId.clear();
	me->atosIDToObjectName.clear();
	me->objectNameToAtosId.clear();
	me->pathPublishers.clear();
	me->gnssPathPublishers.clear();
	auto logFilePath = std::string(getenv("HOME")) + std::string("/.astazero/ATOS/logs/esmini.log");
	SE_SetLogFilePath(logFilePath.c_str());
	SE_Close(); // Stop ScenarioEngine in case it is running
	RM_Close(); // Stop RoadManager in case it is running

	RCLCPP_INFO(me->get_logger(), "Initializing esmini with scenario file %s", me->oscFilePath.c_str());
	if (SE_Init(me->oscFilePath.c_str(),1,0,0,0) < 0) { // Disable controllers, let DefaultController be used
		throw std::runtime_error("Failed to initialize esmini with scenario file " + me->oscFilePath.string() + ". For more information, see " + logFilePath + ".");
	}
	auto odrFile = getOpenDriveFile();
	if (RM_Init(odrFile.c_str()) < 0) {
		throw std::runtime_error(std::string("Failed to initialize with odr file ").append(odrFile));
	}

	
	// Call RM_GetOpenDriveGeoReference to get the RM_GeoReference struct
	RM_GeoReference geoRef;
	if (RM_GetOpenDriveGeoReference(&geoRef) == 0) {
		try {
			std::string projStringFrom = projStrFromGeoReference(geoRef);
			std::string toDatum = "WGS84";
			auto llh_0 = CRSTransformation::projToLLH(projStringFrom, toDatum);
			RCLCPP_INFO(me->get_logger(), "llh origin: %lf, %lf, %lf", llh_0[0], llh_0[1], llh_0[2]);
			me->testOrigin.position.latitude = llh_0[0];
			me->testOrigin.position.longitude = llh_0[1];
			me->testOrigin.position.altitude = llh_0[2];
			me->testOriginSet = true;

			std::string projStringTo = "+proj=tmerc +lat_0=" + std::to_string(llh_0[0]) + 
													" +lon_0=" + std::to_string(llh_0[1]) + 
													" +datum="+ toDatum + " +units=m +no_defs";

			me->crsTransformation = std::make_shared<CRSTransformation>(projStringFrom, projStringTo);
			me->applyTrajTransform = true;
		} 
		catch (std::exception& e) {
			RCLCPP_ERROR(me->get_logger(), e.what());
			return;
		}
	} else {
		RCLCPP_WARN(me->get_logger(), "Failed to get OpenDRIVE geo reference from RoadManager");
	}
	RM_Close();

	auto response = std::make_shared<atos_interfaces::srv::GetObjectIds::Response>();
	auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();

	std::promise<void> idsFetched;
	auto objectNameAndAtosIDsCallback = [&](rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedFutureWithRequest future) {
		auto response = future.get();
		for (int i = 0; i < response.second->ids.size(); i++) {
			me->atosIDToObjectName[response.second->ids[i]] = response.second->names[i];
			me->objectNameToAtosId[response.second->names[i]] = response.second->ids[i];
		}
		idsFetched.set_value();
	};

	auto future = me->objectIdsClient_->async_send_request(request, std::move(objectNameAndAtosIDsCallback));
	if (idsFetched.get_future().wait_for(250ms) == std::future_status::timeout) {
		RCLCPP_ERROR(me->get_logger(), "Failed to fetch object names and ids");
		return;
	}

	RCLCPP_INFO(me->get_logger(), "Starting extracting trajectories");
	double timeStep = me->get_parameter("timestep").as_double();
	std::map<uint32_t,ATOS::Trajectory> esminiIdToTraj =  me->extractTrajectories(timeStep);
	// Fill atosIdToTraj with the extracted trajectories
	for (auto& it : esminiIdToTraj) {
		auto esminiId = it.first;
		auto traj = it.second;
		auto objectName = SE_GetObjectName(esminiId);
			if (auto idMapping = me->objectNameToAtosId.find(objectName); idMapping != me->objectNameToAtosId.end()) {
				auto atos_id = idMapping->second;
				me->atosObjectIdToTraj.emplace(atos_id, traj);
				me->atosIdToEsminiId.emplace(atos_id, esminiId);
				RCLCPP_INFO(me->get_logger(), "Extracted trajectory for object %s with size %d", objectName, traj.points.size());
				
				me->pathPublishers.emplace(atos_id, ROSChannels::Path::Pub(*me, atos_id));
				me->pathPublishers.at(atos_id).publish(traj.toPath());
				std::array<double,3> llh_0 = {me->testOrigin.position.latitude, me->testOrigin.position.longitude, me->testOrigin.position.altitude};
				me->gnssPathPublishers.emplace(atos_id, ROSChannels::GNSSPath::Pub(*me, atos_id));
				me->gnssPathPublishers.at(atos_id).publish(traj.toGeoJSON(llh_0));
			}
			else {
				RCLCPP_DEBUG(me->get_logger(), "Object %s is not an active object in the scenario", objectName);
			}

		for (auto& tp : traj.points){
			RCLCPP_DEBUG(me->get_logger(), "Trajectory point: %lf, %lf, %lf, %lf, %ld", tp.getXCoord(), tp.getYCoord(), tp.getZCoord(), tp.getHeading(), tp.getTime().count());
		}
	}
	SE_Close(); // Stop ScenarioEngine
}

void EsminiAdapter::onRequestObjectTrajectory(
	const std::shared_ptr<ObjectTrajectorySrv::Request> req,
	std::shared_ptr<ObjectTrajectorySrv::Response> res)
{
	res->id;
	me->fetchOSCFilePath();
	if (me->runSimulation || me->atosObjectIdToTraj.find(req->id) == me->atosObjectIdToTraj.end()) {
		me->runEsminiSimulation();
		me->runSimulation = false;
	}

	try {
		res->trajectory = me->atosObjectIdToTraj.at(req->id).toCartesianTrajectory();
		res->success = true;
	}
	catch (std::out_of_range& e){
		RCLCPP_ERROR(me->get_logger(), "Esmini trajectory service called, no trajectory found for object %d", req->id);
		res->success = false;
	}
}

void EsminiAdapter::onRequestTestOrigin(
	const std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Request>,
	std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Response> res)
{
	while (me->testOriginSet == false){
		RCLCPP_DEBUG(me->get_logger(), "Waiting for test origin to be available");
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	res->origin.position.latitude = me->testOrigin.position.latitude;
	res->origin.position.longitude = me->testOrigin.position.longitude;
	res->origin.position.altitude = me->testOrigin.position.altitude;
}

/*!
 * \brief initializeModule Initializes this module by creating log,
 *			connecting to the message queue bus, setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int EsminiAdapter::initializeModule() {
	int retval = 0;

	me->objectTrajectoryService = me->create_service<ObjectTrajectorySrv>(ServiceNames::getObjectTrajectory,
		std::bind(&EsminiAdapter::onRequestObjectTrajectory, _1, _2));
	me->testOriginService = me->create_service<atos_interfaces::srv::GetTestOrigin>(ServiceNames::getTestOrigin,
		std::bind(&EsminiAdapter::onRequestTestOrigin, _1, _2));

	return retval;
}