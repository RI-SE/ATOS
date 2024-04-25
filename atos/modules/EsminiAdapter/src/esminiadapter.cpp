/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esminiadapter.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <regex>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "atos_interfaces/msg/cartesian_trajectory.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "string_utility.hpp"
#include "trajectory.hpp"
#include "util.h"

using namespace ROSChannels;
using TestOriginSrv = atos_interfaces::srv::GetTestOrigin;
using ObjectTrajectorySrv = atos_interfaces::srv::GetObjectTrajectory;
using ObjectTriggerSrv = atos_interfaces::srv::GetObjectTriggerStart;
using ObjectIpSrv = atos_interfaces::srv::GetObjectIp;
using SetObjectIpSrv = atos_interfaces::srv::SetObjectIp;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

std::shared_ptr<EsminiAdapter> EsminiAdapter::me = nullptr;
std::unordered_map<int, int> EsminiAdapter::ATOStoEsminiObjectId = std::unordered_map<int, int>();
std::map<uint32_t, ATOS::Trajectory> EsminiAdapter::idToTraj = std::map<uint32_t, ATOS::Trajectory>();
std::map<uint32_t, std::string> EsminiAdapter::idToIp = std::map<uint32_t, std::string>();

std::unordered_map<uint32_t, std::shared_ptr<ROSChannels::Monitor::Sub>> EsminiAdapter::monrSubscribers = std::unordered_map<uint32_t, std::shared_ptr<Monitor::Sub>>();
std::shared_ptr<rclcpp::Service<ObjectTrajectorySrv>> EsminiAdapter::objectTrajectoryService = std::shared_ptr<rclcpp::Service<ObjectTrajectorySrv>>();
std::shared_ptr<rclcpp::Service<ObjectTriggerSrv>> EsminiAdapter::startOnTriggerService = std::shared_ptr<rclcpp::Service<ObjectTriggerSrv>>();
std::shared_ptr<rclcpp::Service<ObjectIpSrv>> EsminiAdapter::objectIpService = std::shared_ptr<rclcpp::Service<ObjectIpSrv>>();
std::shared_ptr<rclcpp::Service<TestOriginSrv>> EsminiAdapter::testOriginService = std::shared_ptr<rclcpp::Service<TestOriginSrv>>();
std::shared_ptr<rclcpp::Service<SetObjectIpSrv>> EsminiAdapter::setObjectIpService = std::shared_ptr<rclcpp::Service<SetObjectIpSrv>>();
std::vector<uint32_t> EsminiAdapter::delayedStartIds = std::vector<uint32_t>();
geographic_msgs::msg::GeoPose EsminiAdapter::testOrigin = geographic_msgs::msg::GeoPose();

EsminiAdapter::EsminiAdapter() : Module(moduleName),
                                 startObjectPub(*this),
                                 v2xPub(*this),
                                 connectedObjectIdsSub(*this, &EsminiAdapter::onConnectedObjectIdsMessage),
                                 exitSub(*this, &EsminiAdapter::onStaticExitMessage),
                                 stateChangeSub(*this, &EsminiAdapter::onStaticStateChangeMessage),
                                 triggerEventPub(*this),
                                 applyTrajTransform(false),
                                 testOriginSet(false) {
  declare_parameter("open_scenario_file", "");
}

/*!
 * \brief Fetches the open_scenario_file parameter from node and prepends
 * 		necessary config path.
 * \return Configured path
 */
std::filesystem::path EsminiAdapter::getOpenScenarioFileParameter() {
  // Get the file path of xosc file
  std::string result;
  auto success = me->get_parameter("open_scenario_file", result);
  if (!success) {
    throw std::runtime_error("Could not read parameter open_scenario_file");
  }
  if (std::filesystem::path(result).is_absolute()) {
    return result;
  } else {
    char path[MAX_FILE_PATH];
    UtilGetOscDirectoryPath(path, MAX_FILE_PATH);
    return std::string(path) + result;
  }
}

/*!
 * \brief Fetches the open drive file path from the open scenario file parameter
 * \return Configured path
 */
std::filesystem::path EsminiAdapter::getOpenDriveFile() {
  std::filesystem::path odrPath;
  if (SE_GetODRFilename() != nullptr) {
    odrPath = std::filesystem::path(SE_GetODRFilename());
    RCLCPP_INFO(me->get_logger(), "Found ODR file %s", odrPath.string().c_str());
  } else {
    RCLCPP_DEBUG(me->get_logger(), "No ODR file found");
  }

  if (odrPath.is_absolute()) {
    return odrPath;
  } else {
    char path[MAX_FILE_PATH];
    UtilGetConfDirectoryPath(path, MAX_FILE_PATH);
    return std::string(path) + odrPath.string();
  }
}

/*!
 * \brief Sets the OpenSCENARIO file path to use
 * \param path OpenSCENARIO file path
 */
void EsminiAdapter::setOpenScenarioFile(
    const std::filesystem::path &path) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("Could not open file " + path.string());
  }
  oscFilePath = path;
}

/*!
 * \brief Creates an instance and initialize esmini if none exists, otherwise returns the existing instance.
 * \return the sole EsminiAdapter instance.
 */
std::shared_ptr<EsminiAdapter> EsminiAdapter::instance() {
  if (me == nullptr) {
    me = std::shared_ptr<EsminiAdapter>(new EsminiAdapter());
    // Start listening to connected object ids
    me->connectedObjectIdsSub = ROSChannels::ConnectedObjectIds::Sub(*me, &EsminiAdapter::onConnectedObjectIdsMessage);
    me->exitSub = ROSChannels::Exit::Sub(*me, &EsminiAdapter::onStaticExitMessage);
    me->stateChangeSub = ROSChannels::StateChange::Sub(*me, &EsminiAdapter::onStaticStateChangeMessage);
    // Start V2X publisher
    me->v2xPub = ROSChannels::V2X::Pub(*me);
  }
  return me;
}

/*!
 * \brief This callback is executed when objects are connected, and creates Monitor subscribers for all connected objects.
 * \param msg Array of object IDs
 */
void EsminiAdapter::onConnectedObjectIdsMessage(const ConnectedObjectIds::message_type::SharedPtr msg) {
  for (uint32_t id : msg->ids) {
    if (me->monrSubscribers.find(id) == me->monrSubscribers.end()) {
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

  if (prevState == OBCState_t::OBC_STATE_IDLE && currentState == OBC_STATE_INITIALIZED) {
    me->handleInitCommand();
  } else if (prevState == OBCState_t::OBC_STATE_ARMED && currentState == OBCState_t::OBC_STATE_RUNNING) {
    me->handleStartCommand();
  } else if (currentState == OBCState_t::OBC_STATE_ABORTING) {
    me->handleAbortCommand();
  }
}

void EsminiAdapter::handleAbortCommand() {
  SE_Close();
  RCLCPP_INFO(me->get_logger(), "Esmini ScenarioEngine stopped due to Abort");
}

void EsminiAdapter::handleInitCommand() {
  try {
    setOpenScenarioFile(getOpenScenarioFileParameter());
  } catch (std::exception &e) {
    RCLCPP_ERROR(me->get_logger(), e.what());
    return;
  }

  me->InitializeEsmini();

  std::array<double, 3> llh_0 = {me->testOrigin.position.latitude, me->testOrigin.position.longitude, me->testOrigin.position.altitude};
  for (auto &it : me->idToTraj) {
    me->gnssPathPublishers.emplace(it.first, ROSChannels::GNSSPath::Pub(*me, it.first));
    me->gnssPathPublishers.at(it.first).publish(it.second.toGeoJSON(llh_0));
  }
}

void EsminiAdapter::onStaticExitMessage(const ROSChannels::Exit::message_type::SharedPtr) {
  SE_Close();
  RCLCPP_DEBUG(me->get_logger(), "Received exit command");
  rclcpp::shutdown();
}

void EsminiAdapter::handleStartCommand() {
  if (SE_Init(me->oscFilePath.c_str(), 0, 0, 0, 0) < 0) {
    throw std::runtime_error("Failed to initialize esmini with scenario file " + me->oscFilePath.string());
  }
  // Handle triggers and story board element changes
  SE_RegisterStoryBoardElementStateChangeCallback(&handleStoryBoardElementChange);

  SE_Step(); // Make sure that the scenario is started
  RCLCPP_INFO(me->get_logger(), "Esmini ScenarioEngine started");
}

/*!
 * \brief Split action into ID and action name.
 * \param actionName Action name in the form ActorObjectId,Action
 * \return pair of actor object ID and action name
 */
std::pair<uint32_t, std::string> EsminiAdapter::parseAction(const std::string &actionName) {
  std::vector<std::string> res;
  split(actionName, ',', res);
  if (res.size() < 2) {
    throw std::runtime_error("Action name " + actionName + "  is not of the form ActorObjectId,Action");
  }
  return {std::stoul(res[0]), res[1]};
}

/*!
 * \brief Check if action is a start action.
 * \param action Action name
 * \return true if action is a start action, false otherwise
 */
bool EsminiAdapter::isStartAction(const std::string &action) {
  return std::regex_search(action, std::regex("^(begin|start)", std::regex_constants::icase));
}

/*!
 * \brief Check if action is a DENM action.
 * \param action Action name
 * \return true if action is a DENM action, false otherwise
 */
bool EsminiAdapter::isSendDenmAction(const std::string &action) {
  return std::regex_search(action, std::regex("denm", std::regex_constants::icase));
}

/*!
 * brief Check if action is a trigger action.
 * \param action Action name
 * \return true if action is a trigger action, false otherwise
 */
bool EsminiAdapter::isTriggerAction(const std::string &action) {
  return std::regex_search(action, std::regex("trigger", std::regex_constants::icase));
}

/*!
 * \brief Add delayed start to object state if start action occurred.
 * \param name Name of the StoryBoardElement whose state has changed.
 * \param type Possible values: STORY = 1, ACT = 2, MANEUVER_GROUP = 3, MANEUVER = 4, EVENT = 5, ACTION = 6, UNDEFINED_ELEMENT_TYPE = 0.
 * \param state new state, possible values: STANDBY = 1, RUNNING = 2, COMPLETE = 3, UNDEFINED_ELEMENT_STATE = 0.
 */
void EsminiAdapter::collectStartAction(
    const char *name,
    int type,
    int state) {
  if (type != ElementType::ACTION || state != ElementState::RUNNING) {
    return;
  } // Only handle actions that are started
  try {
    auto [objectId, action] = parseAction(name);
    if (isStartAction(action)) {
      delayedStartIds.push_back(objectId);
    }
  } catch (std::exception &e) {
    RCLCPP_WARN(me->get_logger(), e.what());
    return;
  }
}

/*!
 * \brief Callback to be executed by esmini when story board state changes.
 * 		If story board element is an action, and the action is supported, the action is run.
 * \param name Name of the StoryBoardElement whose state has changed.
 * \param type The type of the element.
 * \param state The new state.
 */
void EsminiAdapter::handleStoryBoardElementChange(
    const char *name,
    int type,
    int state) {
  RCLCPP_DEBUG(me->get_logger(), "Storyboard state changed! Name: %s, Type: %d, State: %d", name, type, state);
  // switch on type
  switch (type) {
  case ElementType::ACTION:
    me->handleActionElementStateChange(name, static_cast<ElementState>(state));
    break;
  case ElementType::STORY: // Ignore all other types for now
  case ElementType::ACT:
  case ElementType::MANEUVER_GROUP:
  case ElementType::MANEUVER:
  case ElementType::EVENT:
    break;
  default:
    RCLCPP_INFO(me->get_logger(), "Type %d not recognised for element %s", type, name);
    break;
  }
}

void EsminiAdapter::handleActionElementStateChange(
    const char *name,
    ElementState state) {
  try {
    auto [objectId, action] = parseAction(name);
    if (isStartAction(action) && state == ElementState::RUNNING) {
      RCLCPP_INFO(me->get_logger(), "Running start action for object %d", objectId);
      ROSChannels::StartObject::message_type startObjectMsg;
      startObjectMsg.id = objectId;
      startObjectMsg.stamp = me->get_clock()->now(); // TODO + std::chrono::milliseconds(100);
      me->startObjectPub.publish(startObjectMsg);
    } else if (isSendDenmAction(action) && state == ElementState::COMPLETE) {
      // Get the latest Monitor message
      RCLCPP_INFO(me->get_logger(), "Running send DENM action triggered by object %d", objectId);
      ROSChannels::Monitor::message_type monr;
      rclcpp::wait_for_message(monr, me, std::string(me->get_namespace()) + "/object_" + std::to_string(objectId) + "/object_monitor", 10ms);
      double llh[3] = {me->testOrigin.position.latitude, me->testOrigin.position.longitude, me->testOrigin.position.altitude};
      double offset[3] = {monr.pose.pose.position.x, monr.pose.pose.position.y, monr.pose.pose.position.z};
      CRSTransformation::llhOffsetMeters(llh, offset);
      me->v2xPub.publish(denmFromTestOrigin(llh));
    } else if (isTriggerAction(action) && state == ElementState::COMPLETE) {
      RCLCPP_INFO(me->get_logger(), "Running trigger action for object %d", objectId);
      ROSChannels::TriggerEventOccurred::message_type triggerEventMsg;
      triggerEventMsg.trigger_id = objectId;
      me->triggerEventPub.publish(triggerEventMsg);
    } else {
      RCLCPP_DEBUG(me->get_logger(), "Action %s is not supported", action.c_str());
    }
  } catch (std::exception &e) {
    RCLCPP_WARN(me->get_logger(), e.what());
    return;
  }
}

ROSChannels::V2X::message_type EsminiAdapter::denmFromTestOrigin(double *llh) {
  ROSChannels::V2X::message_type denm;
  denm.message_type = "DENM";
  denm.event_id = "ATOSEvent1";
  denm.cause_code = 12;
  denm.latitude = static_cast<int32_t>(llh[0] * 10000000); // Microdegrees
  denm.longitude = static_cast<int32_t>(llh[1] * 10000000);
  denm.altitude = static_cast<int32_t>(llh[2] * 100);                     // Centimeters
  denm.detection_time = std::chrono::duration_cast<std::chrono::seconds>( // Time since epoch in seconds
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
  return denm;
}

/*!
 * \brief Utility function to convert a ROS MONR message to Esmini representation
 *			and report the object position to Esmini
 * \param monr ROS Monitor message of an object
 * \param id The object ID to which the monr belongs
 */
void EsminiAdapter::reportObjectPosition(const Monitor::message_type::SharedPtr monr, uint32_t esminiObjectId) {
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
  if (me->ATOStoEsminiObjectId.find(ATOSObjectId) != me->ATOStoEsminiObjectId.end()) {
    auto esminiObjectId = me->ATOStoEsminiObjectId[ATOSObjectId];
    reportObjectPosition(monr, esminiObjectId); // Report object position to esmini
    SE_Step();                                  // Advance the "simulation world"-time
  } else {
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
    std::vector<SE_ScenarioObjectState> &states) {
  ATOS::Trajectory trajectory(me->get_logger());
  trajectory.name = "Esmini Trajectory for object " + std::to_string(id);
  if (states.empty()) {
    return trajectory;
  }

  RCLCPP_DEBUG(me->get_logger(), "Creating trajectory for object %d", id);
  auto saveTp = [&](auto &state, const auto &prevState) {
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
    } else {
      tp.setLongitudinalAcceleration(0);
      tp.setLateralAcceleration(0);
    }

    trajectory.points.push_back(tp);
  };

  if (states.size() > 1) {
    for (auto it = states.begin() + 1; it != states.end(); ++it) {
      if (it->x == (it - 1)->x && it->y == (it - 1)->y && // Nothing interesting happens within 1 timestep, skip
          it->z == (it - 1)->z && it->h == (it - 1)->h) {
        continue;
      }
      saveTp(*it, *(it - 1)); // Next timestep is different, save current one.
    }
  }
  if (trajectory.points.size() == 0) {
    saveTp(states.front(), states.front()); // Only one state or no points, save it.
  }
  auto startTime = trajectory.points.front().getTime();

  // Subtract start time from all timesteps
  for (auto &tp : trajectory.points) {
    tp.setTime(tp.getTime() - startTime);
  }
  return trajectory;
}

/*!
 * \brief Given a RM_georeference converts into a proj string
 * \param geoRef The geo reference to convert
 * \return The proj string
 */
std::string EsminiAdapter::projStrFromGeoReference(RM_GeoReference &geoRef) {
  std::string projStringFrom = "+proj=";
  if (strlen(geoRef.proj_) != 0) {
    projStringFrom += std::string(geoRef.proj_) + " ";
  } else {
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
    std::map<uint32_t, std::vector<SE_ScenarioObjectState>> &states) {
  double accumTime = 0.0;
  auto pushCurrentState = [&](auto &st, auto &index) {
    auto id = SE_GetId(index);
    // SE_SetAlignModeZ(id, 0); // Disable Z-alignment, not implemented in esmini yet
    SE_ScenarioObjectState s;
    SE_GetObjectState(id, &s);
    s.timestamp = accumTime;
    st.at(std::stoi(SE_GetObjectName(id))).push_back(s);
  };

  // Populate States map with vector for each object containing the initial state
  SE_StepDT(timeStep);
  accumTime += timeStep;
  for (int j = 0; j < SE_GetNumberOfObjects(); j++) {
    states[std::stoi(SE_GetObjectName(SE_GetId(j)))] = std::vector<SE_ScenarioObjectState>();
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
    for (int j = 0; j < SE_GetNumberOfObjects(); j++) {
      pushCurrentState(states, j);
    }
    bool noMovement = std::all_of(states.begin(), states.end(), [&](auto &pair) {
      return pair.second.back().speed < 0.1;
    });
    bool atLeastMinTimePassed = accumTime > MIN_SCENARIO_TIME;
    bool moreThanMaxTimePassed = accumTime > MAX_SCENARIO_TIME;
    stopSimulation = (noMovement && atLeastMinTimePassed) || moreThanMaxTimePassed;
  }
  if (accumTime > MAX_SCENARIO_TIME) {
    RCLCPP_WARN(me->get_logger(), "Scenario time limit reached, stopping simulation");
  } else if (accumTime < MIN_SCENARIO_TIME + timeStep) {
    RCLCPP_WARN(me->get_logger(), "Ran scenario for the minimum time %.2f, possibly no movement in scenario", MIN_SCENARIO_TIME);
  }
  RCLCPP_INFO(me->get_logger(), "Finished %f s simulation", accumTime);
}

/*!
 * \brief Runs the esmini simulator with the xosc file and returns the trajectories for each object
 * \param timeStep Time step to use for generating the trajectories
 * \param endTime End time of the simulation TODO: not nessescary if xosc has a stop trigger at the end of the scenario
 * \param idToTraj The return map of ids mapping to the respective trajectories
 * \return A map of ids mapping to the respective trajectories
 */
std::map<uint32_t, ATOS::Trajectory> EsminiAdapter::extractTrajectories(
    double timeStep,
    std::map<uint32_t, ATOS::Trajectory> &idToTraj) {
  // Get object states
  std::map<uint32_t, std::vector<SE_ScenarioObjectState>> idToStates;
  getObjectStates(timeStep, idToStates);

  // Extract trajectories
  for (auto &os : idToStates) {
    auto id = os.first;
    auto objectStates = os.second;
    auto traj = getTrajectoryFromObjectState(id, objectStates);
    // Apply CRS transform if OpenDrive CRS Transformation is defined
    if (me->applyTrajTransform) {
      RCLCPP_DEBUG(me->get_logger(), "Applying CRS transformation to trajectory for object %d", id);
      me->crsTransformation->apply(traj.points);
    }
    idToTraj.insert(std::pair<uint32_t, ATOS::Trajectory>(id, traj));
  }
  return idToTraj;
}

/*!
 * \brief Initialize the esmini simulator and perform subsequent setup tasks.
 * Can be called many times, each time the test is initialized.
 */
void EsminiAdapter::InitializeEsmini() {
  me->delayedStartIds.clear();
  me->idToTraj.clear();
  me->ATOStoEsminiObjectId.clear();
  me->idToIp.clear();
  me->pathPublishers.clear();
  me->gnssPathPublishers.clear();
  auto logFilePath = std::string(getenv("HOME")) + std::string("/.astazero/ATOS/logs/esmini.log");
  SE_SetLogFilePath(logFilePath.c_str());
  SE_Close(); // Stop ScenarioEngine in case it is running
  RM_Close(); // Stop RoadManager in case it is running

  RCLCPP_INFO(me->get_logger(), "Initializing esmini with scenario file %s", me->oscFilePath.c_str());
  if (SE_Init(me->oscFilePath.c_str(), 1, 0, 0, 0) < 0) { // Disable controllers, let DefaultController be used
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
                                 " +datum=" + toDatum + " +units=m +no_defs";

      me->crsTransformation = std::make_shared<CRSTransformation>(projStringFrom, projStringTo);
      me->applyTrajTransform = true;
    } catch (std::exception &e) {
      RCLCPP_ERROR(me->get_logger(), e.what());
      return;
    }
  } else {
    RCLCPP_WARN(me->get_logger(), "Failed to get OpenDRIVE geo reference from RoadManager");
  }
  RM_Close();

  for (int j = 0; j < SE_GetNumberOfObjects(); j++) {
    // SE_SetAlignModeZ(SE_GetId(j), 0); // Disable Z-alignment not implemented in esmini yet
  }
  // Register callbacks to figure out what actions need to be taken
  SE_RegisterStoryBoardElementStateChangeCallback(&collectStartAction);

  RCLCPP_INFO(me->get_logger(), "Starting extracting trajs");
  double timeStep = 0.1;
  me->extractTrajectories(timeStep, me->idToTraj);
  RCLCPP_INFO(me->get_logger(), "Done extracting trajs");

  RCLCPP_INFO(me->get_logger(), "Extracted %ld trajectories", me->idToTraj.size());
  RCLCPP_INFO(me->get_logger(), "Number of objects with triggered start: %ld", me->delayedStartIds.size());

  // Find object IPs as defined in VehicleCatalog file
  for (int j = 0; j < SE_GetNumberOfObjects(); j++) {
    auto id = std::stoi(SE_GetObjectName(SE_GetId(j)));
    auto ip = SE_GetObjectPropertyValue(j, "ip");
    if (ip != nullptr) {
      me->idToIp[id] = std::string(ip);
    }
  }

  // Populate the map tracking Object ID -> esmini index
  for (int j = 0; j < SE_GetNumberOfObjects(); j++) {
    me->ATOStoEsminiObjectId[std::stoi(SE_GetObjectName(SE_GetId(j)))] = SE_GetId(j);
  }
  SE_Close(); // Stop ScenarioEngine

  RCLCPP_DEBUG(me->get_logger(), "Extracted trajectories");

  for (auto &it : me->idToTraj) {
    auto id = it.first;
    auto traj = it.second;

    RCLCPP_INFO(me->get_logger(), "Trajectory for object %d has %ld points", id, traj.points.size());

    // Publish the trajectory as a path
    me->pathPublishers.emplace(id, ROSChannels::Path::Pub(*me, id));
    me->pathPublishers.at(id).publish(traj.toPath());

    // below is for dumping the trajectory points to the console
    // for (auto& tp : traj.points){
    // 	RCLCPP_INFO(me->get_logger(), "Trajectory point: %lf, %lf, %lf, %lf, %ld", tp.getXCoord(), tp.getYCoord(), tp.getZCoord(), tp.getHeading(), tp.getTime().count());
    // }
  }
}

void EsminiAdapter::onRequestObjectTrajectory(
    const std::shared_ptr<ObjectTrajectorySrv::Request> req,
    std::shared_ptr<ObjectTrajectorySrv::Response> res) {
  res->id = req->id;
  try {
    res->trajectory = me->idToTraj.at(req->id).toCartesianTrajectory();
    res->success = true;
  } catch (std::out_of_range &e) {
    RCLCPP_ERROR(me->get_logger(), "Esmini trajectory service called, no trajectory found for object %d", req->id);
    res->success = false;
  }
}

void EsminiAdapter::onRequestTestOrigin(
    const std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Request>,
    std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Response> res) {
  while (me->testOriginSet == false) {
    RCLCPP_DEBUG(me->get_logger(), "Waiting for test origin to be available");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  res->origin.position.latitude = me->testOrigin.position.latitude;
  res->origin.position.longitude = me->testOrigin.position.longitude;
  res->origin.position.altitude = me->testOrigin.position.altitude;
}

void EsminiAdapter::onRequestObjectStartOnTrigger(
    const std::shared_ptr<ObjectTriggerSrv::Request> req,
    std::shared_ptr<ObjectTriggerSrv::Response> res) {
  res->id = req->id;
  res->success = true;
  res->trigger_start = std::count(me->delayedStartIds.begin(), me->delayedStartIds.end(), req->id) > 0;
  if (!res->trigger_start) {
    RCLCPP_INFO(me->get_logger(), "No triggers found for object %d", req->id);
  }
}

void EsminiAdapter::onRequestObjectIP(
    const std::shared_ptr<ObjectIpSrv::Request> req,
    std::shared_ptr<ObjectIpSrv::Response> res) {
  res->id = req->id;
  try {
    res->ip = me->idToIp.at(req->id);
    res->success = true;
  } catch (std::out_of_range &e) {
    RCLCPP_ERROR(me->get_logger(), "Esmini IP service called, no IP found for object %d", req->id);
    res->success = false;
  }
}

void EsminiAdapter::onSetObjectIP(
    const std::shared_ptr<SetObjectIpSrv::Request> req,
    std::shared_ptr<SetObjectIpSrv::Response> res) {
  res->id = req->id;
  res->ip = req->ip;
  try {
    me->idToIp.at(req->id) = req->ip;
    res->success = true;
  } catch (std::out_of_range &e) {
    RCLCPP_ERROR(me->get_logger(), "Esmini set IP service called, no object with ID %d found", req->id);
    res->success = false;
  }
}

/*!
 * \brief initializeModule Initializes this module by creating log,
 *			connecting to the message queue bus, setting up signal handers etc.
 * \param logLevel Level of the module log to be used.
 * \return 0 on success, -1 otherwise
 */
int EsminiAdapter::initializeModule() {
  int retval = 0;

  // Providing services
  me->startOnTriggerService = me->create_service<ObjectTriggerSrv>(ServiceNames::getObjectTriggerStart,
                                                                   std::bind(&EsminiAdapter::onRequestObjectStartOnTrigger, _1, _2));
  me->objectIpService = me->create_service<ObjectIpSrv>(ServiceNames::getObjectIp,
                                                        std::bind(&EsminiAdapter::onRequestObjectIP, _1, _2));
  me->objectTrajectoryService = me->create_service<ObjectTrajectorySrv>(ServiceNames::getObjectTrajectory,
                                                                        std::bind(&EsminiAdapter::onRequestObjectTrajectory, _1, _2));
  me->testOriginService = me->create_service<atos_interfaces::srv::GetTestOrigin>(ServiceNames::getTestOrigin,
                                                                                  std::bind(&EsminiAdapter::onRequestTestOrigin, _1, _2));
  me->setObjectIpService = me->create_service<atos_interfaces::srv::SetObjectIp>(SetObjectIpSrv::Request::SERVICE_NAME,
                                                                                 std::bind(&EsminiAdapter::onSetObjectIP, _1, _2));

  return retval;
}