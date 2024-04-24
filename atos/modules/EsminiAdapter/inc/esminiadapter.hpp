/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "CRSTransformation.hpp"
#include "esmini/esminiLib.hpp"
#include "esmini/esminiRMLib.hpp"
#include "module.hpp"
#include "roschannels/commandchannels.hpp"
#include "roschannels/gnsspathchannel.hpp"
#include "roschannels/monitorchannel.hpp"
#include "roschannels/pathchannel.hpp"
#include "roschannels/statechange.hpp"
#include "roschannels/v2xchannel.hpp"
#include <filesystem>
#include <unordered_map>

#include "atos_interfaces/srv/get_object_ip.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"
#include "atos_interfaces/srv/get_object_trigger_start.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "atos_interfaces/srv/set_object_ip.hpp"
#include "trajectory.hpp"

enum ElementType { STORY = 1,
                   ACT = 2,
                   MANEUVER_GROUP = 3,
                   MANEUVER = 4,
                   EVENT = 5,
                   ACTION = 6,
                   UNDEFINED_ELEMENT_TYPE = 0 };
enum ElementState { STANDBY = 1,
                    RUNNING = 2,
                    COMPLETE = 3,
                    UNDEFINED_ELEMENT_STATE = 0 };

/*!
 * \brief The EsminiAdapter class is a singleton class that
 * 	handles the communication between the esmini simulator and ATOS
 */
class EsminiAdapter : public Module {
public:
  static inline std::string const moduleName = "esmini_adapter";
  static inline std::filesystem::path oscFilePath;
  static int initializeModule();
  EsminiAdapter(EsminiAdapter const &) = delete;
  EsminiAdapter &operator=(EsminiAdapter const &) = delete;
  static std::shared_ptr<EsminiAdapter> instance();

private:
  EsminiAdapter();

  ROSChannels::StartObject::Pub startObjectPub;
  ROSChannels::V2X::Pub v2xPub;
  ROSChannels::ConnectedObjectIds::Sub connectedObjectIdsSub;
  ROSChannels::Exit::Sub exitSub;
  ROSChannels::StateChange::Sub stateChangeSub;
  std::unordered_map<uint32_t, ROSChannels::Path::Pub> pathPublishers;
  std::unordered_map<uint32_t, ROSChannels::GNSSPath::Pub> gnssPathPublishers;

  static std::unordered_map<uint32_t, std::shared_ptr<ROSChannels::Monitor::Sub>> monrSubscribers;
  static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectTrajectory>> objectTrajectoryService;
  static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectTriggerStart>> startOnTriggerService;
  static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectIp>> objectIpService;
  static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetTestOrigin>> testOriginService;
  static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::SetObjectIp>> setObjectIpService;

  void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id);
  // Below is a quickfix, fix properly later
  static void handleInitCommand();
  static void handleStartCommand();
  static void handleAbortCommand();
  static void onStaticExitMessage(const ROSChannels::Exit::message_type::SharedPtr);
  static void onStaticStateChangeMessage(const ROSChannels::StateChange::message_type::SharedPtr);

  static void onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg);
  static void reportObjectPosition(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id);
  static std::filesystem::path getOpenScenarioFileParameter();
  static std::filesystem::path getOpenDriveFile();
  static void setOpenScenarioFile(const std::filesystem::path &);
  static void handleStoryBoardElementChange(const char *name, ElementType type, ElementState state);
  static void handleActionElementStateChange(const char *name, ElementState state);
  static void InitializeEsmini();
  static void getObjectStates(double timeStep, std::map<uint32_t, std::vector<SE_ScenarioObjectState>> &states);
  static ATOS::Trajectory getTrajectoryFromObjectState(uint32_t, std::vector<SE_ScenarioObjectState> &states);
  static std::string projStrFromGeoReference(RM_GeoReference &geoRef);
  static std::map<uint32_t, ATOS::Trajectory> extractTrajectories(double timeStep, std::map<uint32_t, ATOS::Trajectory> &idToTraj);
  static std::pair<uint32_t, std::string> parseAction(const std::string &action);
  static bool isStartAction(const std::string &action);
  static bool isSendDenmAction(const std::string &action);
  static void collectStartAction(const char *name, int type, int state);
  static ROSChannels::V2X::message_type denmFromTestOrigin(double *llh);

  static void onRequestObjectTrajectory(
      const std::shared_ptr<atos_interfaces::srv::GetObjectTrajectory::Request> req,
      std::shared_ptr<atos_interfaces::srv::GetObjectTrajectory::Response> res);

  static void onRequestObjectStartOnTrigger(
      const std::shared_ptr<atos_interfaces::srv::GetObjectTriggerStart::Request> req,
      std::shared_ptr<atos_interfaces::srv::GetObjectTriggerStart::Response> res);

  static void onRequestObjectIP(
      const std::shared_ptr<atos_interfaces::srv::GetObjectIp::Request> req,
      std::shared_ptr<atos_interfaces::srv::GetObjectIp::Response> res);

  static void onRequestTestOrigin(const std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Request>,
                                  std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Response>);

  static void onSetObjectIP(const std::shared_ptr<atos_interfaces::srv::SetObjectIp::Request>,
                            std::shared_ptr<atos_interfaces::srv::SetObjectIp::Response>);

  static std::shared_ptr<rclcpp::Client<atos_interfaces::srv::GetTestOrigin>> testOriginClient;
  static std::shared_ptr<EsminiAdapter> me;
  static std::unordered_map<int, int> ATOStoEsminiObjectId;
  static std::map<uint32_t, ATOS::Trajectory> idToTraj;
  static std::map<uint32_t, std::string> idToIp;
  static std::vector<uint32_t> delayedStartIds;

  std::shared_ptr<CRSTransformation> crsTransformation;
  bool applyTrajTransform;
  bool testOriginSet;

  static geographic_msgs::msg::GeoPose testOrigin;
};
