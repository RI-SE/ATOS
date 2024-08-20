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
#include "roschannels/scenariochannel.hpp"
#include "roschannels/statechange.hpp"
#include <filesystem>
#include <unordered_map>

#include "atos_interfaces/srv/get_object_ids.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"
#include "atos_interfaces/srv/get_object_trigger_start.hpp"
#include "atos_interfaces/srv/get_open_scenario_file_path.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "trajectory.hpp"

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

	ROSChannels::StoryBoardElementStateChange::Pub
		storyBoardElementStateChangePub;
	ROSChannels::ConnectedObjectIds::Sub connectedObjectIdsSub;
	ROSChannels::Exit::Sub exitSub;
	ROSChannels::StateChange::Sub stateChangeSub;
	std::unordered_map<uint32_t, ROSChannels::Path::Pub> pathPublishers;
	std::unordered_map<uint32_t, ROSChannels::GNSSPath::Pub> gnssPathPublishers;

	static std::unordered_map<uint32_t,std::shared_ptr<ROSChannels::Monitor::Sub>> monrSubscribers;
	static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetObjectTrajectory>> objectTrajectoryService;
	static std::shared_ptr<rclcpp::Service<atos_interfaces::srv::GetTestOrigin>> testOriginService;

	rclcpp::CallbackGroup::SharedPtr oscFilePathClient_cb_group_;
	rclcpp::CallbackGroup::SharedPtr objectIdsClient_cb_group_;

	rclcpp::Client<atos_interfaces::srv::GetOpenScenarioFilePath>::SharedPtr oscFilePathClient_;	//!< Client to request the current open scenario file path
	rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedPtr objectIdsClient_;	//!< Client to request the ATOS object id for each openx entity name

	void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id);
	// Below is a quickfix, fix properly later
	static void fetchOSCFilePath();
	static void handleStartCommand();
	static void handleAbortCommand();
	static void onStaticExitMessage(const ROSChannels::Exit::message_type::SharedPtr);
	static void onStaticStateChangeMessage(const ROSChannels::StateChange::message_type::SharedPtr);

	static void onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg);
	static void reportObjectPosition(const ROSChannels::Monitor::message_type::SharedPtr monr, uint32_t id);
	static void executeActionIfStarted(const char* name, int type, int state);
	static std::filesystem::path getOpenDriveFile();
	static void handleStoryBoardElementChange(const char* name, int type, int state, const char* full_path);
	static void runEsminiSimulation();
	static void getObjectStates(double timeStep, std::map<uint32_t,std::vector<SE_ScenarioObjectState>>& states);
	static ATOS::Trajectory getTrajectoryFromObjectState(uint32_t,std::vector<SE_ScenarioObjectState>& states);
	static std::string projStrFromGeoReference(RM_GeoReference& geoRef);
	static std::map<uint32_t, ATOS::Trajectory> extractTrajectories(double timeStep);

	static void onRequestObjectTrajectory(
		const std::shared_ptr<atos_interfaces::srv::GetObjectTrajectory::Request>
			req,
		std::shared_ptr<atos_interfaces::srv::GetObjectTrajectory::Response> res);

	static void onRequestTestOrigin(
		const std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Request>,
		std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Response>);

	static std::unordered_map<int, std::string> atosIDToObjectName;
	static std::unordered_map<std::string, int> objectNameToAtosId;
	static std::unordered_map<int, int> atosIdToEsminiId;
	static std::map<uint32_t, ATOS::Trajectory> atosObjectIdToTraj;
	static std::shared_ptr<EsminiAdapter> me;

	std::string scenarioFileMd5hash;
	bool runSimulation;

	std::shared_ptr<CRSTransformation> crsTransformation;
	bool applyTrajTransform;
	bool testOriginSet;

	static geographic_msgs::msg::GeoPose testOrigin;
	rclcpp::TimerBase::SharedPtr timer_;
};
