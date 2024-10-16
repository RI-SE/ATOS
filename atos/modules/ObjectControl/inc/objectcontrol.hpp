/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <map>
#include <future>
#include <set>
#include <chrono>
#include <mutex>
#include <memory>
#include <unordered_map>

#include "geographic_msgs/msg/geo_point.hpp"

#include "module.hpp"
#include "atosTime.h"
#include "testobject.hpp"
#include "objectlistener.hpp"
#include "roschannels/commandchannels.hpp"
#include "roschannels/monitorchannel.hpp"
#include "roschannels/remotecontrolchannels.hpp"
#include "roschannels/pathchannel.hpp"
#include "roschannels/gnsspathchannel.hpp"
#include "roschannels/controlsignalchannel.hpp"
#include "roschannels/objstatechangechannel.hpp"
#include "roschannels/statechange.hpp"
#include "atos_interfaces/srv/get_object_ids.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"
#include "atos_interfaces/srv/get_object_ip.hpp"
#include "atos_interfaces/srv/get_object_trigger_start.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "atos_interfaces/srv/get_object_control_state.hpp"
#include "atos_interfaces/srv/get_object_return_trajectory.hpp"

// Forward declarations
class ObjectControlState;
class ObjectListener;

namespace AbstractKinematics {
	class Idle;
	class Initialized;
	class Connecting;
	class Ready;
	class Aborting;
	class Clearing;
	class Armed;
	class TestLive;
	class Disarming;
	class Done;
	class RemoteControlled;
}

namespace RelativeKinematics {
	class Initialized;
	class Connecting;
	class Ready;
	class Aborting;
	class Clearing;
	class Armed;
	class TestLive;
	class Disarming;
	class Done;
	class RemoteControlled;
}

namespace AbsoluteKinematics {
	class Initialized;
	class Connecting;
	class Ready;
	class Aborting;
	class Clearing;
	class Armed;
	class TestLive;
	class Disarming;
	class Done;
	class RemoteControlled;
}

class SmImpl;

/*!
 * \brief The ObjectControl class is intended as an overarching device
 *			used to control a scenario. No behaviour is implemented in it
 *			(this is left up to the State to determine), only functionality
 *			which can be called.
 */
class ObjectControl : public Module
{
	// Forward declare state machine
	class Sm;
	friend class SmImpl;
	friend class ObjectControlState;
	friend class AbstractKinematics::Idle;
	friend class AbstractKinematics::Initialized;
	friend class AbstractKinematics::Connecting;
	friend class AbstractKinematics::Ready;
	friend class AbstractKinematics::Aborting;
	friend class AbstractKinematics::Clearing;
	friend class AbstractKinematics::Armed;
	friend class AbstractKinematics::TestLive;
	friend class AbstractKinematics::Disarming;
	friend class AbstractKinematics::Done;
	friend class AbstractKinematics::RemoteControlled;
	friend class RelativeKinematics::Initialized;
	friend class RelativeKinematics::Connecting;
	friend class RelativeKinematics::Ready;
	friend class RelativeKinematics::Aborting;
	friend class RelativeKinematics::Clearing;
	friend class RelativeKinematics::Armed;
	friend class RelativeKinematics::TestLive;
	friend class RelativeKinematics::Disarming;
	friend class RelativeKinematics::Done;
	friend class RelativeKinematics::RemoteControlled;
	friend class AbsoluteKinematics::Initialized;
	friend class AbsoluteKinematics::Connecting;
	friend class AbsoluteKinematics::Ready;
	friend class AbsoluteKinematics::Aborting;
	friend class AbsoluteKinematics::Clearing;
	friend class AbsoluteKinematics::Armed;
	friend class AbsoluteKinematics::TestLive;
	friend class AbsoluteKinematics::Disarming;
	friend class AbsoluteKinematics::Done;
	friend class AbsoluteKinematics::RemoteControlled;

	friend class ObjectListener;

public:
	ObjectControl(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>);
	typedef enum {
		RELATIVE_KINEMATICS,	//!< Scenario executed relative to immobile VUT
		ABSOLUTE_KINEMATICS		//!< Scenario executed relative to earth-fixed point
	} ControlMode;


	typedef struct {
		unsigned int numberOfTargets;
		uint32_t *targetIDs;
		bool isActive;
	} DataInjectionMap;

	typedef struct {
		uint16_t actionID;
		uint32_t objectID;
		ActionTypeParameter_t command;
	} TestScenarioCommandAction;

	~ObjectControl();

	//! Handlers for user commands
	//! \brief Performs actions in response to an initialization request.
	void handleInitCommand();
	//! \brief Performs actions in response to a connect request.
	void handleConnectCommand();
	//! \brief Performs actions in response to a disconnect request.
	void handleDisconnectCommand();
	//! \brief Performs actions in response to an arm request.
	void handleArmCommand();
	//! \brief Performs actions in response to a start request.
	void handleStartCommand();
	//! \brief Performs actions in response to an abort request.
	void handleStopCommand();
	//! \brief Performs actions in response to an abort request.
	void handleAbortCommand();
	//! \brief Performs actions in response to an all clear request.
	void handleAllClearCommand();
	//! \brief Performs actions in response to an action configuration request.
	void handleActionConfigurationCommand(const TestScenarioCommandAction&);
	//! \brief Performs actions in response to an action execution request.
	void handleExecuteActionCommand(const uint16_t& actionID, const std::chrono::system_clock::time_point& when);
	void sendAbortNotification();

	//! Getters
	//! \brief Get transmitter ID of anchor object participating in test.
	uint32_t getAnchorObjectID() const;
	//! \brief Get last reported data by anchor object
	ObjectMonitorType getLastAnchorData() const;
	//! \brief Get transmitter IDs of all test participants.
	std::vector<uint32_t> getVehicleIDs() const {
		std::vector<uint32_t> retval;
		for (auto it  = objects.begin(); it != objects.end(); ++it) {
			retval.push_back(it->first);
		}
		return retval;
	}

	[[deprecated("Avoid referring to objects by IP")]]
	uint32_t getVehicleIDByIP(const in_addr_t& ip) {
		auto res = std::find_if(objects.begin(), objects.end(), [&](const std::pair<const uint32_t,std::shared_ptr<TestObject>>& elem){
			return elem.second->getObjectConfig().getIP() == ip;
		});
		return res->first;
	}

	//! \brief Get last known ISO state of test participants.
	std::map<uint32_t,ObjectStateType> getObjectStates() const;

	//! \brief Chneck if any object fulfill a predicate.
	bool isAnyObject(std::function<bool(const std::shared_ptr<TestObject>)> predicate) const;
	//! \brief Check if all objects fulfill a predicate.
	bool areAllObjects(std::function<bool(const std::shared_ptr<TestObject>)> predicate) const;
	//! \brief Check if any test participant is in the specified state.
	//!			The method does not wait for the next MONR to arrive.
	bool isAnyObjectIn(const ObjectStateType state);
	//! \brief Check if any test participant is in any of the specified states.
	//!			The method does not wait for the next MONR to arrive.
	bool isAnyObjectIn(const std::set<ObjectStateType>& state);
	//! \brief Checks if all test participants are in the specified state.
	//!			The method does not wait for the next MONR to arrive.
	bool areAllObjectsIn(const ObjectStateType state);
	//! \brief Checks if all test participants are in any of the specified states.
	//!			The method does not wait for the next MONR to arrive.
	bool areAllObjectsIn(const std::set<ObjectStateType>& state);

	//! Setters
	void startControlSignalSubscriber();
	void stopControlSignalSubscriber();

private:
	bool isResetting;
	geographic_msgs::msg::GeoPoint origin_pos; //!< Test origin
	std::mutex stateMutex;
	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;
	static inline std::string const moduleName = "object_control";
	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onConnectMessage(const ROSChannels::Connect::message_type::SharedPtr) override;
	void onArmMessage(const ROSChannels::Arm::message_type::SharedPtr) override;
	void onDisarmMessage(const ROSChannels::Arm::message_type::SharedPtr) override;
	void onStartMessage(const ROSChannels::Start::message_type::SharedPtr) override;
	void onStartObjectMessage(const ROSChannels::StartObject::message_type::SharedPtr) override;
	void onDisconnectMessage(const ROSChannels::Disconnect::message_type::SharedPtr) override;
	void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) override;
	void onResetTestObjectsMessage(const ROSChannels::ResetTestObjects::message_type::SharedPtr) override;
	void onReloadObjectSettingsMessage(const ROSChannels::ReloadObjectSettings::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;
	void onRemoteControlEnableMessage(const ROSChannels::RemoteControlEnable::message_type::SharedPtr);
	void onRemoteControlDisableMessage(const ROSChannels::RemoteControlDisable::message_type::SharedPtr);
	void onObjectStateChangeMessage(const ROSChannels::ObjectStateChange::message_type::SharedPtr);
	void onControlSignalMessage(const ROSChannels::ControlSignal::message_type::SharedPtr);
	void onPathMessage(const ROSChannels::Path::message_type::SharedPtr,const uint32_t);
	void onRequestState(const std::shared_ptr<atos_interfaces::srv::GetObjectControlState::Request>,
							 std::shared_ptr<atos_interfaces::srv::GetObjectControlState::Response>);

	using clock = std::chrono::steady_clock;

	ControlMode controlMode;
	ObjectControlState* state;					//!< State of module
	std::map<uint32_t,std::shared_ptr<TestObject>> objects;		//!< List of configured test participants
	std::map<uint32_t,ObjectListener> objectListeners;
	std::map<uint16_t,std::function<void()>> storedActions;
	std::mutex monitorTimeMutex;
	static constexpr auto heartbeatPeriod = std::chrono::milliseconds(1000 / HEAB_FREQUENCY_HZ);
	std::thread safetyThread;
	std::promise<void> stopHeartbeatSignal;

	std::shared_future<void> connStopReqFuture;	//!< Request to stop a connection attempt
	std::promise<void> connStopReqPromise;		//!< Promise that the above value will be emitted
	atos_interfaces::srv::GetObjectTrajectory::Response::SharedPtr trajResponse;
	atos_interfaces::srv::GetObjectReturnTrajectory::Response::SharedPtr returnTrajResponse;

	ROSChannels::Init::Sub scnInitSub;			//!< Subscriber to scenario initialization requests
	ROSChannels::Start::Sub scnStartSub;		//!< Subscriber to scenario start requests
	ROSChannels::StartObject::Sub objectStartSub;	//!< Subscriber to scenario start requests
	ROSChannels::Arm::Sub scnArmSub;			//!< Subscriber to scenario arm requests
	ROSChannels::Disarm::Sub scnDisarmSub;			//!< Subscriber to scenario arm requests
	ROSChannels::Stop::Sub scnStopSub;			//!< Subscriber to scenario stop requests
	ROSChannels::Abort::Sub scnAbortSub;		//!< Subscriber to scenario abort requests
	ROSChannels::AllClear::Sub scnAllClearSub;	//!< Subscriber to scenario all clear requests
	ROSChannels::Connect::Sub scnConnectSub;	//!< Subscriber to scenario connect requests
	ROSChannels::Disconnect::Sub scnDisconnectSub;	//!< Subscriber to scenario disconnect requests
	ROSChannels::RemoteControlEnable::Sub scnRemoteControlEnableSub;		//!< Subscriber to remote control enable requests
	ROSChannels::RemoteControlDisable::Sub scnRemoteControlDisableSub;	//!< Subscriber to remote control disable requests
	ROSChannels::GetStatus::Sub getStatusSub;				//!< Subscriber to scenario get status requests
	ROSChannels::ObjectStateChange::Sub objectStateChangeSub;	//!< Subscriber to object state changes
	std::shared_ptr<ROSChannels::ControlSignal::Sub> controlSignalSub;	//!< Pointer to subscriber to receive control signal messages with percentage
	ROSChannels::ResetTestObjects::Sub scnResetTestObjectsSub;	//!< Subscriber to scenario reset test requests
	ROSChannels::ReloadObjectSettings::Sub scnReloadObjectSettingsSub;	//!< Subscriber to scenario reset test requests

	rclcpp::TimerBase::SharedPtr objectsConnectedTimer;	//!< Timer to periodically publish connected objects

	ROSChannels::Failure::Pub failurePub;					//!< Publisher to scenario failure reports
	ROSChannels::Abort::Pub scnAbortPub;					//!< Publisher to scenario abort reports
	ROSChannels::ObjectsConnected::Pub objectsConnectedPub;	//!< Publisher to report that objects have been connected
	ROSChannels::ConnectedObjectIds::Pub connectedObjectIdsPub;	//!< Publisher to periodically report connected object ids
	ROSChannels::StateChange::Pub stateChangePub;			//!< Publisher to report state changes
	std::unordered_map<uint32_t,ROSChannels::Path::Pub> pathPublishers;
	std::unordered_map<uint32_t,ROSChannels::GNSSPath::Pub> gnssPathPublishers;

	rclcpp::CallbackGroup::SharedPtr id_client_cb_group_;
	rclcpp::CallbackGroup::SharedPtr traj_client_cb_group_;
	rclcpp::CallbackGroup::SharedPtr ip_client_cb_group_;
	rclcpp::CallbackGroup::SharedPtr origin_client_cb_group_;

	rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedPtr idClient;	//!< Client to request object ids
	rclcpp::Client<atos_interfaces::srv::GetTestOrigin>::SharedPtr originClient;	//!< Client to request object status
	rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedPtr trajectoryClient;	//!< Client to request object trajectories
	rclcpp::Client<atos_interfaces::srv::GetObjectIp>::SharedPtr ipClient;	//!< Client to request object IPs
	rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr returnTrajectoryClient;	//!< Client to request object return trajectory
	rclcpp::Service<atos_interfaces::srv::GetObjectControlState>::SharedPtr stateService;	//!< Service to request object control state
	//! Connection methods
	//! \brief Initiate a thread-based connection attempt. Threads are detached after start,
	//!			and can be terminated by calling ::abortConnectionAttempt or setting ::connStopReqFuture.
	void beginConnectionAttempt();
	//! \brief Abort all ongoing connection threads.
	void abortConnectionAttempt();
	//! \brief Abort ongoing connection attempts and disconnect objects.
	void disconnectObjects();

	//! \brief Disconnect specific object.
	void disconnectObject(const uint32_t id);

	//! \brief Establishe a connection to a specified object, and check the first
	//!			MONR state. This is a blocking method.
	void connectToObject(std::shared_ptr<TestObject> obj, std::shared_future<void>& connStopReq);

	void startListeners();
	void notifyObjectsConnected();
	void publishObjectIds();

	void startSafetyThread();
	void heartbeat();

	//! Configuration methods
	//! \brief Read the configured object and trajectory files and load related data
	//!			into the ScenarioHandler.
	bool loadScenario();
	//! \brief Read all object files and fill the list of TestObjects.
	void loadObjectFiles();
	//! \brief Transform the scenario trajectories relative to the trajectory of the
	//!			specified object.
	void transformScenarioRelativeTo(const uint32_t objectID);
	//! \brief Upload the configuration in the ScenarioHandler to all connected objects.
	void uploadAllConfigurations();
	//! \brief Upload the configuration in the ScenarioHandler to the connected obj	std::unique_ptr<ScenarioHandler> scenarioHandler;
	void uploadObjectConfiguration(const uint32_t id);
	//! \brief Clear loaded data and object list.
	void clearScenario();

	//! \brief TODO
	void armObjects();
	//! \brief TODO
	void disarmObjects();
	//! \brief
	void startScenario();
	//! \brief Resets the test by offering a back to start trajectory. Still needs arm and start commands to execute the reset.
	void resetTestObjects();
	//! \brief Reloads the scenario trajectories for each object.
	void reloadScenarioTrajectories();
	//! \brief Updates the paths in the GUI to reflect the new trajectories.
	void republishTrajectoryPaths(uint32_t id);
	//! \brief Callback for the trajectory request. Sends the new trajectory to the object.
	void trajectoryCallback(const rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedFuture future);
	//! \brief Callback for the return trajectory request. Sends the new trajectory to the object.
	void returnTrajectoryCallback(const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future);
	//! \brief Requests a new trajectory and sends it to the object.
	void setObjectTrajectory(uint32_t id);
	//! \brief
	void startObject(uint32_t id, std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now());
	//! \brief
	void allClearObjects();
	//! \brief TODO
	void remoteControlObjects(bool on);
	//! \brief TODO
	void injectObjectData(const MonitorMessage& monr);
	//! \brief TODO
	OsiHandler::LocalObjectGroundTruth_t buildOSILocalGroundTruth(const MonitorMessage&) const;
	std::unique_ptr<Sm> sm;
	void publishScenarioInfoToJournal();
};

