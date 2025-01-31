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

#include "sml.hpp"

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
class ObjectControl;

namespace state_machine {

namespace events {
	struct ApplicationStarted{};
	struct Initialize{};
	struct Connect{};
	struct Disconnect{};
	struct DisconnectedFromObject{
		uint32_t id{};
	};
	struct Ready{};
	struct Reset{};
	struct Reload{};
	struct Arm{};
	struct Disarm{};
	struct TestLive{};
	struct RemoteControl{};
	struct Clear{};
	struct Abort{};
	struct Start{};
	struct StartObject {
		uint32_t id{};
		std::chrono::system_clock::time_point startTime{};
	};
	struct Done{};
}

static constexpr auto Off			= boost::sml::state<struct Off>;
static constexpr auto Idle			= boost::sml::state<struct Idle>;
static constexpr auto Initialized	= boost::sml::state<struct Initialized>;
static constexpr auto Connecting	= boost::sml::state<struct Connecting>;
static constexpr auto Disconnecting = boost::sml::state<struct Disconnecting>;
static constexpr auto Ready			= boost::sml::state<struct Ready>;
static constexpr auto Armed			= boost::sml::state<struct Armed>;
static constexpr auto Disarming		= boost::sml::state<struct Disarming>;
static constexpr auto TestLive		= boost::sml::state<struct TestLive>;
static constexpr auto Aborting		= boost::sml::state<struct Aborting>;
static constexpr auto Clearing		= boost::sml::state<struct Clearing>;
static constexpr auto Done			= boost::sml::state<struct Done>;
static constexpr auto RemoteControl = boost::sml::state<struct RemoteControl>;

struct idle_on_entry             { void operator()(ObjectControl* oc) const; };
struct idle_to_init_guard        { bool operator()(ObjectControl* oc) const; };
struct idle_to_init_action       { void operator()(ObjectControl* oc) const; };
struct init_to_connecting_action { void operator()(ObjectControl* oc) const; };
struct init_to_connecting_guard  { bool operator()(ObjectControl* oc) const; };
struct init_to_idle_action       { void operator()(ObjectControl* oc) const; };
struct connecting_on_entry       { void operator()(ObjectControl* oc) const; };
struct connecting_to_idle_action { void operator()(ObjectControl* oc) const; };
struct clearing_on_entry         { void operator()(ObjectControl* oc) const; };
struct ready_on_entry            { void operator()(ObjectControl* oc) const; };
struct ready_reset               { void operator()(ObjectControl* oc) const; };
struct ready_reload              { void operator()(ObjectControl* oc) const; };
struct remote_control_on_entry   { void operator()(ObjectControl* oc) const; };
struct remote_control_on_exit    { void operator()(ObjectControl* oc) const; };
struct disarming_on_entry        { void operator()(ObjectControl* oc) const; };
struct done_on_entry             { void operator()(ObjectControl* oc) const; };
struct armed_on_enter            { void operator()(ObjectControl* oc) const; };
struct armed_to_testlive_guard   { bool operator()(ObjectControl* oc) const; };
struct test_live_start_object    { void operator()(const events::StartObject& event, ObjectControl* oc) const; };
struct disconnected_from_object  { void operator()(const events::DisconnectedFromObject& event, ObjectControl* oc) const; };

template<class T>
OBCState_t asNumber() {
	if constexpr(std::is_same_v<typename T::type, decltype(Idle)::type>){
		return OBC_STATE_IDLE;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Initialized)::type>){
		return OBC_STATE_INITIALIZED;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Connecting)::type>){
		return OBC_STATE_CONNECTED;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Ready)::type>){
		return OBC_STATE_CONNECTED;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Armed)::type>){
		return OBC_STATE_ARMED;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Disarming)::type>){
		return OBC_STATE_DISARMING;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(TestLive)::type>){
		return OBC_STATE_RUNNING;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(RemoteControl)::type>){
		return OBC_STATE_REMOTECTRL;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Aborting)::type>){
		return OBC_STATE_ABORTING;
	}
	else if constexpr(std::is_same_v<typename T::type, decltype(Clearing)::type>){
		return OBC_STATE_CLEARING;
	}
	else {
		return OBC_STATE_UNDEFINED;
	}
}

class Logger {
public:
  Logger(rclcpp::Logger l, ROSChannels::StateChange::Pub p) : logger_{l}, stateChangePub_{p} {}

  template <class SM, class TEvent>
  void log_process_event(const TEvent&) {
	RCLCPP_INFO(logger_, "[%s][process_event] %s",
		boost::sml::aux::get_type_name<SM>(),
		boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard&, const TEvent&, bool result) {
	RCLCPP_INFO(logger_, "[%s][guard] %s %s %s",
		boost::sml::aux::get_type_name<SM>(),
		boost::sml::aux::get_type_name<TGuard>(),
		boost::sml::aux::get_type_name<TEvent>(),
		(result ? "[OK]" : "[Reject]"));
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction&, const TEvent&) {
    RCLCPP_INFO(logger_, "[%s][action] %s %s",
		boost::sml::aux::get_type_name<SM>(),
		boost::sml::aux::get_type_name<TAction>(),
		boost::sml::aux::get_type_name<TEvent>());
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState& src, const TDstState& dst) {
	RCLCPP_INFO(logger_, "[%s][transition] %s -> %s",
		boost::sml::aux::get_type_name<SM>(),
		src.c_str(),
		dst.c_str());

	atos_interfaces::msg::StateChange stateChangeMsg = atos_interfaces::msg::StateChange();
	stateChangeMsg.prev_state = asNumber<TSrcState>();
	stateChangeMsg.current_state = asNumber<TDstState>();
	stateChangePub_.publish(stateChangeMsg);
  }

private:
  rclcpp::Logger logger_;
  ROSChannels::StateChange::Pub stateChangePub_;
};

class StateMachine
{
  public:
	// Transition table
	auto operator()() const noexcept
	{
		namespace sml = boost::sml;
		return sml::make_transition_table(
			// *Off + sml::event<events::ApplicationStarted> = Idle,

			*Idle + sml::on_entry<sml::_> / idle_on_entry(),
			Idle + sml::event<events::Initialize>[idle_to_init_guard()] / idle_to_init_action() = Initialized,

			Initialized + sml::event<events::Disconnect> / init_to_idle_action()  = Idle,
			Initialized + sml::event<events::Connect>[init_to_connecting_guard()] / init_to_connecting_action() = Connecting,

			Connecting + sml::on_entry<sml::_> / connecting_on_entry(),
			Connecting + sml::event<events::Abort> = Aborting,
			Connecting + sml::event<events::Disarm> = Disarming,
			Connecting + sml::event<events::Disconnect> / connecting_to_idle_action() = Idle,
			Connecting + sml::event<events::Ready> = Ready,

			Ready + sml::on_entry<sml::_> / ready_on_entry(),
			Ready + sml::event<events::Abort>                  = Aborting,
			Ready + sml::event<events::Arm>                    = Armed,
			Ready + sml::event<events::Disconnect>             = Idle,
			Ready + sml::event<events::DisconnectedFromObject> / disconnected_from_object() = Connecting,
			Ready + sml::event<events::RemoteControl>          = RemoteControl,
			Ready + sml::event<events::Reset> / ready_reset(),
			Ready + sml::event<events::Reload> / ready_reload(),

			Armed + sml::on_entry<sml::_> / armed_on_enter(),
			Armed + sml::event<events::Start>[armed_to_testlive_guard()] = TestLive,
			Armed + sml::event<events::Disarm>   = Disarming,

			Disarming + sml::on_entry<sml::_> / disarming_on_entry(),
			Disarming + sml::event<events::Abort>      = Aborting,
			Disarming + sml::event<events::Connect>    = Connecting,
			Disarming + sml::event<events::Disconnect> = Idle,
			Disarming + sml::event<events::Ready>      = Ready,

			TestLive + sml::event<events::Abort> = Aborting,
			TestLive + sml::event<events::Done>  = Done,
			TestLive + sml::event<events::StartObject> / test_live_start_object(),

			Done + sml::on_entry<sml::_> / done_on_entry(),
			Done + sml::event<events::Ready> = Ready,

			Aborting + sml::event<events::Clear> = Clearing,

			Clearing + sml::on_entry<sml::_> / clearing_on_entry(),
			Clearing + sml::event<events::Ready> = Ready,

			RemoteControl + sml::on_entry<sml::_> / remote_control_on_entry(),
			RemoteControl + sml::on_exit<sml::_> / remote_control_on_exit(),
			RemoteControl + sml::event<events::Ready> = Ready);
	}
};

}

enum class ControlMode : int {
	AbsoluteKinematics,
	RelativeKinematics,
};


/*!
 * \brief The ObjectControl class is intended as an overarching device
 *			used to control a scenario. No behaviour is implemented in it
 *			(this is left up to the State to determine), only functionality
 *			which can be called.
 */
class ObjectControl : public Module
{
	friend class ObjectControlState;
	friend class ObjectListener;

public:
	ObjectControl(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>);

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

	void setControlMode(ControlMode cm)	{ controlMode = cm; }

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

	ControlMode controlMode{ControlMode::AbsoluteKinematics};


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
public:
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

	void publishScenarioInfoToJournal();

	state_machine::Logger sm_logger{get_logger(), stateChangePub};
	boost::sml::sm<state_machine::StateMachine,
					boost::sml::logger<state_machine::Logger>,
					boost::sml::thread_safe<std::recursive_mutex>> sm{this, sm_logger};

	OBCState_t stateAsNumber() {
		if (sm.is(state_machine::Idle)) {
			return OBC_STATE_IDLE;
		}
		else if (sm.is(state_machine::Initialized)) {
			return OBC_STATE_INITIALIZED;
		}
		else if (sm.is(state_machine::Connecting)) {
			return OBC_STATE_CONNECTED;
		}
		else if (sm.is(state_machine::Ready)) {
			return OBC_STATE_CONNECTED;
		}
		else if (sm.is(state_machine::Armed)) {
			return OBC_STATE_ARMED;
		}
		else if (sm.is(state_machine::Disarming)) {
			return OBC_STATE_DISARMING;
		}
		else if (sm.is(state_machine::TestLive)) {
			return OBC_STATE_RUNNING;
		}
		else if (sm.is(state_machine::RemoteControl)) {
			return OBC_STATE_REMOTECTRL;
		}
		else if (sm.is(state_machine::Aborting)) {
			return OBC_STATE_ABORTING;
		}
		else if (sm.is(state_machine::Clearing)) {
			return OBC_STATE_CLEARING;
		}
		else {
			return OBC_STATE_UNDEFINED;
		}
	}

	ControlCenterStatusType controlCenterStatus() {
		if (sm.is(state_machine::Idle)) {
			return CONTROL_CENTER_STATUS_INIT;
		}
		else if (sm.is(state_machine::Initialized)) {
			return CONTROL_CENTER_STATUS_INIT;
		}
		else if (sm.is(state_machine::Connecting)) {
			return CONTROL_CENTER_STATUS_INIT;
		}
		else if (sm.is(state_machine::Ready)) {
			return CONTROL_CENTER_STATUS_READY;
		}
		else if (sm.is(state_machine::Armed)) {
			return CONTROL_CENTER_STATUS_RUNNING; // TODO
		}
		else if (sm.is(state_machine::Disarming)) {
			return CONTROL_CENTER_STATUS_RUNNING; // TODO
		}
		else if (sm.is(state_machine::TestLive)) {
			return CONTROL_CENTER_STATUS_RUNNING;
		}
		else if (sm.is(state_machine::RemoteControl)) {
			return CONTROL_CENTER_STATUS_READY;
		}
		else if (sm.is(state_machine::Aborting)) {
			return CONTROL_CENTER_STATUS_ABORT;
		}
		else if (sm.is(state_machine::Clearing)) {
			return CONTROL_CENTER_STATUS_READY;
		}
		else if (sm.is(state_machine::Done)){
			return CONTROL_CENTER_STATUS_TEST_DONE;
		}
		else {
			return CONTROL_CENTER_STATUS_ABORT;
		}
	}

};
