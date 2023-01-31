#pragma once
#include <map>
#include <future>
#include <set>
#include <chrono>

#include "module.hpp"
#include "maestroTime.h"
#include "state.hpp"
#include "testobject.hpp"
#include "objectlistener.hpp"
#include "atos_interfaces/srv/get_object_ids.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"
#include "atos_interfaces/srv/get_object_ip.hpp"
#include "atos_interfaces/srv/get_object_trigger_start.hpp"

// Forward declarations
class ObjectControlState;
class ObjectListener;

namespace AbstractKinematics {
	class Idle;
	class Initialized;
	class Connecting;
	class Ready;
	class Aborting;
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
	class Armed;
	class TestLive;
	class Disarming;
	class Done;
	class RemoteControlled;
}

/*!
 * \brief The ObjectControl class is intended as an overarching device
 *			used to control a scenario. No behaviour is implemented in it
 *			(this is left up to the State to determine), only functionality
 *			which can be called.
 */
class ObjectControl : public Module
{
	friend class ObjectControlState;
	friend class AbstractKinematics::Idle;
	friend class AbstractKinematics::Initialized;
	friend class AbstractKinematics::Connecting;
	friend class AbstractKinematics::Ready;
	friend class AbstractKinematics::Aborting;
	friend class AbstractKinematics::Armed;
	friend class AbstractKinematics::TestLive;
	friend class AbstractKinematics::Disarming;
	friend class AbstractKinematics::Done;
	friend class AbstractKinematics::RemoteControlled;
	friend class RelativeKinematics::Initialized;
	friend class RelativeKinematics::Connecting;
	friend class RelativeKinematics::Ready;
	friend class RelativeKinematics::Aborting;
	friend class RelativeKinematics::Armed;
	friend class RelativeKinematics::TestLive;
	friend class RelativeKinematics::Disarming;
	friend class RelativeKinematics::Done;
	friend class RelativeKinematics::RemoteControlled;
	friend class AbsoluteKinematics::Initialized;
	friend class AbsoluteKinematics::Connecting;
	friend class AbsoluteKinematics::Ready;
	friend class AbsoluteKinematics::Aborting;
	friend class AbsoluteKinematics::Armed;
	friend class AbsoluteKinematics::TestLive;
	friend class AbsoluteKinematics::Disarming;
	friend class AbsoluteKinematics::Done;
	friend class AbsoluteKinematics::RemoteControlled;

	friend class ObjectListener;

public:
	int initialize();
	ObjectControl();
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

	//! Handlers for MQ bus messages
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
	static inline std::string const moduleName = "object_control";
	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onConnectMessage(const ROSChannels::Connect::message_type::SharedPtr) override;
	void onArmMessage(const ROSChannels::Arm::message_type::SharedPtr) override;
	void onStartMessage(const ROSChannels::Start::message_type::SharedPtr) override;
	void onStartObjectMessage(const ROSChannels::StartObject::message_type::SharedPtr) override;
	void onDisconnectMessage(const ROSChannels::Disconnect::message_type::SharedPtr) override;
	void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;
	void onACCMMessage(const ROSChannels::ActionConfiguration::message_type::SharedPtr) override;
	void onEXACMessage(const ROSChannels::ExecuteAction::message_type::SharedPtr) override;
	void onRemoteControlEnableMessage(const ROSChannels::RemoteControlEnable::message_type::SharedPtr) override;
	void onRemoteControlDisableMessage(const ROSChannels::RemoteControlDisable::message_type::SharedPtr) override;
	void onControlSignalMessage(const ROSChannels::ControlSignal::message_type::SharedPtr) override;
	void onPathMessage(const ROSChannels::Path::message_type::SharedPtr,const uint32_t) override;

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

	ROSChannels::Init::Sub scnInitSub;			//!< Subscriber to scenario initialization requests
	ROSChannels::Start::Sub scnStartSub;		//!< Subscriber to scenario start requests
	ROSChannels::StartObject::Sub objectStartSub;	//!< Subscriber to scenario start requests
	ROSChannels::Arm::Sub scnArmSub;			//!< Subscriber to scenario arm requests
	ROSChannels::Stop::Sub scnStopSub;			//!< Subscriber to scenario stop requests
	ROSChannels::Abort::Sub scnAbortSub;		//!< Subscriber to scenario abort requests
	ROSChannels::AllClear::Sub scnAllClearSub;	//!< Subscriber to scenario all clear requests
	ROSChannels::Connect::Sub scnConnectSub;	//!< Subscriber to scenario connect requests
	ROSChannels::Disconnect::Sub scnDisconnectSub;	//!< Subscriber to scenario disconnect requests
	ROSChannels::ExecuteAction::Sub scnActionSub;		//!< Subscriber to scenario action requests
	ROSChannels::ActionConfiguration::Sub scnActionConfigSub;	//!< Subscriber to scenario action configuration requests
	ROSChannels::RemoteControlEnable::Sub scnRemoteControlEnableSub;		//!< Subscriber to remote control enable requests
	ROSChannels::RemoteControlDisable::Sub scnRemoteControlDisableSub;	//!< Subscriber to remote control disable requests
	ROSChannels::GetStatus::Sub getStatusSub;				//!< Subscriber to scenario get status requests
	std::shared_ptr<ROSChannels::ControlSignal::Sub> controlSignalSub;	//!< Pointer to subscriber to receive control signal messages with percentage

	rclcpp::TimerBase::SharedPtr objectsConnectedTimer;	//!< Timer to periodically publish connected objects

	ROSChannels::Failure::Pub failurePub;					//!< Publisher to scenario failure reports
	ROSChannels::Abort::Pub scnAbortPub;					//!< Publisher to scenario abort reports
	ROSChannels::ObjectsConnected::Pub objectsConnectedPub;	//!< Publisher to report that objects have been connected
	ROSChannels::ConnectedObjectIds::Pub connectedObjectIdsPub;	//!< Publisher to periodically report connected object ids
	rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedPtr idClient;	//!< Client to request object ids
	rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedPtr trajectoryClient;	//!< Client to request object trajectories
	rclcpp::Client<atos_interfaces::srv::GetObjectIp>::SharedPtr ipClient;	//!< Client to request object IPs
	rclcpp::Client<atos_interfaces::srv::GetObjectTriggerStart>::SharedPtr triggerClient;	//!< Client to request object trigger start

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
	void loadScenario();
	//! \brief Read all object files and fill the list of TestObjects.
	void loadObjectFiles();
	//! \brief Transform the scenario trajectories relative to the trajectory of the
	//!			specified object.
	void transformScenarioRelativeTo(const uint32_t objectID);
	//! \brief Upload the configuration in the ScenarioHandler to the connected obj	std::unique_ptr<ScenarioHandler> scenarioHandler;
	void uploadObjectConfiguration(const uint32_t id);
	//! \brief Clear loaded data and object list.
	void clearScenario();

	//! \brief TODO
	void armObjects();
	//! \brief TODO
	void disarmObjects();
	//! \brief
	void startObjects();
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
};

