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
	friend class RelativeKinematics::Initialized;
	friend class RelativeKinematics::Connecting;
	friend class RelativeKinematics::Ready;
	friend class RelativeKinematics::Aborting;
	friend class RelativeKinematics::Armed;
	friend class RelativeKinematics::TestLive;
	friend class RelativeKinematics::Disarming;
	friend class RelativeKinematics::Done;
	friend class AbsoluteKinematics::Initialized;
	friend class AbsoluteKinematics::Connecting;
	friend class AbsoluteKinematics::Ready;
	friend class AbsoluteKinematics::Aborting;
	friend class AbsoluteKinematics::Armed;
	friend class AbsoluteKinematics::TestLive;
	friend class AbsoluteKinematics::Disarming;
	friend class AbsoluteKinematics::Done;

	friend class ObjectListener;

public:
	int initialize(LOG_LEVEL logLevel);
	ObjectControl(LOG_LEVEL logLevel);
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
		auto res = std::find_if(objects.begin(), objects.end(), [&](const std::pair<const uint32_t,TestObject>& elem){
			return elem.second.getObjectConfig().getIP() == ip;
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

private:
	static inline std::string const moduleName = "ObjectControl";
	void onInitMessage(const Empty::SharedPtr) override;
	void onConnectMessage(const Empty::SharedPtr) override;
	void onArmMessage(const Empty::SharedPtr) override;
	void onStartMessage(const Empty::SharedPtr) override;
	void onDisconnectMessage(const Empty::SharedPtr) override;
	void onStopMessage(const Empty::SharedPtr) override;
	void onAbortMessage(const Empty::SharedPtr) override;
	void onAllClearMessage(const Empty::SharedPtr) override;
	void onACCMMessage(const Accm::SharedPtr) override;
	void onEXACMessage(const Exac::SharedPtr) override;

	using clock = std::chrono::steady_clock;

	ControlMode controlMode;
	ObjectControlState* state;					//!< State of module
	std::map<uint32_t,TestObject> objects;		//!< List of configured test participants
	std::map<uint32_t,ObjectListener> objectListeners;
	std::map<uint16_t,std::function<void()>> storedActions;
	std::mutex monitorTimeMutex;
	static constexpr auto heartbeatPeriod = std::chrono::milliseconds(1000 / HEAB_FREQUENCY_HZ);
	std::thread safetyThread;
	std::promise<void> stopHeartbeatSignal;

	std::shared_future<void> connStopReqFuture;	//!< Request to stop a connection attempt
	std::promise<void> connStopReqPromise;		//!< Promise that the above value will be emitted

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
	void connectToObject(TestObject& obj, std::shared_future<void>& connStopReq);

	void startListeners();

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
	void allClearObjects();
	//! \brief TODO
	void injectObjectData(const MonitorMessage& monr);
	//! \brief TODO
	OsiHandler::LocalObjectGroundTruth_t buildOSILocalGroundTruth(const MonitorMessage&) const;
};

