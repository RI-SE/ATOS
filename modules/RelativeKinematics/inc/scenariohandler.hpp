#pragma once

#include "state.hpp"
#include "testobject.hpp"
#include "objectlistener.hpp"
#include <map>
#include <future>
#include <set>

// Forward declarations
class ObjectControlState;
class ObjectListener;

namespace ObjectControl {
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

class ScenarioHandler {
	friend class ObjectControlState;
	friend class ObjectControl::Idle;
	friend class ObjectControl::Initialized;
	friend class ObjectControl::Connecting;
	friend class ObjectControl::Ready;
	friend class ObjectControl::Aborting;
	friend class ObjectControl::Armed;
	friend class ObjectControl::TestLive;
	friend class ObjectControl::Disarming;
	friend class ObjectControl::Done;
	friend class RelativeKinematics::Idle;
	friend class RelativeKinematics::Initialized;
	friend class RelativeKinematics::Connecting;
	friend class RelativeKinematics::Ready;
	friend class RelativeKinematics::Aborting;
	friend class RelativeKinematics::Armed;
	friend class RelativeKinematics::TestLive;
	friend class RelativeKinematics::Disarming;
	friend class RelativeKinematics::Done;

	friend class ObjectListener;
public:
	typedef enum {
		RELATIVE_KINEMATICS,	//!< Scenario executed relative to immobile VUT
		ABSOLUTE_KINEMATICS		//!< Scenario executed relative to earth-fixed point
	} ControlMode;


	typedef struct {
		uint32_t sourceID;
		unsigned int numberOfTargets;
		uint32_t *targetIDs;
		int isActive;
	} DataInjectionMap;

	ScenarioHandler(ControlMode);
	~ScenarioHandler();

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
	void handleAbortCommand();

	void handleAllClearCommand();

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
	using clock = std::chrono::steady_clock;

	ControlMode controlMode;
	ObjectControlState* state;					//!< State of module
	std::map<uint32_t,TestObject> objects;		//!< List of configured test participants
	std::map<uint32_t,ObjectListener> objectListeners;
	std::mutex monitorTimeMutex;
	static constexpr auto heartbeatPeriod = std::chrono::milliseconds(1000 / HEAB_FREQUENCY_HZ);
	std::thread safetyThread;
	std::promise<void> stopHeartbeatSignal;

	std::shared_future<void> connStopReqFuture;	//!< Request to stop a connection attempt
	std::promise<void> connStopReqPromise;		//!< Promise that the above value will be emitted

	DataInjectionMap dataInjectionMaps[MAX_OBJECTS];
	
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
	//! \brief Upload the configuration in the ScenarioHandler to the connected objects.
	void uploadObjectConfiguration(const uint32_t id);
	//! \brief Clear loaded data and object list.
	void clearScenario();

	//! \brief TODO
	void armObjects();
	//! \brief TODO
	void disarmObjects();
	//! \brief
	void startObjects();
	int configureObjectDataInjection(DataInjectionMap injectionMaps[],
								 const uint32_t transmitterIDs[],
								 const unsigned int numberOfObjects);
	int parseDataInjectionSetting(const char objectFilePath[MAX_FILE_PATH],
								  DataInjectionMap injectionMaps[],
								  const unsigned int numberOfMaps);


};

