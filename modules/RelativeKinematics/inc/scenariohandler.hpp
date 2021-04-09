#pragma once

#include "state.hpp"
#include "testobject.hpp"
#include <map>
#include <future>
#include <set>

// Forward declarations
class ObjectControlState;

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
public:
	typedef enum {
		RELATIVE_KINEMATICS,
		ABSOLUTE_KINEMATICS
	} ControlMode;

	ScenarioHandler(ControlMode);
	~ScenarioHandler();

	void handleInitCommand();
	void handleConnectCommand();
	void handleDisconnectCommand();
	void handleArmCommand();

	std::vector<uint32_t> getVehicleUnderTestIDs() const;
	std::vector<uint32_t> getVehicleIDs() const {
		std::vector<uint32_t> retval;
		for (auto it  = objects.begin(); it != objects.end(); ++it) {
			retval.push_back(it->first);
		}
		return retval;
	}

	std::map<uint32_t,ObjectStateType> getObjectStates() const;
private:
	ObjectControlState* state;
	std::map<uint32_t,TestObject> objects;

	std::promise<void> connStopReqPromise;
	std::shared_future<void> connStopReqFuture;

	void beginConnectionAttempt();
	void abortConnectionAttempt() {
		try {
			connStopReqPromise.set_value();
		}
		catch (std::future_error) {
			// Attempted to stop when none in progress
		}
	}

	void disconnectObjects();

	void loadScenario();
	void loadObjectFiles();
	void parseObjectFile(const fs::path& objectFile, TestObject& object);
	void transformScenarioRelativeTo(const uint32_t objectID);

	void uploadObjectConfiguration(const uint32_t id);

	void clearScenario();

	bool isAnyObjectIn(const ObjectStateType state);
	bool isAnyObjectIn(const std::set<ObjectStateType>& state);
	bool areAllObjectsIn(const ObjectStateType state);
	bool areAllObjectsIn(const std::set<ObjectStateType>& state);

	void connectToObject(TestObject& obj, std::shared_future<void>& connStopReq);

	void armObjects();
};


