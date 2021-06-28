#include "scenariohandler.hpp"
#include "logging.h"
#include "util.h"
#include "datadictionary.h"

#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>


ScenarioHandler::ScenarioHandler(
		ControlMode controlMode) {
	switch (controlMode) {
	case RELATIVE_KINEMATICS:
		this->state = static_cast<ObjectControlState*>(new RelativeKinematics::Idle());
		DataDictionarySetOBCState(this->state->asNumber());
		break;
	case ABSOLUTE_KINEMATICS:
		// TODO
		LogMessage(LOG_LEVEL_ERROR, "Unimplemented control mode requested");
		break;
	}
	this->controlMode = controlMode;
}

ScenarioHandler::~ScenarioHandler() {
	delete state;
}

void ScenarioHandler::handleInitCommand() {
	this->state->initializeRequest(*this);
}

void ScenarioHandler::handleConnectCommand() {
	this->state->connectRequest(*this);
}

void ScenarioHandler::handleDisconnectCommand() {
	this->state->disconnectRequest(*this);
}

void ScenarioHandler::handleArmCommand() {
	this->state->armRequest(*this);
}

void ScenarioHandler::handleStartCommand() {
	this->state->startRequest(*this);
}

void ScenarioHandler::handleAbortCommand() {
	this->state->abortRequest(*this);
}

void ScenarioHandler::handleAllClearCommand() {
	this->state->allClearRequest(*this);
}

void ScenarioHandler::loadScenario() {
	this->loadObjectFiles();
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, TestObject> &o) {
		o.second.parseTrajectoryFile(o.second.getTrajectoryFile());
	});
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, TestObject> &o) {
		auto data = o.second.getAsObjectData();
		DataDictionarySetObjectData(&data);
	});
}

void ScenarioHandler::loadObjectFiles() {
	this->objects.clear();
	char path[MAX_FILE_PATH];
	std::vector<std::invalid_argument> errors;

	UtilGetObjectDirectoryPath(path, sizeof (path));
	fs::path objectDir(path);
	if (!fs::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	for (const auto& entry : fs::directory_iterator(objectDir)) {
		if (fs::is_regular_file(entry.status())) {
			const auto inputFile = entry.path();
			TestObject object;
			try {
				object.parseConfigurationFile(entry.path());
				LogMessage(LOG_LEVEL_INFO, "Loaded configuration: %s", object.toString().c_str());
				// Check preexisting
				auto foundObject = objects.find(object.getTransmitterID());
				if (foundObject == objects.end()) {
					objects.emplace(object.getTransmitterID(), std::move(object));
				}
				else {
					auto badID = object.getTransmitterID();
					std::string errMsg = "Duplicate object ID " + std::to_string(badID)
							+ " detected in files " + objects[badID].getTrajectoryFile().string()
							+ " and " + object.getTrajectoryFile().string();
					throw std::invalid_argument(errMsg);
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, e.what());
				errors.push_back(e);
			}
		}
	}
	if (!errors.empty()) {
		objects.clear();
		std::ostringstream ostr;
		auto append = [&ostr](const std::invalid_argument& e){
			ostr << e.what() << std::endl;
		};
		std::for_each(errors.begin(), errors.end(), append);
		throw std::invalid_argument("Failed to parse object file(s):\n" + ostr.str());
	}
}

uint32_t ScenarioHandler::getAnchorObjectID() const {
	for (auto& object : objects) {
		if (object.second.isAnchor()) {
			return object.first;
		}
	}
	throw std::invalid_argument("No configured anchor object found");
}

ObjectMonitorType ScenarioHandler::getLastAnchorData() const {
	auto anchorID = getAnchorObjectID();
	return objects.at(anchorID).getLastMonitorData();
}

std::map<uint32_t,ObjectStateType> ScenarioHandler::getObjectStates() const {
	std::map<uint32_t, ObjectStateType> retval;
	for (const auto& elem : objects) {
		retval[elem.first] = elem.second.getState();
	}
	return retval;
}

void ScenarioHandler::transformScenarioRelativeTo(
		const uint32_t objectID) {
	for (auto& id : getVehicleIDs()) {
		if (id == objectID) {
			// Skip for now TODO also here - maybe?
			continue;
		}
		auto traj = objects[id].getTrajectory();
		auto relTraj = traj.relativeTo(objects[objectID].getTrajectory());

		objects[id].setTrajectory(relTraj);
	}
}

void ScenarioHandler::clearScenario() {
	objects.clear();
}


void ScenarioHandler::beginConnectionAttempt() {
	connStopReqPromise = std::promise<void>();
	connStopReqFuture = connStopReqPromise.get_future();

	LogMessage(LOG_LEVEL_DEBUG, "Initiating connection attempt");
	for (const auto id : getVehicleIDs()) {
		auto t = std::thread(&ScenarioHandler::connectToObject, this,
							 std::ref(objects[id]),
							 std::ref(connStopReqFuture));
		t.detach();
	}
}

void ScenarioHandler::abortConnectionAttempt() {
	try {
		connStopReqPromise.set_value();
	}
	catch (std::future_error) {
		// Attempted to stop when none in progress
	}
}

void ScenarioHandler::disconnectObjects() {
	abortConnectionAttempt();
	try {
		stopHeartbeatSignal.set_value();
	}
	catch (std::future_error) {
		// Attempted to stop when none in progress
	}
	for (const auto id : getVehicleIDs()) {
		objects[id].disconnect();
	}
	objectListeners.clear();
}

void ScenarioHandler::disconnectObject(
		const uint32_t id) {
	objects[id].disconnect();
	objectListeners.erase(id);
}

void ScenarioHandler::uploadObjectConfiguration(
		const uint32_t id) {
	objects[id].sendSettings();
}

void ScenarioHandler::startSafetyThread() {
	stopHeartbeatSignal = std::promise<void>();
	if (safetyThread.joinable()) {
		safetyThread.join();
	}
	safetyThread = std::thread(&ScenarioHandler::heartbeat, this);
}

void ScenarioHandler::heartbeat() {
	auto stopRequest = stopHeartbeatSignal.get_future();
	clock::time_point nextHeartbeat = clock::now();

	LogMessage(LOG_LEVEL_DEBUG, "Starting heartbeat thread");
	while (stopRequest.wait_until(nextHeartbeat) == std::future_status::timeout) {
		nextHeartbeat += heartbeatPeriod;

		// Check time since MONR for all objects
		for (const auto& id : getVehicleIDs()) {
			if (objects[id].isConnected()) {
				auto diff = objects[id].getTimeSinceLastMonitor();
				if (diff > objects[id].getMaxAllowedMonitorPeriod()) {
					LogMessage(LOG_LEVEL_WARNING, "MONR timeout for object %u: %d ms > %d ms", id,
							   diff.count(), objects[id].getMaxAllowedMonitorPeriod().count());
					objects[id].disconnect();
					this->state->disconnectedFromObject(*this, id);
				}
			}
		}

		// Send heartbeat
		for (const auto& id : getVehicleIDs()) {
			try {
				if (objects[id].isConnected()) {
					objects[id].sendHeartbeat(this->state->asControlCenterStatus());
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_WARNING, e.what());
				objects[id].disconnect();
				this->state->disconnectedFromObject(*this, id);
			}
		}
	}
	LogMessage(LOG_LEVEL_INFO, "Heartbeat thread exiting");
}

void ScenarioHandler::startListeners() {
	LogMessage(LOG_LEVEL_DEBUG, "Starting listeners");
	objectListeners.clear();
	for (const auto& id : getVehicleIDs()) {
		objectListeners.try_emplace(id, this, &objects[id]);
	}
}

void ScenarioHandler::connectToObject(
		TestObject &obj,
		std::shared_future<void> &connStopReq) {
	constexpr int maxConnHeabs = 10;
	constexpr int maxConnMonrs = 10;
	try {
		if (!obj.isConnected()) {
			try {
				obj.establishConnection(connStopReq);
				obj.sendSettings();
			}
			catch (std::runtime_error& e) {
				LogMessage(LOG_LEVEL_ERROR, "Connection attempt for object %u failed: %s",
						   obj.getTransmitterID(), e.what());
				obj.disconnect();
				// TODO connection failed event?
			}
			try {
				int initializingMonrs = maxConnMonrs;
				int connectionHeartbeats = maxConnHeabs;
				while (true) {
					ObjectStateType objState = OBJECT_STATE_UNKNOWN;
					auto nextSendTime = std::chrono::system_clock::now();
					try {
						obj.sendHeartbeat(this->state->asControlCenterStatus());
						nextSendTime += heartbeatPeriod;
						objState = obj.getState(true, heartbeatPeriod);
					} catch (std::runtime_error& e) {
						if (connectionHeartbeats-- > 0) {
							std::this_thread::sleep_until(nextSendTime);
							continue;
						}
						else {
							throw std::runtime_error("No monitor reply after " + std::to_string(maxConnHeabs) + " heartbeats. Details:\n" + e.what());
						}
					}

					switch (objState) {
					case OBJECT_STATE_ARMED:
					case OBJECT_STATE_REMOTE_CONTROL:
						LogMessage(LOG_LEVEL_INFO, "Connected to armed object");
						this->state->connectedToArmedObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
					case OBJECT_STATE_POSTRUN:
					case OBJECT_STATE_RUNNING:
						LogMessage(LOG_LEVEL_INFO, "Connected to running object");
						this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_INIT:
						if (initializingMonrs-- > 0) {
							continue;
						}
						else {
							LogMessage(LOG_LEVEL_INFO, "Connected object in initializing state after connection");
							this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						}
						break;
					case OBJECT_STATE_DISARMED:
						LogMessage(LOG_LEVEL_INFO, "Connected to disarmed object");
						this->state->connectedToObject(*this, obj.getTransmitterID());
						break;
					default:
						LogMessage(LOG_LEVEL_INFO, "Connected to object in unknown state");
						this->state->connectedToLiveObject(*this, obj.getTransmitterID());
					}
					break;
				}
			}
			catch (std::runtime_error &e) {
				LogMessage(LOG_LEVEL_ERROR, "Connection startup procedure failed for object %u: %s",
						   obj.getTransmitterID(), e.what());
				obj.disconnect();
				// TODO: connection failed event?
			}
		}
	}
	catch (std::invalid_argument &e) {
		LogMessage(LOG_LEVEL_ERROR, "Bad connection attempt for object %u: %s",
				   obj.getTransmitterID(), e.what());
		obj.disconnect();
		// TODO: connection failed event?
	}
};

void ScenarioHandler::armObjects() {
	for (auto& id : getVehicleIDs()) {
		objects[id].sendArm();
	}
}

void ScenarioHandler::disarmObjects() {
	for (auto& id : getVehicleIDs()) {
		try {
			objects[id].sendDisarm();
		}
		catch (std::invalid_argument& e) {
			LogMessage(LOG_LEVEL_ERROR, "Unable to disarm object %u: %s", id, e.what());
		}
	}
	this->state->allObjectsDisarmed(*this); // TODO add a check on object states as well
}

void ScenarioHandler::startObjects() {
	for (auto& id : getVehicleIDs()) {
		objects[id].sendStart();
	}
}

bool ScenarioHandler::isAnyObjectIn(
		const ObjectStateType state) {
	return std::any_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

bool ScenarioHandler::areAllObjectsIn(
		const ObjectStateType state) {
	return std::all_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

bool ScenarioHandler::isAnyObjectIn(
		const std::set<ObjectStateType>& states) {
	return std::any_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,TestObject>& obj) {
		return states.find(obj.second.getState()) != states.end();
	});
}

bool ScenarioHandler::areAllObjectsIn(
		const std::set<ObjectStateType>& states) {
	return std::all_of(objects.cbegin(), objects.cend(), [states](const std::pair<const uint32_t,TestObject>& obj) {
		return states.find(obj.second.getState()) != states.end();
	});
}
