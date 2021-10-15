#include "scenariohandler.hpp"
#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "osi_handler.hpp"
#include "objectconfig.hpp"

#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>
#include <dirent.h>


ScenarioHandler::ScenarioHandler() {
	this->state = static_cast<ObjectControlState*>(new ObjectControl::Idle);
	DataDictionarySetOBCState(this->state->asNumber());
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

void ScenarioHandler::handleActionConfigurationCommand(
		const TestScenarioCommandAction& action) {
	this->state->settingModificationRequested(*this);
	if (action.command == ACTION_PARAMETER_VS_SEND_START) {
		LogMessage(LOG_LEVEL_INFO, "Configuring delayed start for object %u", action.objectID);
		objects[action.objectID].setTriggerStart(true);
		this->storedActions[action.actionID] = std::bind(&TestObject::sendStart,
														 &objects[action.objectID]);
	}
}

void ScenarioHandler::handleExecuteActionCommand(
		const uint16_t &actionID,
		const std::chrono::system_clock::time_point &when) {
	this->state->actionExecutionRequested(*this);
	auto delayedExecutor = [&](){
		using namespace std::chrono;
		LogMessage(LOG_LEVEL_DEBUG, "Executing action %u in %d ms", actionID,
				   duration_cast<milliseconds>(when - system_clock::now()).count());
		std::this_thread::sleep_until(when);
		LogMessage(LOG_LEVEL_INFO, "Executing action %u", actionID);
		this->storedActions[actionID]();
	};
	auto thd = std::thread(delayedExecutor);
	thd.detach();
}

void ScenarioHandler::loadScenario() {
	this->loadObjectFiles();
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, TestObject> &o) {
		auto data = o.second.getAsObjectData();
		DataDictionarySetObjectData(&data);
	});
}

void ScenarioHandler::loadObjectFiles() {
	this->clearScenario();
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
							+ " detected in files " + objects[badID].getTrajectoryFileName()
							+ " and " + object.getTrajectoryFileName();
					throw std::invalid_argument(errMsg);
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, e.what());
				errors.push_back(e);
			}
		}
	}

	// Fix injector ID maps - reverse their direction
	for (const auto& id : getVehicleIDs()) {
		auto injMap = objects[id].getObjectConfig().getInjectionMap();
		for (const auto& sourceID : injMap.sourceIDs) {
			auto conf = objects[sourceID].getObjectConfig();
			conf.addInjectionTarget(id);
			objects[sourceID].setObjectConfig(conf);
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
	storedActions.clear();
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

OsiHandler::LocalObjectGroundTruth_t ScenarioHandler::buildOSILocalGroundTruth(
		const MonitorMessage& monr) const {

	OsiHandler::LocalObjectGroundTruth_t gt;

	gt.id = monr.first;
	gt.pos_m.x = monr.second.position.xCoord_m;
	gt.pos_m.y = monr.second.position.yCoord_m;
	gt.pos_m.z = monr.second.position.zCoord_m;
	gt.vel_m_s.lon = monr.second.speed.isLongitudinalValid ? monr.second.speed.longitudinal_m_s : 0.0;
	gt.vel_m_s.lat = monr.second.speed.isLateralValid ? monr.second.speed.lateral_m_s : 0.0;
	gt.vel_m_s.up = 0.0;
	gt.acc_m_s2.lon = monr.second.acceleration.isLongitudinalValid ? monr.second.acceleration.longitudinal_m_s2 : 0.0;
	gt.acc_m_s2.lat = monr.second.acceleration.isLateralValid ? monr.second.acceleration.lateral_m_s2 : 0.0;
	gt.acc_m_s2.up = 0.0;
	gt.orientation_rad.yaw = monr.second.position.isHeadingValid ? monr.second.position.heading_rad : 0.0;
	gt.orientation_rad.roll = 0.0;
	gt.orientation_rad.pitch = 0.0;

	return gt;
}

void ScenarioHandler::injectObjectData(const MonitorMessage &monr) {
	if (!objects[monr.first].getObjectConfig().getInjectionMap().targetIDs.empty()) {
		std::chrono::system_clock::time_point ts;
		auto secs = std::chrono::seconds(monr.second.timestamp.tv_sec);
		auto usecs = std::chrono::microseconds(monr.second.timestamp.tv_usec);
		ts += secs + usecs;
		auto osiGtData = buildOSILocalGroundTruth(monr);
		for (const auto& targetID : objects[monr.first].getObjectConfig().getInjectionMap().targetIDs) {
			if (objects[targetID].isOsiCompatible()) {
				objects[targetID].sendOsiData(osiGtData, objects[targetID].getProjString(), ts);
			}
		}
	}
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
				return;
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
						LogMessage(LOG_LEVEL_INFO, "Connected to armed object ID %u", obj.getTransmitterID());
						this->state->connectedToArmedObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
					case OBJECT_STATE_POSTRUN:
					case OBJECT_STATE_RUNNING:
						LogMessage(LOG_LEVEL_INFO, "Connected to running object ID %u", obj.getTransmitterID());
						this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						break;
					case OBJECT_STATE_INIT:
						if (initializingMonrs-- > 0) {
							continue;
						}
						else {
							LogMessage(LOG_LEVEL_INFO, "Connected object %u in initializing state after connection", obj.getTransmitterID());
							this->state->connectedToLiveObject(*this, obj.getTransmitterID());
						}
						break;
					case OBJECT_STATE_DISARMED:
						LogMessage(LOG_LEVEL_INFO, "Connected to disarmed object ID %u", obj.getTransmitterID());
						this->state->connectedToObject(*this, obj.getTransmitterID());
						break;
					default:
						LogMessage(LOG_LEVEL_INFO, "Connected to object %u in unknown state", obj.getTransmitterID());
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
		if (!objects[id].isStartingOnTrigger()) {
			objects[id].sendStart();
		}
	}
}

void ScenarioHandler::allClearObjects() {
	for (auto& id : getVehicleIDs()) {
		objects[id].sendAllClear();
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
