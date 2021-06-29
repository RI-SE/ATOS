#include "scenariohandler.hpp"
#include "logging.h"
#include "util.h"
#include "datadictionary.h"
#include "osi_handler.hpp"

#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>
#include <dirent.h>


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
	//Load injectorIds
	uint32_t numberOfObjects;
	DataDictionaryGetNumberOfObjects(&numberOfObjects);
 	uint32_t transmitterIDs[numberOfObjects];
 	DataDictionaryGetObjectTransmitterIDs(transmitterIDs, numberOfObjects);
	configureObjectDataInjection(this->dataInjectionMaps, transmitterIDs, numberOfObjects);

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
		/*
		//Check if new monr to send as OSI
		for(const auto& id: getVehicleIDs())
		{
			int32_t nofObjects = getVehicleIDs().size();
			for(int i = 0; i < getVehicleIDs().size(); i ++){

					if(objects[id].getTransmitterID() == this->dataInjectionMaps[i].sourceID &&
						objects[id].getTimeSinceLastMonitor() > std::chrono::milliseconds(0))
					{
						std::vector<char> outBuffer =
						buildOSIgogtArray(objects[id].getTransmitterID());
						for(int j = 0; j < this->dataInjectionMaps[i].numberOfTargets; j ++){
						    objects[this->dataInjectionMaps[i].targetIDs[j]].sendOsiData(outBuffer);
		  				}
		  			}
	  			}
  				
		}
		*/
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

std::vector<char> ScenarioHandler::buildOSIgogtArray(uint32_t transmitterId) {
	ObjectMonitorType monrData;
	DataDictionaryGetMonitorData(transmitterId, &monrData);
	OsiHandler osi;

    OsiHandler::GlobalObjectGroundTruth_t gt;

    auto d = std::chrono::seconds{monrData.timestamp.tv_sec};
    + std::chrono::nanoseconds{monrData.timestamp.tv_usec*1000};
    std::chrono::system_clock::time_point ositime {d};

    gt.id = transmitterId;

    gt.pos_m.x = monrData.position.xCoord_m;
    gt.pos_m.y = monrData.position.yCoord_m;
    gt.pos_m.z = monrData.position.zCoord_m;
    char miscData[MISC_DATA_MAX_SIZE];
    DataDictionaryGetMiscData(miscData);
    std::string projstr(miscData);
    std::string sendstr;
    sendstr = osi.encodeSvGtMessage(gt, ositime, projstr, true);
    std::vector<char> outBuffer(sendstr.begin(), sendstr.end());
    return outBuffer;
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
				int initializingMonrs = 500;
				int connectionHeartbeats = 500;
				while (true) {
					ObjectStateType objState = OBJECT_STATE_UNKNOWN;
					try {
						obj.sendHeartbeat(this->state->asControlCenterStatus());
						objState = obj.getState(true, heartbeatPeriod);
					} catch (std::runtime_error& e) {
						if (connectionHeartbeats-- > 0) {
							continue;
						}
						else {
							throw e;
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


int ScenarioHandler::configureObjectDataInjection(DataInjectionMap injectionMaps[],
								 const uint32_t transmitterIDs[], const unsigned int numberOfObjects) {

	char objectDirPath[MAX_FILE_PATH];
	char objectFilePath[MAX_FILE_PATH];
	DIR *objectDirectory;
	struct dirent *dirEntry;
	int retval = 0;

	if (injectionMaps == NULL || transmitterIDs == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Data injection configuration input pointer error");
		return -1;
	}

	// Reset maps
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		injectionMaps[i].sourceID = transmitterIDs[i];
		free(injectionMaps[i].targetIDs);
		injectionMaps[i].targetIDs = NULL;
		injectionMaps[i].numberOfTargets = 0;
		injectionMaps[i].isActive = 1;
	}

	UtilGetObjectDirectoryPath(objectDirPath, sizeof (objectDirPath));
	objectDirectory = opendir(objectDirPath);
	if (objectDirectory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to open object directory");
		return -1;
	}

	while ((dirEntry = readdir(objectDirectory)) != NULL) {
		if (!strncmp(dirEntry->d_name, ".", 1)) {
			continue;
		}
		strcpy(objectFilePath, objectDirPath);
		strcat(objectFilePath, dirEntry->d_name);
		if (parseDataInjectionSetting(objectFilePath, injectionMaps, numberOfObjects) == -1) {
			retval = -1;
			LogMessage(LOG_LEVEL_ERROR, "Failed to parse injection settings of file %s", objectFilePath);
		}
	}

	return retval;
}


int ScenarioHandler::parseDataInjectionSetting(const char objectFilePath[MAX_FILE_PATH],
							  DataInjectionMap injectionMaps[], const unsigned int numberOfMaps) {

	char objectSetting[100];
	char *token = nullptr, *endptr = nullptr;
	const char delimiter[] = ",";
	int retval = 0;
	uint32_t sourceID = 0, targetID = 0;

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID,
								 objectFilePath, MAX_FILE_PATH,
								 objectSetting, sizeof (objectSetting)) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Object ID missing from file <%s>", objectFilePath);
		return -1;
	}

	targetID = (uint32_t) strtoul(objectSetting, &endptr, 10);
	if (endptr == objectSetting) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Invalid ID setting <%s> in file %s", objectSetting, objectFilePath);
		return -1;
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_INJECTOR_IDS,
								 objectFilePath, MAX_FILE_PATH,
								 objectSetting, sizeof (objectSetting)) == -1) {
		return 0;				// No setting found
	}

	token = strtok(objectSetting, delimiter);
	if (token == NULL) {
		return 0;				// Empty setting found
	}

	do {
		sourceID = (uint32_t) strtoul(token, &endptr, 10);
		if (endptr == token) {
			errno = EINVAL;
			LogMessage(LOG_LEVEL_ERROR, "Unparsable injector ID setting <%s>", token);
			retval = -1;
		}
		else {
			// Find the map matching source ID in configuration
			int found = false;
			for (unsigned int i = 0; i < numberOfMaps; ++i) {
				if (injectionMaps[i].sourceID == sourceID) {
					found = true;
					// Append object ID of open file to targets of ID in configuration
					injectionMaps[i].targetIDs =
						(uint32_t *)realloc(injectionMaps[i].targetIDs,
								++injectionMaps[i].numberOfTargets * sizeof (uint32_t));
					if (injectionMaps[i].targetIDs == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Memory allocation error");
						return -1;
					}
					injectionMaps[i].targetIDs[injectionMaps[i].numberOfTargets - 1] = targetID;
					LogMessage(LOG_LEVEL_INFO,
						   "Data injection from source ID %u to target ID %u",
						   sourceID, targetID);
				}
			}
			if (!found) {
				LogMessage(LOG_LEVEL_ERROR,
						   "Data injection source object with ID %u not among configured transmitter IDs",
						   sourceID, objectFilePath);
				retval = -1;
			}
		}
	} while ((token = strtok(NULL, delimiter)) != NULL);

	return retval;
}
