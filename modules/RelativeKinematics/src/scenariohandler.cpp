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

void ScenarioHandler::loadScenario() {
	this->loadObjectFiles();
	std::for_each(objects.begin(), objects.end(), [] (std::pair<const uint32_t, TestObject> &o) {
		o.second.parseTrajectoryFile(o.second.getTrajectoryFile());
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
					objects[object.getTransmitterID()] = object;
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

std::vector<uint32_t> ScenarioHandler::getVehicleUnderTestIDs() const {
	std::vector<uint32_t> retval;
	for (auto& object : objects) {
		if (object.second.isVehicleUnderTest()) {
			retval.push_back(object.first);
		}
	}
	return retval;
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
			// Skip for now TODO also here
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
	connStopReqFuture = connStopReqPromise.get_future();

	auto connect = [this](TestObject& obj, std::shared_future<void>& connStopReq) {
		try {
			if (!obj.isConnected()) {
				obj.establishConnection(connStopReq);
				this->state->connectedToObject(*this, obj.getTransmitterID());
			}
		}
		catch (std::runtime_error &e) {
			LogMessage(LOG_LEVEL_ERROR, "Connection attempt for object %u failed: %s",
					   obj.getTransmitterID(), e.what());
		}
		catch (std::invalid_argument &e) {
			LogMessage(LOG_LEVEL_ERROR, "Bad connection attempt for object %u: %s",
					   obj.getTransmitterID(), e.what());
		}
	};

	LogMessage(LOG_LEVEL_DEBUG, "Initiating connection attempt");
	for (const auto id : getVehicleIDs()) {
		auto t = std::thread(connect, std::ref(objects[id]), std::ref(connStopReqFuture));
		t.detach();
	}
}

void ScenarioHandler::disconnectObjects() {
	abortConnectionAttempt();
	for (const auto id : getVehicleIDs()) {
		objects[id].disconnect();
	}
}

void ScenarioHandler::uploadObjectConfiguration(
		const uint32_t id) {
	objects[id].sendSettings();
}

bool ScenarioHandler::isAnyObjectIn(
		const ObjectStateType state) const {
	return std::any_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

bool ScenarioHandler::areAllObjectsIn(
		const ObjectStateType state) const {
	return std::all_of(objects.cbegin(), objects.cend(), [state](const std::pair<const uint32_t,TestObject>& obj) {
		return obj.second.getState() == state;
	});
}

