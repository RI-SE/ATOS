#include "scenariohandler.hpp"
#include "logging.h"
#include "util.h"
#include <algorithm>
#include <stdexcept>
#include <fstream>


ScenarioHandler::ScenarioHandler(
		ControlMode controlMode) {
	switch (controlMode) {
	case RELATIVE_KINEMATICS:
		this->state = static_cast<ObjectControlState*>(new RelativeKinematics::Idle());
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

void ScenarioHandler::transformScenarioRelativeTo(
		const uint32_t objectID) {
	for (auto& id : getVehicleIDs()) {
		if (id == objectID) {
			// Skip for now TODO also here
			continue;
		}
		auto traj = objects[id].getTrajectory();
		auto relTraj = traj.relativeTo(objects[objectID].getTrajectory());
		LogPrint("TRAJECTORIES:");
		LogPrint(traj.toString().c_str());
		LogPrint(objects[objectID].getTrajectory().toString().c_str());
		LogPrint("RELATIVE:");
		LogPrint(relTraj.toString().c_str());

		objects[id].setTrajectory(relTraj);
	}
}

