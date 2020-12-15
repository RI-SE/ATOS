#include "objectdata.hpp"
#include "util.h"
#include <exception>
#include <fstream>

ObjectData::ObjectData() {
}

ObjectData::ObjectData(const std::string &fromFile) {
	initializeFromFile(fromFile);
}

void ObjectData::initializeFromFile(
		const std::string &fileName) {

	using namespace std;
	char objectSetting[100];
	int result = 0;
	ifstream file(fileName);

	// Sanity check
	if (!file.is_open()) {
		throw ifstream::failure("Unable to open file <" + fileName + ">");
	}
	file.close();

	// Get ID setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID, fileName.c_str(), fileName.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find ID setting in file <" + fileName + ">");
	}
	this->id = static_cast<uint32_t>(stoul(objectSetting));

	// Get IP setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, fileName.c_str(), fileName.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find IP setting in file <" + fileName + ">");
	}
	result = inet_pton(AF_INET, objectSetting, &this->ip);
	if (result == -1) {
		throw invalid_argument("Invalid IP address family in file <" + fileName + ">");
	}
	else if (result == 0) {
		throw invalid_argument("Invalid IP address in file <" + fileName + ">");
	}

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, fileName.c_str(), fileName.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find trajectory file setting in file <" + fileName + ">");
	}
	trajectory.initializeFromFile(objectSetting);
	LogMessage(LOG_LEVEL_DEBUG, "Loaded trajectory with %u points", trajectory.points.size());
}
