/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectconfiguration.hpp"
#include "util.h"
#include <exception>
#include <fstream>

ObjectConfiguration::ObjectConfiguration() {
}

ObjectConfiguration::ObjectConfiguration(const std::string &fromFile) {
	initializeFromFile(fromFile);
}

void ObjectConfiguration::initializeFromFile(
		const std::string &fileName) {

	using namespace std;
	char objectDirPath[PATH_MAX];
	char objectSetting[100];
	int result = 0;

	UtilGetObjectDirectoryPath(objectDirPath, sizeof (objectDirPath));
	string objectFilePath(objectDirPath);
	objectFilePath += fileName;

	ifstream file(objectFilePath);

	// Sanity check
	if (!file.is_open()) {
		throw ifstream::failure("Unable to open file <" + objectFilePath + ">");
	}
	file.close();

	// Get ID setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID, objectFilePath.c_str(), objectFilePath.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find ID setting in file <" + objectFilePath + ">");
	}
	this->id = static_cast<uint32_t>(stoul(objectSetting));

	// Get IP setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, objectFilePath.c_str(), objectFilePath.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find IP setting in file <" + objectFilePath + ">");
	}
	result = inet_pton(AF_INET, objectSetting, &this->ip);
	if (result == -1) {
		throw invalid_argument("Invalid IP address family in file <" + objectFilePath + ">");
	}
	else if (result == 0) {
		throw invalid_argument("Invalid IP address in file <" + objectFilePath + ">");
	}

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFilePath.c_str(), objectFilePath.length()+1,
								 objectSetting, sizeof (objectSetting)) == -1) {
		throw invalid_argument("Cannot find trajectory file setting in file <" + objectFilePath + ">");
	}
	trajectory.initializeFromFile(objectSetting);
	LogMessage(LOG_LEVEL_DEBUG, "Loaded trajectory with %u points", trajectory.points.size());
}
