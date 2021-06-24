#include "objectconfig.hpp"
#include "util.h"

ObjectConfig::ObjectConfig() {
}

ObjectConfig::ObjectConfig(const ObjectConfig&& other) :
	objectFile(other.objectFile),
	transmitterID(other.transmitterID)
{

}

std::string ObjectConfig::toString() const {
	std::string retval = "";
	retval += "Object ID " + std::to_string(transmitterID)
		+ ", object file " + objectFile.filename().string();
	return retval;
}

void ObjectConfig::parseConfigurationFile(
		const fs::path &objectFile) {

	char setting[100];
	int result;
	struct sockaddr_in addr;
	char path[MAX_FILE_PATH];

	// Get ID setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find ID setting in file " + objectFile.string());
	}
	char *endptr;
	uint32_t id = static_cast<uint32_t>(strtoul(setting, &endptr, 10));

	if (endptr == setting) {
		throw std::invalid_argument("ID " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
	}
	this->transmitterID = id;

	// Get Turning Radius
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TURNING_RADIUS, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Cannot find Turning radius setting in file %s", path);
			throw std::invalid_argument("Turning radius " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
	}
	else {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			LogMessage(LOG_LEVEL_ERROR, "Turning radius <%s> in file <%s> is invalid", setting, path);
		}
		else {
			// set value
			LogMessage(LOG_LEVEL_INFO, "Read turning radius %.3lf [m]", val);
		}
	}

	// Get Maximum speed
	if (UtilGetObjectFileSetting(OBJECT_SETTING_MAX_SPEED, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Cannot find Max speed setting in file %s", path);
			throw std::invalid_argument("Max speed " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
	}
	else {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			LogMessage(LOG_LEVEL_ERROR, "Max speed <%s> in file <%s> is invalid", setting, path);
		}
		else {
			// set value
			LogMessage(LOG_LEVEL_INFO, "Read Max speed %.3lf [m/s]", val);
		}
	}

	this->objectFile = objectFile;
}
