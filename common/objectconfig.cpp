#include "objectconfig.hpp"
#include "util.h"

ObjectConfig::ObjectConfig() {
	origin.latitude_deg = origin.longitude_deg = origin.altitude_m = 0.0;
	origin.isLongitudeValid = origin.isLatitudeValid = origin.isAltitudeValid = false;
}

ObjectConfig::ObjectConfig(const ObjectConfig&& other) :
	objectFile(other.objectFile),
	transmitterID(other.transmitterID),
	isAnchorObject(other.isAnchorObject),
	trajectory(other.trajectory),
	origin(other.origin)
{

}

std::string ObjectConfig::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &ip_addr, ipAddr, sizeof (ipAddr));
	std::string retval = "";

	retval += "Object ID " + std::to_string(transmitterID)
			+ ", IP " + ipAddr + ", trajectory " + trajectory.name.c_str()
			+ ", turning radius " + std::to_string(turningRadius) + ", max speed " + std::to_string(maximumSpeed)
			+ ", object file " + objectFile.filename().string() + ", anchor: " + (isAnchorObject?"Yes":"No");
	return retval;
}

void ObjectConfig::parseConfigurationFile(
		const fs::path &objectFile) {

	char setting[100];
	int result;
	struct sockaddr_in addr;
	char path[MAX_FILE_PATH];

	UtilGetTrajDirectoryPath(path, sizeof (path));

	// Get IP setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, objectFile.c_str(),
								 objectFile.string().length()+1, setting,
								 sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find IP setting in file " + objectFile.string());
	}
	result = inet_pton(AF_INET, setting, &this->ip_addr);
	if (result == -1) {
		using namespace std;
		throw system_error(make_error_code(static_cast<errc>(errno)));
	}
	else if (result == 0) {
		throw std::invalid_argument("Address " + std::string(setting)
									+ " in object file " + objectFile.string()
									+ " is not a valid IPv4 address");
	}

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

	// Get anchor setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IS_ANCHOR, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		this->isAnchorObject = false;
	}
	else {
		if (setting[0] == '1' || setting[0] == '0') {
			this->isAnchorObject = setting[0] == '1';
		}
		else {
			std::string anchorSetting(setting);
			for (char &c : anchorSetting) {
				c = std::tolower(c, std::locale());
			}
			if (anchorSetting.compare("true") == 0) {
				this->isAnchorObject = true;
			}
			else if (anchorSetting.compare("false") == 0) {
				this->isAnchorObject = false;
			}
			else {
				throw std::invalid_argument("Anchor setting " + std::string(setting) + " in file "
											+ objectFile.string() + " is invalid");
			}
		}
	}

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find trajectory setting in file " + objectFile.string());
	}

	fs::path trajFile(std::string(path) + std::string(setting));
	if (!fs::exists(trajFile)) {
		throw std::invalid_argument("Configured trajectory file " + std::string(setting)
									+ " in file " + objectFile.string() + " not found");
	}
	this->trajectory.initializeFromFile(setting);
	LogMessage(LOG_LEVEL_DEBUG, "Loaded trajectory with %u points", trajectory.points.size());

	// Get origin settings
	this->origin = {};
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_LATITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.latitude_deg = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isLatitudeValid = true;
		}
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_LONGITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.longitude_deg = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isLongitudeValid = true;
		}
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_ALTITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.altitude_m = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isAltitudeValid = true;
		}
	}

	if (origin.isAltitudeValid == origin.isLatitudeValid
			&& origin.isLatitudeValid == origin.isLongitudeValid) {
		if (!origin.isAltitudeValid) {
			GeoPosition orig;
			if (UtilReadOriginConfiguration(&orig) != -1) {
				this->origin.latitude_deg = orig.Latitude;
				this->origin.longitude_deg = orig.Longitude;
				this->origin.altitude_m = orig.Altitude;
				this->origin.isLatitudeValid = true;
				this->origin.isLongitudeValid = true;
				this->origin.isAltitudeValid = true;
			}
			else {
				throw std::invalid_argument("No origin setting found");
			}
		}
	}
	else {
		throw std::invalid_argument("Partial origin setting in file " + objectFile.string());
	}

	// Get Turning Radius
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TURNING_RADIUS, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find Turning radius setting in file " + objectFile.string());
	}
	else {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			throw std::invalid_argument("Turning radius " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
		}
		this->turningRadius = val;
	}

	// Get Maximum speed
	if (UtilGetObjectFileSetting(OBJECT_SETTING_MAX_SPEED, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find Max speed setting in file " + objectFile.string());
	}
	else {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			throw std::invalid_argument("Max speed " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
		}
		this->maximumSpeed = val;
	}

	this->objectFile = objectFile;
}
