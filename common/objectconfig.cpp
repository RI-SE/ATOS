#include "objectconfig.hpp"
#include "util.h"
#include <iomanip>

ObjectConfig::ObjectConfig(rclcpp::Logger lg) : Loggable(lg), trajectory(lg) {
	origin.latitude_deg = origin.longitude_deg = origin.altitude_m = 0.0;
	origin.isLongitudeValid = origin.isLatitudeValid = origin.isAltitudeValid = false;
}

/*
ObjectConfig::ObjectConfig(const ObjectConfig&& other) :
	objectFile(other.objectFile),
	transmitterID(other.transmitterID),
	injectionMap(other.injectionMap),
	isAnchorObject(other.isAnchorObject),
	isOSICompatible(other.isOSICompatible),
	trajectory(other.trajectory),
	origin(other.origin)
{

}
*/
std::string ObjectConfig::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &remoteIP, ipAddr, sizeof (ipAddr));
	std::string retval = "";

	std::string idsString;
	for(auto id: injectionMap.sourceIDs) {
		idsString += std::to_string(id) + " ";
	}

	retval += "\n Object ID: " + std::to_string(transmitterID)
			+ "\n IP: " + ipAddr + "\n Trajectory: " + trajectory.name.c_str()
			+ "\n OpenDRIVE: " + opendriveFile.filename().string().c_str()
			+ "\n OpenSCENARIO: " + openscenarioFile.filename().string().c_str()
			+ "\n Turning diameter: " + std::to_string(turningDiameter) + "\n Max speed: " + std::to_string(maximumSpeed)
			+ "\n Object file: " + objectFile.filename().string() + "\n Anchor: " + (isAnchorObject? "Yes":"No")
			+ "\n OSI compatible: " + (isOSICompatible? "Yes":"No")
			+ "\n Injection IDs: " + idsString;
	return retval;
}

void ObjectConfig::parseConfigurationFile(
		const fs::path &objectFile) {

	char setting[100];
	int result;
	char trajDirPath[MAX_FILE_PATH];
	char odrDirPath[MAX_FILE_PATH];
	char oscDirPath[MAX_FILE_PATH];

	UtilGetTrajDirectoryPath(trajDirPath, sizeof (trajDirPath));
	UtilGetOdrDirectoryPath(odrDirPath, sizeof (odrDirPath));
	UtilGetOscDirectoryPath(oscDirPath, sizeof (oscDirPath));


	// Get IP setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, objectFile.c_str(),
								 objectFile.string().length()+1, setting,
								 sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find IP setting in file " + objectFile.string());
	}
	result = inet_pton(AF_INET, setting, &this->remoteIP);
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
		this->isAnchorObject = this->isSettingTrue(setting);
	}

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == 0) {

		fs::path trajFile(std::string(trajDirPath) + std::string(setting));
		if (!fs::exists(trajFile.string())) {
			throw std::invalid_argument("Configured trajectory file " + std::string(setting)
										+ " in file " + objectFile.string() + " not found");
		}
		this->trajectoryFile = trajFile;
		this->trajectory.initializeFromFile(setting);
		RCLCPP_DEBUG(get_logger(), "Loaded trajectory with %u points", trajectory.points.size());
	}

	// Get opendrive file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_OPENDRIVE, objectFile.c_str(),
								objectFile.string().length(),
								setting, sizeof(setting)) == 0) {

		fs::path odrFile(std::string(odrDirPath) + std::string(setting));
		if (!fs::exists(odrFile.string())) {
			throw std::invalid_argument("Configured OpenDRIVE file " + std::string(setting)
										+ " in file " + objectFile.string() + " not found");
		}
		this->opendriveFile = odrFile;
	}
	
	// Get openscenario file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_OPENSCENARIO, objectFile.c_str(),
								objectFile.string().length(),
								setting, sizeof(setting)) == 0) {

		fs::path oscFile(std::string(oscDirPath) + std::string(setting));
		if (!fs::exists(oscFile.string())) {
			throw std::invalid_argument("Configured OpenSCENARIO file " + std::string(setting)
										+ " in file " + objectFile.string() + " not found");
		}
		this->openscenarioFile = oscFile;
	}

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
			GeoPositionType orig;
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

	// Get Turning diameter
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TURNING_DIAMETER, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			throw std::invalid_argument("Turning diameter " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
		}
		this->turningDiameterKnown = true;
		this->turningDiameter = val;
	}

	// Get Maximum speed
	if (UtilGetObjectFileSetting(OBJECT_SETTING_MAX_SPEED, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		char *endptr;
		double val = strtod(setting, &endptr);

		if (endptr == setting) {
			throw std::invalid_argument("Max speed " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
		}
		this->hasMaximumSpeed = true;
		this->maximumSpeed = val;
	}

	// Get OSI compatibility
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IS_OSI_COMPATIBLE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		this->isOSICompatible = false;
	}
	else {
		this->isOSICompatible = this->isSettingTrue(setting);
	}
	// Get Injector IDs
	if (UtilGetObjectFileSetting(OBJECT_SETTING_INJECTOR_IDS, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		std::vector<int> ids;
		std::string settingString(setting);
		this->split(settingString, ',', ids);

		this->injectionMap.sourceIDs.clear();
		this->injectionMap.targetIDs.clear();

		for (const auto& id : ids) {
			RCLCPP_DEBUG(get_logger(), "Injection ID %d", id);
			this->injectionMap.sourceIDs.insert(static_cast<uint32_t>(id));
		}
	}
	this->objectFile = objectFile;
}

template<size_t N>
bool ObjectConfig::isSettingTrue(char (&setting)[N]) {
	if (setting[0] == '1' || setting[0] == '0') {
		return setting[0] == '1';
	}
	else {
		std::string settingString(setting);
		for (char &c : settingString) {
			c = std::tolower(c, std::locale());
		}
		if (settingString.compare("true") == 0) {
			return true;
		}
		else if (settingString.compare("false") == 0) {
			return false;
		}
		else {
			throw std::invalid_argument("Setting " + settingString + " is invalid");
		}
	}
}

void ObjectConfig::split(std::string &str, char delim, std::vector<int> &out) {
	size_t start;
	size_t end = 0;

	while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
		end = str.find(delim, start);
		out.push_back(stoi(str.substr(start, end - start)));
	}
}

std::string ObjectConfig::getProjString() const {
	std::stringstream projStr;
	projStr << "+proj=topocentric +ellps=GRS80 ";
	projStr << std::setprecision(10) << "+lat_0=" << this->getOrigin().latitude_deg << " ";
	projStr << std::setprecision(10) << "+lon_0=" << this->getOrigin().longitude_deg << " ";
	projStr << std::setprecision(10) << "+h_0=" << this->getOrigin().altitude_m;
	return projStr.str();
}
