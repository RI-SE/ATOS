#pragma once

#include <netinet/in.h>
#include "trajectory.hpp"

// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

struct DataInjectionMap {
	uint32_t sourceID;
	unsigned int numberOfTargets;
	std::vector<uint32_t> targetIDs;
	bool isActive;
};

class ObjectConfig {
public:
	ObjectConfig();
	ObjectConfig(const ObjectConfig&&);

	void parseConfigurationFile(const fs::path& file);

	bool isAnchor() const { return isAnchorObject; }
	bool isOSI() const { return isOSICompatible; }
	DataInjectionMap getInjectionMap() const { return injectionMap; }
	in_addr_t getIP(void) const { return ip_addr; }
	double getMaximumSpeed() const { return maximumSpeed; }
	GeographicPositionType getOrigin() const { return origin; }
	Trajectory getTrajectory() const { return trajectory; }
	uint32_t getTransmitterID() const { return transmitterID; }
	double getTurningDiameter() const { return turningDiameter; }

	std::string toString() const;

private:
	fs::path objectFile;
	bool isAnchorObject = false;
	bool isOSICompatible = false;
	in_addr_t ip_addr = 0;
	double maximumSpeed = 0;
	GeographicPositionType origin;
	Trajectory trajectory;
	uint32_t transmitterID = 0;
	double turningDiameter = 0;
	DataInjectionMap injectionMap;

	bool isSettingTrue(std::string setting);
	void split(std::string &str, char delim, std::vector<int> &out);
};
