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

class ObjectConfig {
public:
	ObjectConfig();
	ObjectConfig(const ObjectConfig&&);

	void parseConfigurationFile(const fs::path& file);

	in_addr_t getIP(void) const { return ip_addr; }
	uint32_t getTransmitterID() const { return transmitterID; }
	double getTurningRadius() const { return turningRadius; }
	double getMaximumSpeed() const { return maximumSpeed; }
	Trajectory getTrajectory() const { return trajectory; }
	GeographicPositionType getOrigin() const { return origin; }
	bool isAnchor() const { return isAnchorObject; }

	std::string toString() const;

private:
	fs::path objectFile;
	in_addr_t ip_addr = 0;
	uint32_t transmitterID = 0;
	bool isAnchorObject = false;
	Trajectory trajectory;
	GeographicPositionType origin;
	double turningRadius = 0;
	double maximumSpeed = 0;
};
