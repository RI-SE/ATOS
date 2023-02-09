#pragma once

#include <netinet/in.h>
#include <set>
#include "trajectory.hpp"
#include "loggable.hpp"

// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

// TODO move out of object config class (which should only contain sourceIDs per setting file)
struct DataInjectionMap {
	std::set<uint32_t> sourceIDs;
	std::set<uint32_t> targetIDs;
};

class ObjectConfig : public Loggable {
public:
	ObjectConfig(rclcpp::Logger);
	//ObjectConfig(const ObjectConfig&&);

	void parseConfigurationFile(const fs::path& file);

	bool isAnchor() const { return isAnchorObject; }
	bool isOSI() const { return isOSICompatible; }
	DataInjectionMap getInjectionMap() const { return injectionMap; }
	in_addr_t getIP(void) const { return remoteIP; }
	double getMaximumSpeed() const { return maximumSpeed; }
	GeographicPositionType getOrigin() const { return origin; }
	std::string getProjString() const;
	ATOS::Trajectory getTrajectory() const { return trajectory; }
	void setTrajectory(const ATOS::Trajectory& newTraj) { trajectory = newTraj; } // TODO danger danger - don't do this
	uint32_t getTransmitterID() const { return transmitterID; }
	void setTransmitterID(const uint32_t id) { transmitterID = id; }
	double getTurningDiameter() const { return turningDiameter; }
	std::string getObjectFileName() const { return objectFile.filename().string(); }
	std::string getTrajectoryFileName() const { return trajectoryFile.filename().string(); }
	void addInjectionTarget(const uint32_t target) { this->injectionMap.targetIDs.insert(target); }
	void clearInjectionSources() { this->injectionMap.sourceIDs.clear(); }
	void setOrigin(const GeographicPositionType& origin) { this->origin = origin; }

	std::string toString() const;

private:
	fs::path objectFile;
	fs::path trajectoryFile;
	fs::path opendriveFile;
	fs::path openscenarioFile;
	bool isAnchorObject = false;
	bool isOSICompatible = false;
	in_addr_t remoteIP = 0;
	double maximumSpeed = 0;
	bool hasMaximumSpeed = false;
	GeographicPositionType origin;
	ATOS::Trajectory trajectory;
	uint32_t transmitterID = 0;
	bool turningDiameterKnown = false;
	double turningDiameter = 0;
	DataInjectionMap injectionMap;

	template<size_t N>
	bool isSettingTrue(char (&setting)[N]);
	void split(std::string &str, char delim, std::vector<int>& out);
};
