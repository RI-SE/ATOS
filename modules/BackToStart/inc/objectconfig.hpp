#pragma once

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

	uint32_t getTransmitterID() const { return transmitterID; }
	uint32_t getTurningRadius() const { return turningRadius; }
	uint32_t getMaximumSpeed() const { return maximumSpeed; }

	std::string toString() const;

private:
	fs::path objectFile;
	uint32_t transmitterID = 0;
	double turningRadius = 0;
	double maximumSpeed = 0;
};
