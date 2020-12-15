#ifndef OBJECTDATA_HPP
#define OBJECTDATA_HPP
#include "trajectory.hpp"

class ObjectConfiguration
{
public:
	ObjectConfiguration();
	ObjectConfiguration(const std::string &fromFile);
	Trajectory trajectory;
	in_addr_t ip = 0;
	uint32_t id = 0;

	void initializeFromFile(const std::string &fileName);
};

#endif // OBJECTDATA_HPP
