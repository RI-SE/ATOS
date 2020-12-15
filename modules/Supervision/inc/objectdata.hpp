#ifndef OBJECTDATA_HPP
#define OBJECTDATA_HPP
#include "trajectory.hpp"

class ObjectData
{
public:
	ObjectData();
	ObjectData(const std::string &fromFile);
	Trajectory trajectory;
	in_addr_t ip;

	void initializeFromFile(const std::string &fileName);
};

#endif // OBJECTDATA_HPP
