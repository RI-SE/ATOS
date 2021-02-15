#pragma once

#include "state.hpp"
#include "testobject.hpp"
#include <map>



class ObjectControlState;

class ObjectHandler {
	friend class ObjectControlState;
public:
	typedef enum {
		RELATIVE_KINEMATICS,
		ABSOLUTE_KINEMATICS
	} ControlMode;

	ObjectHandler(ControlMode);
	~ObjectHandler();

	void loadConfiguration();
private:
	ObjectControlState* state;
	std::map<uint32_t,TestObject> objects;

	void loadObjectFiles();
	void parseObjectFile(const fs::path& objectFile, TestObject& object);
};


