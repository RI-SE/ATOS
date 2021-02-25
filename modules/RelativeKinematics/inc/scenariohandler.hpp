#pragma once

#include "state.hpp"
#include "testobject.hpp"
#include <map>



class ObjectControlState;

class ScenarioHandler {
	friend class ObjectControlState;
public:
	typedef enum {
		RELATIVE_KINEMATICS,
		ABSOLUTE_KINEMATICS
	} ControlMode;

	ScenarioHandler(ControlMode);
	~ScenarioHandler();

	void loadScenario();
private:
	ObjectControlState* state;
	std::map<uint32_t,TestObject> objects;

	void loadObjectFiles();
	void parseObjectFile(const fs::path& objectFile, TestObject& object);
};


