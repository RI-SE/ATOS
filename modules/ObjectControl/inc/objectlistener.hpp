#pragma once
#include "positioning.h"
#include "objectcontrol.hpp"
#include "testobject.hpp"
#include "state.hpp"
#include "loggable.hpp"
#include <thread>

class ObjectControl;
class ObjectControlState;

class ObjectListener : public Loggable
{
public:
	ObjectListener(ObjectControl*, TestObject*, rclcpp::Logger);
	~ObjectListener();
private:
	TestObject* obj;
	ObjectControl* handler;
	std::thread listener;

	void listen();
	bool quit = false;
};

