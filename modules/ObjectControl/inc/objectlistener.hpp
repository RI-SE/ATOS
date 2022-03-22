#pragma once
#include "positioning.h"
#include "objectcontrol.hpp"
#include "testobject.hpp"
#include "state.hpp"
#include "loggable.hpp"
#include "roschannel.hpp"
#include <thread>

class ObjectControl;
class ObjectControlState;

class ObjectListener : public Loggable
{
public:
	ObjectListener(ObjectControl*, TestObject*, ROSChannels::Monitor::Pub&, rclcpp::Logger);
	~ObjectListener();
private:
	TestObject* obj;
	ObjectControl* handler;
	ROSChannels::Monitor::Pub& monitorChannel;
	std::thread listener;

	void listen();
	bool quit = false;
};

