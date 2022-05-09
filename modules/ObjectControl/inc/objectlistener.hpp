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
	ObjectListener(
		ObjectControl*,
		std::shared_ptr<TestObject>,
		rclcpp::Logger
	);
	~ObjectListener();
private:
	std::shared_ptr<TestObject> obj;
	ObjectControl* handler;
	std::thread listener;

	void listen();
	bool quit = false;
};

