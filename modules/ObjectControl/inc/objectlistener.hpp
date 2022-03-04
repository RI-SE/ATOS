#pragma once
#include "positioning.h"
#include "objectcontrol.hpp"
#include "testobject.hpp"
#include "state.hpp"
#include <thread>

class ObjectControl;
class ObjectControlState;

class ObjectListener
{
public:
	ObjectListener(ObjectControl*, TestObject*);
	~ObjectListener();
private:
	TestObject* obj;
	ObjectControl* handler;
	std::thread listener;

	void listen();
	bool quit = false;
};

