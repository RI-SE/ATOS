#pragma once

#include "scenariohandler.hpp"
#include "testobject.hpp"
#include "state.hpp"
#include <thread>

class ScenarioHandler;
class ObjectControlState;

class ObjectListener
{
public:
	ObjectListener(ScenarioHandler*, TestObject*);
	~ObjectListener();
private:
	TestObject* obj;
	ScenarioHandler* handler;
	std::thread listener;

	void listen();
	bool quit = false;
};

