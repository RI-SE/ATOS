/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
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

