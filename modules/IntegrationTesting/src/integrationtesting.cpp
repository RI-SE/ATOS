/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "integrationtesting.hpp"
#include "util.h"

using namespace std::chrono_literals;

IntegrationTesting::IntegrationTesting(const std::string& moduleName) : Module(moduleName) {
	initPub = this->create_publisher<std_msgs::msg::Empty>(initTopic, 10);
	connectPub = this->create_publisher<std_msgs::msg::Empty>(connectTopic, 10);
	armPub = this->create_publisher<std_msgs::msg::Empty>(armTopic, 10);
	startPub = this->create_publisher<std_msgs::msg::Empty>(startTopic, 10);
	getObjectControlStateClient = this->create_client<atos_interfaces::srv::GetObjectControlState>("/atos/get_object_control_state");
}

IntegrationTesting::~IntegrationTesting() {}


int IntegrationTesting::getObjectControlState() {
	std::shared_ptr<atos_interfaces::srv::GetObjectControlState::Response> response;
	this->callService(1000ms, getObjectControlStateClient, response);
	return int(response->state);
}


bool IntegrationTesting::checkState(const std::string& command, int& state) {
	if (command == initTopic) {
		return state == OBCState_t::OBC_STATE_INITIALIZED;
	}
	else if (command == connectTopic) {
		return state == OBCState_t::OBC_STATE_CONNECTED;
	}
	else if (command == armTopic) {
		return state == OBCState_t::OBC_STATE_ARMED;
	}
	else if (command == startTopic) {
		return state == OBCState_t::OBC_STATE_RUNNING;
	}
	else {
		return false;
	}
}