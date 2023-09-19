/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "scenarioexecution.hpp"


using namespace std::chrono_literals;

ScenarioExecution::ScenarioExecution() : IntegrationTesting(moduleName) {
	RCLCPP_INFO(get_logger(), "Started integration test: %s", moduleName.c_str());
	getObjectControlStateClient = this->create_client<atos_interfaces::srv::GetObjectControlState>("/atos/get_object_control_state");
	runScenario();
}

ScenarioExecution::~ScenarioExecution() {}


void ScenarioExecution::runScenario() {
	RCLCPP_INFO(get_logger(), "Running scenario");
	auto msg = std_msgs::msg::Empty();

	initPub->publish(msg);
	auto state = getObjectControlState();
	auto isStateCorrect = checkState(initTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
	}

	// sleep to allow the object to initialize
	std::this_thread::sleep_for(std::chrono::seconds(2));
	connectPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(connectTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
	}

	// sleep to allow the object to connect
	std::this_thread::sleep_for(std::chrono::seconds(2));
	armPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(connectTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
	}

	// sleep to allow the object to arm
	std::this_thread::sleep_for(std::chrono::seconds(2));
	startPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(connectTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
	}
}