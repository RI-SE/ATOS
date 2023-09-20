/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "scenarioexecution.hpp"
#include "rclcpp/wait_for_message.hpp"


using namespace std::chrono_literals;

ScenarioExecution::ScenarioExecution() : IntegrationTesting(moduleName) {
	RCLCPP_INFO(get_logger(), "Started integration test: %s", moduleName.c_str());
	getObjectTrajectoryClient = this->create_client<atos_interfaces::srv::GetObjectTrajectory>("/atos/get_object_trajectory");
	runIntegrationTest();
}

ScenarioExecution::~ScenarioExecution() {}


void ScenarioExecution::runIntegrationTest() {
	RCLCPP_INFO(get_logger(), "Running scenario");
	auto msg = std_msgs::msg::Empty();

	initPub->publish(msg);
	auto state = getObjectControlState();
	auto isStateCorrect = checkState(initTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
		RCLCPP_ERROR(get_logger(), "State is %d", state);
	}

	// sleep to allow the object to initialize
	std::this_thread::sleep_for(std::chrono::seconds(1));
	connectPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(connectTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
		RCLCPP_ERROR(get_logger(), "State is %d", state);
	}

	// sleep to allow the object to connect
	std::this_thread::sleep_for(std::chrono::seconds(1));
	armPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(armTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
		RCLCPP_ERROR(get_logger(), "State is %d", state);
	}

	// sleep to allow the object to arm
	std::this_thread::sleep_for(std::chrono::seconds(1));
	startPub->publish(msg);
	state = getObjectControlState();
	isStateCorrect = checkState(startTopic, state);
	if (!isStateCorrect) {
		RCLCPP_ERROR(get_logger(), "State is not correct");
		RCLCPP_ERROR(get_logger(), "State is %d", state);
	}

	auto trajectory = getTrajectoryPoints();
	std::pair<double, double> lastPoint;
	auto isObjectMoving = true;
	while(isObjectMoving) {
		atos_interfaces::msg::Monitor monr;
		monitorSub = this->create_subscription<atos_interfaces::msg::Monitor>("/atos/object_1/object_monitor", 10, std::bind(&ScenarioExecution::monitorCallback, this, std::placeholders::_1));
		rclcpp::wait_for_message(monr, monitorSub, this->get_node_base_interface()->get_context(), 50ms);
		std::cerr << "POINT: " << monr.pose.pose.position.x << " " << monr.pose.pose.position.y << std::endl;
		if (monr.velocity.twist.linear.x == 0 && monr.velocity.twist.linear.y == 0) {
			lastPoint = std::make_pair(monr.pose.pose.position.x, monr.pose.pose.position.y);
			isObjectMoving = false;
		}
	}
}	


std::vector<std::pair<double, double>> ScenarioExecution::getTrajectoryPoints() {
	auto request = std::make_shared<atos_interfaces::srv::GetObjectTrajectory::Request>();
	request->id = 1;
	std::shared_ptr<atos_interfaces::srv::GetObjectTrajectory::Response> response;
	this->callService(1000ms, getObjectTrajectoryClient, request, response);

	std::vector<std::pair<double, double>> trajectory;
	for (const auto& t : response->trajectory.points) {
		trajectory.push_back(std::make_pair(t.pose.position.x, t.pose.position.y));
	}
	return trajectory;
}


void ScenarioExecution::monitorCallback(const atos_interfaces::msg::Monitor::SharedPtr msg) {}