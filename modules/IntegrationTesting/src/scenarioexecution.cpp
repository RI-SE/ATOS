/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <iomanip>
#include "scenarioexecution.hpp"
#include "rclcpp/wait_for_message.hpp"

using namespace std::chrono_literals;

/**
 * @brief An integration test for scenario execution. This test will run a scenario execution for one object,
 * and check that all states are correct and that it follows the trajectory.
 * 
 */
ScenarioExecution::ScenarioExecution() : IntegrationTesting(testName) {
	RCLCPP_INFO(get_logger(), "Started integration test: %s", testName.c_str());
	getObjectTrajectoryClient = this->create_client<atos_interfaces::srv::GetObjectTrajectory>("/atos/get_object_trajectory");
	runIntegrationTest();
}


/**
 * @brief Destructor.
 * 
 */
ScenarioExecution::~ScenarioExecution() {}


/**
 * @brief Run the integration test. This integration test will initialize a scenario, connect to object 1,
 * arm the object, and then start it. It will check that all states are correct, and that the object follows
 * the trajectory.
 * 
 */
void ScenarioExecution::runIntegrationTest() {
	RCLCPP_INFO(get_logger(), "Running scenario");
	auto msg = std_msgs::msg::Empty();

	initPub->publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep to allow the object to initialize
	checkState(initTopic);
	
	connectPub->publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep to allow the object to connect
	checkState(connectTopic);

	armPub->publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep to allow the object to arm
	checkState(armTopic);

	startPub->publish(msg);
	std::this_thread::sleep_for(std::chrono::seconds(1)); // sleep to allow the object to start
	checkState(startTopic);

	checkTrajectory();
	printResult();
}	


/**
 * @brief Get the x- and y-coordinates of the trajectory points for object 1.
 * 
 * @return std::vector<std::pair<double, double>> Trajectory points.
 */
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


/**
 * @brief Check that the object ends up near the end point of the trajectory.
 * 
 */
void ScenarioExecution::checkTrajectory() {
	auto trajectory = getTrajectoryPoints();
	std::pair<double, double> lastPoint;
	auto isObjectMoving = true;
	while(isObjectMoving) {
		atos_interfaces::msg::Monitor monr;
		monitorSub = this->create_subscription<atos_interfaces::msg::Monitor>("/atos/object_1/object_monitor", 10, std::bind(&ScenarioExecution::placeholderCallback, this, std::placeholders::_1));
		rclcpp::wait_for_message(monr, monitorSub, this->get_node_base_interface()->get_context(), 50ms);
		if (monr.velocity.twist.linear.x == 0 && monr.velocity.twist.linear.y == 0) {
			lastPoint = std::make_pair(monr.pose.pose.position.x, monr.pose.pose.position.y);
			isObjectMoving = false;
		}
	}

	auto distanceX = lastPoint.first - trajectory.back().first;
	auto distanceY = lastPoint.second - trajectory.back().second;
	auto distanceThreshold = 1.0;
	if (abs(distanceX) < distanceThreshold && abs(distanceY) < distanceThreshold) {
		followedTrajectory = true;
		RCLCPP_INFO(get_logger(), "Object followed trajectory succesfully");
	}
	else {
		followedTrajectory = false;
		RCLCPP_ERROR(get_logger(), "Object did not follow trajectory succesfully. Distance from end point of trajectory is x: %f, y: %f", distanceX, distanceY);
	}
}


/**
 * @brief Print the result of the integration test.
 * 
 */
void ScenarioExecution::printResult() {
	std::stringstream ss;
	auto width = 20;
	ss << "State change results:\n";
	ss << std::setfill('-') << std::setw(width * 3) << "-" << std::endl;
	ss << std::left << std::setfill(' ') << std::setw(width) << "State" 
		 << std::setw(width) << "Expected state" 
		 << std::setw(width) << "Pass" << std::endl;
	ss << std::setfill('-') << std::setw(width * 3) << "-" << std::endl;

	for (auto const& [state, expectedState] : stateResult) {
		auto pass = (state == expectedState) ? "OK" : "NOT OK";
		ss << std::left << std::setfill(' ') << std::setw(width) << state 
			 << std::setw(width) << expectedState 
			 << std::setw(width) << pass << std::endl;
	}
	ss << std::setfill('-') << std::setw(width * 3) << "-" << std::endl;
	ss << std::setfill(' ');

	ss << "\nScenario Execution result: ";
	auto pass = (followedTrajectory) ? "OK" : "NOT OK";
	ss << pass << "\n";

	RCLCPP_INFO(get_logger(), ss.str().c_str());
}


/**
 * @brief This is a placeholder callback function. It is needed in order to create a topic subscriber.
 * 
 * @param msg Message.
 */
void ScenarioExecution::placeholderCallback(const atos_interfaces::msg::Monitor::SharedPtr msg) {}