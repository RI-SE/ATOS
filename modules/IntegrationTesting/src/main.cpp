/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "scenarioexecution.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto scenarioExecutionNode = std::make_shared<ScenarioExecution>();
	rclcpp::spin(scenarioExecutionNode);
	rclcpp::shutdown();
	return 0;
}