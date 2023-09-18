/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "integrationtesting.hpp"
#include "scenarioexecution.hpp"


IntegrationTesting::IntegrationTesting() : Module(moduleName) {
	getIntegrationTests();
	executeIntegrationTests();
}

IntegrationTesting::~IntegrationTesting() {}


void IntegrationTesting::getIntegrationTests() {
	declare_parameter("scenario_execution", false);
	get_parameter("scenario_execution", integrationTests["scenario_execution"]);
}

void IntegrationTesting::executeIntegrationTests() {
	for (auto const& [testName, testEnabled] : integrationTests) {
		if (testEnabled) {
			RCLCPP_INFO(get_logger(), "Executing integration test: %s", testName.c_str());
			auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
			executor->add_node(std::make_shared<ScenarioExecution>());
			executor->spin_once();
		}
	}
	RCLCPP_INFO(get_logger(), "Finished executing integration tests");
}