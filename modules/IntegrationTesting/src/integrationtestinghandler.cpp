/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "integrationtestingfactory.hpp"
#include "integrationtestinghandler.hpp"
#include "scenarioexecution.hpp"


IntegrationTestingHandler::IntegrationTestingHandler() : Module(moduleName) {
	getIntegrationTests();
	executeIntegrationTests();
}

IntegrationTestingHandler::~IntegrationTestingHandler() {}


void IntegrationTestingHandler::getIntegrationTests() {
	declare_parameter("scenario_execution", false);
	get_parameter("scenario_execution", integrationTests["scenario_execution"]);
}

void IntegrationTestingHandler::executeIntegrationTests() {
	for (auto const& [testName, testEnabled] : integrationTests) {
		if (testEnabled) {
			auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
			auto integrationTestFactory = std::make_shared<IntegrationTestingFactory>();
			auto integrationTest = integrationTestFactory->createIntegrationTestExecution();
			executor->add_node(integrationTest);
			executor->spin_once();
		}
	}
	RCLCPP_INFO(get_logger(), "Finished executing integration tests");
}