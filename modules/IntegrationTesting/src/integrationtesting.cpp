/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "integrationtesting.hpp"


IntegrationTesting::IntegrationTesting() : Module(moduleName) {
	RCLCPP_INFO(get_logger(), "Initializing module %s", moduleName.c_str());
	getIntegrationTests();
}

IntegrationTesting::~IntegrationTesting() {}


void IntegrationTesting::getIntegrationTests() {
	declare_parameter("scenario_execution", false);
	get_parameter("scenario_execution", integrationTests["scenario_execution"]);
}