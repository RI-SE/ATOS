/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "scenarioexecution.hpp"


ScenarioExecution::ScenarioExecution() : Module(moduleName) {
	RCLCPP_INFO(get_logger(), "Initializing module %s", moduleName.c_str());
}

ScenarioExecution::~ScenarioExecution() {}
