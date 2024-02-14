/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "integrationtesting.hpp"
#include "scenarioexecution.hpp"


class IntegrationTestingFactory {

	public:
		IntegrationTestingFactory();
		~IntegrationTestingFactory();

		std::shared_ptr<IntegrationTesting> createIntegrationTestExecution(const std::string& testName);
};