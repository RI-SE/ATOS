/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "integrationtestingfactory.hpp"

IntegrationTestingFactory::IntegrationTestingFactory() {}

IntegrationTestingFactory::~IntegrationTestingFactory() {}

std::shared_ptr<IntegrationTesting> IntegrationTestingFactory::createIntegrationTestExecution() {
	return std::make_shared<ScenarioExecution>();
}