/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "integrationtestingfactory.hpp"

/**
 * @brief Factory class for creating integration tests.
 * 
 */
IntegrationTestingFactory::IntegrationTestingFactory() {}


/**
 * @brief Destructor.
 * 
 */
IntegrationTestingFactory::~IntegrationTestingFactory() {}


/**
 * @brief Create an integration test object.
 * 
 * @param testName Which integration test to create.
 * @return std::shared_ptr<IntegrationTesting> New integration test object.
 */
std::shared_ptr<IntegrationTesting> IntegrationTestingFactory::createIntegrationTestExecution(const std::string& testName) {
	if (testName == ScenarioExecution::testName) {
		return std::make_shared<ScenarioExecution>();
	}
	else {
		throw std::runtime_error("Unknown integration test: " + testName);
	}
}