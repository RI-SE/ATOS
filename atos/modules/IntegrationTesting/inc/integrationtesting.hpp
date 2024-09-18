/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "roschannels/commandchannels.hpp"
#include "atos_interfaces/srv/get_object_control_state.hpp"


class IntegrationTesting : public Module {

	public:
		IntegrationTesting(const std::string& moduleName);
		~IntegrationTesting();

		static inline std::string const testName = "scenario_execution";
		virtual void runIntegrationTest() = 0;

	protected:
		const std::string atosNamespace = "/atos/";
	  const std::string initTopic = atosNamespace + ROSChannels::Init::topicName;
	  const std::string connectTopic = atosNamespace + ROSChannels::Connect::topicName;
	  const std::string armTopic = atosNamespace + ROSChannels::Arm::topicName;
	  const std::string startTopic = atosNamespace + ROSChannels::Start::topicName;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> initPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> connectPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> armPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> startPub;
		std::shared_ptr<rclcpp::Client<atos_interfaces::srv::GetObjectControlState>> getObjectControlStateClient;
		std::vector<std::pair<int, int>> stateResult;

		virtual void printResult() = 0;
		int getObjectControlState();
		void checkState(const std::string& command);
};