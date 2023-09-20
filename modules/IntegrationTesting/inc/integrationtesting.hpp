/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "atos_interfaces/srv/get_object_control_state.hpp"


class IntegrationTesting : public Module {

	public:
		IntegrationTesting(const std::string& moduleName);
		~IntegrationTesting();

		virtual void runIntegrationTest() = 0;

	protected:
		static inline std::string const moduleName = "scenario_execution";
	  static inline std::string const initTopic = "/atos/init";
	  static inline std::string const connectTopic = "/atos/connect";
	  static inline std::string const armTopic = "/atos/arm";
	  static inline std::string const startTopic = "/atos/start";
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> initPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> connectPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> armPub;
		std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty>> startPub;
		std::shared_ptr<rclcpp::Client<atos_interfaces::srv::GetObjectControlState>> getObjectControlStateClient;

		
		int getObjectControlState();
		void checkState(const std::string& command);


};