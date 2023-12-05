/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"
#include <rclcpp/executor.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	auto obc = std::make_shared<ObjectControl>(exec);
	exec->add_node(obc);
	exec->spin();
	rclcpp::shutdown();

	return 0;
}
