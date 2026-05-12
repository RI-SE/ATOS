/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "truckobjectcontrol.hpp"
#include <csignal>

int main(int argc, char** argv) {
	std::signal(SIGPIPE, SIG_IGN);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TruckObjectControl>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
