/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "amqpbridge.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);

	auto ab = std::make_shared<AmqpBridge>();
	if(rclcpp::ok()) {
		rclcpp::spin(ab);
	}
	rclcpp::shutdown();
	return 0;
}
