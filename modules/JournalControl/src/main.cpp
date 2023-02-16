/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "util.h"
#include "rclcpp/rclcpp.hpp"
#include "journalcontrol.hpp"

int main(int argc, char **argv) {
	rclcpp::init(argc,argv);
	auto node = std::make_shared<JournalControl>();
	rclcpp::spin(node);
	return 0;
}