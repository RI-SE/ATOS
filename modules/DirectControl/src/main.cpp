/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "util.h"
#include "datadictionary.h"
#include "directcontrol.hpp"

using namespace std::chrono;

static std::shared_ptr<DirectControl> dc;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	dc = std::make_shared<DirectControl>();
	dc->initializeModule();
	dc->startThreads();
	rclcpp::spin(dc);
	dc->joinThreads();
	rclcpp::shutdown();
	return 0;
}
