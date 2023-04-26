/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esminiadapter.hpp"


static std::shared_ptr<EsminiAdapter> esminiAdapter;

int main(int argc, char** argv) {
	rclcpp::init(argc,argv);
	esminiAdapter = EsminiAdapter::instance();
	esminiAdapter->initializeModule();
	rclcpp::spin(esminiAdapter);
	rclcpp::shutdown();
	return 0;
}
