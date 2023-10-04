/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "backtostart.hpp"


int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	static std::shared_ptr<BackToStart> bts_node = std::make_shared<BackToStart>();
	rclcpp::spin(bts_node);
	return 0;
}
