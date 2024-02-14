/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

using namespace std_msgs::msg;
using namespace std_srvs::srv;

bool Module::shouldExit(){
    return this->quit;
}

void Module::onExitMessage(const Empty::SharedPtr){
    this->quit=true;
}

/*!
 * \brief A try/catch wrapper that logs messages 
 * \param tryExecute function to execute
 * \param executeIfFail if executing tryExecute fails, this function is executed
 * \param topic The topic to print.
 * \param logger The logger to use.
 * \return true if the initialization was successful, false otherwise
 */
void Module::tryHandleMessage(
    std::function<void()> tryExecute,
    std::function<void()> executeIfFail,
    const std::string& topic,
    const rclcpp::Logger& logger)
{
	try {
		RCLCPP_DEBUG(logger, "Handling command on %s", topic.c_str());
		tryExecute();
	} catch (std::invalid_argument& e) {
		RCLCPP_ERROR(logger, "Handling command on %s failed - %s", topic.c_str(), e.what());
		executeIfFail();
	}
}
