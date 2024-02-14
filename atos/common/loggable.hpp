/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <rclcpp/logging.hpp>

class Loggable {
public:
    Loggable(rclcpp::Logger lg) : logger(lg) {}
    rclcpp::Logger get_logger() const {
        return logger;
    }
protected:
    rclcpp::Logger logger;
};