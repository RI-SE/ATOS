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