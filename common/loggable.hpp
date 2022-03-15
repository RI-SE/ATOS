#pragma once
#include <rclcpp/logging.hpp>

class Loggable {
public:
    Loggable(rclcpp::Logger lg) : logger(lg) {}
protected:
    rclcpp::Logger logger;
    rclcpp::Logger get_logger() const {
        return logger;
    }
};