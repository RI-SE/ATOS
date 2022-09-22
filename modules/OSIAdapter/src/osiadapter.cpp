#include <iostream>
#include <chrono>

#include "util.h"
#include "osiadapter.hpp"


using namespace ROSChannels;
using namespace std::chrono_literals;

OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName)
  {
    initialize();
    publisher = this->create_publisher<std_msgs::msg::String>("position", 10);
    timer = this->create_wall_timer(500ms, std::bind(&OSIAdapter::sendPosition, this));  
  };


int OSIAdapter::initialize() {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());
  return 0;
}

void OSIAdapter::sendPosition() {
  RCLCPP_INFO(get_logger(), "Sending position - testing!");


}

void OSIAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}