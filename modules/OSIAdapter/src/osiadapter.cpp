#include <iostream>
#include <chrono>

#include "util.h"
#include "osiadapter.hpp"


using namespace ROSChannels;

OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName)
  {
    initialize();
  };


int OSIAdapter::initialize() {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());
  RCLCPP_INFO(get_logger(), "AAAAAAAAAAAAAAAAAAAAAAAAAAA");
  return 0;
}

void OSIAdapter::sendPosition() {
  RCLCPP_INFO(get_logger(), "Sending position - testing!");
}

void OSIAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}