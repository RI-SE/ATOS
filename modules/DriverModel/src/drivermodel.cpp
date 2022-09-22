#include <iostream>

#include "util.h"
#include "drivermodel.hpp"


using namespace ROSChannels;

DriverModel::DriverModel() :
  Module(DriverModel::moduleName)
  {
    initialize();
  };


int DriverModel::initialize() {
  
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());

  return 0;
}

void DriverModel::onAbortMessage(const Abort::message_type::SharedPtr) {

}