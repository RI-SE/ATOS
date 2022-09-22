#include <iostream>
#include "drivermodel.hpp"


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto driverModelNode = std::make_shared<DriverModel>();
  rclcpp::spin(driverModelNode);
  rclcpp::shutdown();

  return 0;
}