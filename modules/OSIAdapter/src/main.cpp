#include <iostream>
#include "osiadapter.hpp"


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto OSIAdapterNode = std::make_shared<OSIAdapter>();
  rclcpp::spin(OSIAdapterNode);
  rclcpp::shutdown();

  return 0;
}