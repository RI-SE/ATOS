#include "rclcpp/rclcpp.hpp"
#include "restbridge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto rb = std::make_shared<RESTBridge>();
  rclcpp::spin(rb);
  rclcpp::shutdown();
  return 0;
}
