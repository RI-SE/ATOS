#include "rclcpp/rclcpp.hpp"
#include "restbridge.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto sm = std::make_shared<RESTBridge>();
  rclcpp::spin(sm);
  rclcpp::shutdown();
  return 0;
}
