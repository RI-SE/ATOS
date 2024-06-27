#include "rclcpp/rclcpp.hpp"
#include "scenariomodule.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto sm = std::make_shared<ScenarioModule>();
  rclcpp::spin(sm);
  rclcpp::shutdown();
  return 0;
}
