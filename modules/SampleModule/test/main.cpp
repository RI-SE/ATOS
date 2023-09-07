#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}