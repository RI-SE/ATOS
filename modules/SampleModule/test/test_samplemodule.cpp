#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "samplemodule.hpp"

class SampleModuleTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

};

TEST_F(SampleModuleTest, testReceivesAbortMessage){

  auto sm =  std::make_shared<SampleModule>();
  ASSERT_EQ(sm->getAborting(), true);
}

TEST_F(SampleModuleTest, testGetObjectIds){
  
  auto sm =  std::make_shared<SampleModule>();
  std::vector<std::uint32_t> objectIds = sm->getObjectIds();
  ASSERT_EQ(objectIds.size(), 0);
}