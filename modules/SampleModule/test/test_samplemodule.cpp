#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "samplemodule.hpp"

class SampleModuleTest : public SampleModule {
    public:
    // Function that calls the private onAbortMessage function
    void callOnAbortMessage(const ROSChannels::Abort::message_type::SharedPtr msg) {
        onAbortMessage(msg);
    }
};

TEST(SampleModulePkg, testGetObjectIds){
  
  auto sm =  std::make_shared<SampleModule>();
  std::vector<std::uint32_t> objectIds = sm->getObjectIds();
  EXPECT_EQ(objectIds.size(), 0);
}

TEST(SampleModulePkg, testReceivesAbortMessage){

  auto sm =  std::make_shared<SampleModuleTest>();
  auto msg = std::make_shared<std_msgs::msg::Empty>();
  EXPECT_EQ(sm->getAborting(), false);

  sm->callOnAbortMessage(msg);

  EXPECT_EQ(sm->getAborting(), false);
}