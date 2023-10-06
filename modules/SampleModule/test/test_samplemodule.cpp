#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "samplemodule.hpp"
class SampleModuleTest : public ::testing::Test {
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(SampleModuleTest, testReceivesAbortMessage){
  auto sm = std::make_shared<SampleModule>();
  ASSERT_EQ(sm->getAborting(), false);
}

TEST_F(SampleModuleTest, testGetObjectIds){
  auto sm = std::make_shared<SampleModule>();
  std::vector<std::uint32_t> objectIds = sm->getObjectIds();
  ASSERT_EQ(objectIds.size(), 0);
}

TEST_F(SampleModuleTest, testOnAbortCallback){
  auto sm =  std::make_shared<SampleModule>();
  auto node = rclcpp::Node::make_shared("testOnAbortCallback_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sm);
  executor.add_node(node);

  auto abortPublisher = ROSChannels::Abort::Pub(*node);
  std::cout << "Publishing abort message" << std::endl;
  abortPublisher.publish(ROSChannels::Abort::message_type());

  executor.spin_some(std::chrono::milliseconds(200));

  ASSERT_EQ(sm->getAborting(), true);
}