#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tests/rclcpp_utils.hpp"
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
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
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
  // Setup
  auto sampleModule = std::make_shared<SampleModule>();
  auto helperNode = rclcpp::Node::make_shared("testOnAbortCallback_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sampleModule);
  executor.add_node(helperNode);
  auto abortPublisher = ROSChannels::Abort::Pub(*helperNode);
  
  // Act
  abortPublisher.publish(ROSChannels::Abort::message_type());
  executor.spin_some(std::chrono::milliseconds(300));

  // Assert
  ASSERT_EQ(sampleModule->getAborting(), true);
}

TEST_F(SampleModuleTest, testOnInitResponse){
  // Setup
  auto sampleModule = std::make_shared<SampleModule>();
  auto helperNode = rclcpp::Node::make_shared("testOnInitResponse_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sampleModule);
  executor.add_node(helperNode);

  std::promise<void> sub_called;
  std::shared_future<void> sub_called_future(sub_called.get_future());
  auto fail_after_timeout = std::chrono::milliseconds(5000);

  bool receivedMsg = false; 
  auto smOnInitResponseCallback = [&receivedMsg, &sub_called](const ROSChannels::SmOnInitResponse::message_type::SharedPtr msg) {
    receivedMsg = true;
    sub_called.set_value();
  };

  std::string topicname = std::string(sampleModule->get_namespace()) + ROSChannels::SmOnInitResponse::topicName;
  auto sub = ROSChannels::SmOnInitResponse::Sub(*helperNode, smOnInitResponseCallback);

  // Act
  auto initPublisher = ROSChannels::Init::Pub(*helperNode);
  initPublisher.publish(ROSChannels::Init::message_type());
  // wait for discovery and the subscriber to connect
  test_rclcpp::wait_for_subscriber(helperNode, topicname);
  test_rclcpp::wait_for_future(executor, sub_called_future, fail_after_timeout);

  // Assert
  ASSERT_EQ(receivedMsg, true);
}