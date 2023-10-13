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

  std::shared_ptr<SampleModule> sampleModule;
  std::shared_ptr<rclcpp::Node> helperNode;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() override
  {
    helperNode = rclcpp::Node::make_shared("SampleModuleTestHelper_node");
    sampleModule = std::make_shared<SampleModule>();
    executor.add_node(helperNode);
    executor.add_node(sampleModule);
  }

  void TearDown() override
  {
    executor.remove_node(helperNode);
    executor.remove_node(sampleModule);
    helperNode.reset();
    sampleModule.reset();
  }
};

TEST_F(SampleModuleTest, testReceivesAbortMessage){
  ASSERT_EQ(sampleModule->getAborting(), false);
}

TEST_F(SampleModuleTest, testGetObjectIds){
  std::vector<std::uint32_t> objectIds = sampleModule->getObjectIds();
  ASSERT_EQ(objectIds.size(), 0);
}

TEST_F(SampleModuleTest, testOnAbortCallback){
  // Setup
  auto abortPublisher = ROSChannels::Abort::Pub(*helperNode);
  
  // Act
  abortPublisher.publish(ROSChannels::Abort::message_type());
  executor.spin_some(std::chrono::milliseconds(300));

  // Assert
  ASSERT_EQ(sampleModule->getAborting(), true);
}

TEST_F(SampleModuleTest, testOnInitResponse){
  // Setup
  std::promise<void> sub_called;
  std::shared_future<void> sub_called_future(sub_called.get_future());
  auto fail_after_timeout = std::chrono::milliseconds(5000);

  bool receivedMsg = false; 
  auto smOnInitResponseCallback = [&receivedMsg, &sub_called](const ROSChannels::SampleModuleTestForInitResponse::message_type::SharedPtr msg) {
    receivedMsg = true;
    sub_called.set_value();
  };

  std::string topicname = std::string(sampleModule->get_namespace()) + ROSChannels::SampleModuleTestForInitResponse::topicName;
  auto sub = ROSChannels::SampleModuleTestForInitResponse::Sub(*helperNode, smOnInitResponseCallback);

  // Act
  auto initPublisher = ROSChannels::Init::Pub(*helperNode);
  initPublisher.publish(ROSChannels::Init::message_type());
  // wait for discovery and the subscriber to connect
  test_rclcpp::wait_for_subscriber(helperNode, topicname);
  test_rclcpp::wait_for_future(executor, sub_called_future, fail_after_timeout);

  // Assert
  ASSERT_EQ(receivedMsg, true);
}

TEST_F(SampleModuleTest, testServiceCalled){
  // Setup
  std::promise<void> service_called;
  std::shared_future<void> service_called_future(service_called.get_future());
  auto fail_after_timeout = std::chrono::milliseconds(1000);
  bool receivedReq = false; 

  auto getObjectIdsClient = helperNode->create_client<std_srvs::srv::SetBool>("/sample_module_test_service");
  auto getObjectIdsRequest = std::make_shared<std_srvs::srv::SetBool::Request>();

  auto testServiceCalledCallback = [&receivedReq, &service_called](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
    receivedReq = response.get()->success;
    service_called.set_value();
  };

  // Act
  getObjectIdsClient->async_send_request(getObjectIdsRequest, testServiceCalledCallback);
  // wait for the service to be available and for it to process the request
  test_rclcpp::wait_for_service(getObjectIdsClient, std::chrono::milliseconds(1000));
  test_rclcpp::wait_for_future(executor, service_called_future, fail_after_timeout);

  // Assert
  ASSERT_EQ(receivedReq, true);
}