#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "testUtils/testUtils.hpp"
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

TEST_F(SampleModuleTest, testModuleDoesNotAbortOnStartup){
  ASSERT_EQ(sampleModule->getAborting(), false);
}

TEST_F(SampleModuleTest, testObjectIdsEmptyOnStartup){
  std::vector<std::uint32_t> objectIds = sampleModule->getObjectIds();
  ASSERT_EQ((int)objectIds.size(), 0);
}

TEST_F(SampleModuleTest, testSetsAbortingWhenAbortMessagePublished){
  // Setup
  auto abortPublisher = ROSChannels::Abort::Pub(*helperNode);
  
  // Act
  abortPublisher.publish(ROSChannels::Abort::message_type());
  executor.spin_some(std::chrono::milliseconds(300));

  // Assert
  ASSERT_EQ(sampleModule->getAborting(), true);
}

TEST_F(SampleModuleTest, testPublishesOnInitResponseWhenInitIsPublished){
  // Setup
  std::promise<void> subCalled;
  std::shared_future<void> subCalledFuture(subCalled.get_future());
  auto failAfterTimeout = std::chrono::milliseconds(5000);

  bool receivedMsg = false; 
  auto smOnInitResponseCallback = [&receivedMsg, &subCalled](const ROSChannels::SampleModuleTestForInitResponse::message_type::SharedPtr) {
    receivedMsg = true;
    subCalled.set_value();
  };

  std::string topicname = std::string(sampleModule->get_namespace()) + ROSChannels::SampleModuleTestForInitResponse::topicName;
  auto sub = ROSChannels::SampleModuleTestForInitResponse::Sub(*helperNode, smOnInitResponseCallback);

  // Act
  auto initPublisher = ROSChannels::Init::Pub(*helperNode);
  initPublisher.publish(ROSChannels::Init::message_type());
  // wait for discovery and the subscriber to connect
  testUtils::waitForSubscriber(helperNode, topicname);
  testUtils::waitForFuture(executor, subCalledFuture, failAfterTimeout);

  // Assert
  ASSERT_EQ(receivedMsg, true);
}

TEST_F(SampleModuleTest, testThatServiceResponseIsSetToSuccessWhenServiceIsCalled){
  // Setup
  std::promise<void> serviceCalled;
  std::shared_future<void> serviceCalledFuture(serviceCalled.get_future());
  auto failAfterTimeout = std::chrono::milliseconds(1000);
  bool receivedReq = false; 

  auto getObjectIdsClient = helperNode->create_client<std_srvs::srv::SetBool>("/sample_module_test_service");
  auto getObjectIdsRequest = std::make_shared<std_srvs::srv::SetBool::Request>();

  auto testServiceCalledCallback = [&receivedReq, &serviceCalled](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
    receivedReq = response.get()->success;
    serviceCalled.set_value();
  };

  // Act
  getObjectIdsClient->async_send_request(getObjectIdsRequest, testServiceCalledCallback);
  // wait for the service to be available and for it to process the request
  testUtils::waitForService(getObjectIdsClient, std::chrono::milliseconds(1000));
  testUtils::waitForFuture(executor, serviceCalledFuture, failAfterTimeout);

  // Assert
  ASSERT_EQ(receivedReq, true);
}