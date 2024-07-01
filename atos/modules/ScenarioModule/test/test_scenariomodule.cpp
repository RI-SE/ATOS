#include "rclcpp/rclcpp.hpp"
#include "scenariomodule.hpp"
#include "testUtils/testUtils.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <vector>
class ScenarioModuleTest : public ::testing::Test {
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

protected:
  std::shared_ptr<ScenarioModule> scenarioModule;
  std::shared_ptr<rclcpp::Node> helperNode;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() override {
    helperNode = rclcpp::Node::make_shared("ScenarioModuleTestHelper_node");
    scenarioModule = std::make_shared<ScenarioModule>();
    executor.add_node(helperNode);
    executor.add_node(scenarioModule);
  }

  void TearDown() override {
    executor.remove_node(helperNode);
    executor.remove_node(scenarioModule);
    helperNode.reset();
    scenarioModule.reset();
  }
};

TEST_F(ScenarioModuleTest, objectIdsIsEmptyByDefault) {
  // Setup
  auto failAfterTimeout = std::chrono::milliseconds(1000);

  auto getObjectIdsClient = helperNode->create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds);
  testUtils::waitForService(getObjectIdsClient, failAfterTimeout);

  // Act
  auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();
  auto result = getObjectIdsClient->async_send_request(request);
  executor.spin_until_future_complete(result, failAfterTimeout);
  auto response = result.get();

  // Assert
  ASSERT_EQ(response->success, false);
  ASSERT_EQ(response->ids, std::vector<uint32_t>());
}

TEST_F(ScenarioModuleTest, objectIdsCanBeSetAfterModuleConstruction) {
  // Setup
  auto failAfterTimeout = std::chrono::milliseconds(1000);

  // Set ros parameters for object ids for the scenario module
  std::vector<long> objectIds = {1, 2, 3};
  // Convert to uint32_t since the service returns uint32_t
  std::vector<uint32_t> objectIdsUint32;
  objectIdsUint32.reserve(objectIds.size());
  std::transform(objectIds.begin(), objectIds.end(), std::back_inserter(objectIdsUint32),
                 [](long id) { return static_cast<uint32_t>(id); });

  auto res = scenarioModule->set_parameter(rclcpp::Parameter("active_object_ids", objectIds));
  if (!res.successful) {
    FAIL() << "Failed to set active object ids parameter, reason: " << res.reason;
  }

  auto getObjectIdsClient = helperNode->create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds);
  testUtils::waitForService(getObjectIdsClient, failAfterTimeout);

  // Act
  auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();
  auto result = getObjectIdsClient->async_send_request(request);
  executor.spin_until_future_complete(result, failAfterTimeout);
  auto response = result.get();

  // Assert
  ASSERT_EQ(response->success, true);
  ASSERT_EQ(response->ids, objectIdsUint32);
}