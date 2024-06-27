#include "rclcpp/rclcpp.hpp"
#include "scenariomodule.hpp"
#include "testUtils/testUtils.hpp"
#include "gtest/gtest.h"
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

TEST_F(ScenarioModuleTest, placeholder) { ASSERT_EQ(true, true); }