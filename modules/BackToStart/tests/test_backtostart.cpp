#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tests/rclcpp_utils.hpp"
#include "atos_interfaces/srv/get_object_return_trajectory.hpp"
#include "atos_interfaces/msg/cartesian_trajectory_point.hpp"
#include "backtostart.hpp"

class BackToStartTest : public ::testing::Test {
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


TEST_F(BackToStartTest, testServiceCallWithoutTrajectory){
  // Setup
  auto backToStart = std::make_shared<BackToStart>();
  auto helperNode = rclcpp::Node::make_shared("testGetObjectReturnTrajectoryService_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backToStart);
  executor.add_node(helperNode);

  std::promise<void> service_called;
  std::shared_future service_called_future(service_called.get_future());
  auto fail_after_timeout = std::chrono::milliseconds(5000);

  bool receivedResponse = false; 
  auto returnTrajResponseCallback = [&receivedResponse, &service_called](const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future) {
    receivedResponse = future.get()->success;
    service_called.set_value();
  };

  std::string serviceName = ServiceNames::getObjectReturnTrajectory;
  rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr returnTrajectoryClient;
  returnTrajectoryClient = helperNode->create_client<atos_interfaces::srv::GetObjectReturnTrajectory>(serviceName);
  
  auto returnTrajectoryRequest = std::make_shared<atos_interfaces::srv::GetObjectReturnTrajectory::Request>();

  //  wait for service
  while (!returnTrajectoryClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Act
  returnTrajectoryClient->async_send_request(returnTrajectoryRequest, returnTrajResponseCallback);
  // wait for service to respond and callback to execute
  test_rclcpp::wait_for_future(executor, service_called_future, fail_after_timeout);

  // Assert
  ASSERT_EQ(receivedResponse, false);
}

TEST_F(BackToStartTest, testServiceCallWithTrajectory){
  // Setup
  auto backToStart = std::make_shared<BackToStart>();
  auto helperNode = rclcpp::Node::make_shared("testGetObjectReturnTrajectoryService_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backToStart);
  executor.add_node(helperNode);

  std::promise<void> service_called;
  std::shared_future service_called_future(service_called.get_future());
  auto fail_after_timeout = std::chrono::milliseconds(5000);

  bool receivedResponse = false; 
  auto returnTrajResponseCallback = [&receivedResponse, &service_called](const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future) {
    receivedResponse = future.get()->success;
    service_called.set_value();
  };

  std::string serviceName = ServiceNames::getObjectReturnTrajectory;
  rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr returnTrajectoryClient;
  returnTrajectoryClient = helperNode->create_client<atos_interfaces::srv::GetObjectReturnTrajectory>(serviceName);

  auto returnTrajectoryRequest = std::make_shared<atos_interfaces::srv::GetObjectReturnTrajectory::Request>();
  for (int i = 0; i < 10; i++) {
    atos_interfaces::msg::CartesianTrajectoryPoint trajPoint;
    trajPoint.time_from_start.sec = i;
    trajPoint.time_from_start.nanosec = i;
    trajPoint.pose.position.x = i;
    trajPoint.pose.position.y = i;
    trajPoint.pose.position.z = i;
    trajPoint.twist.linear.x = i;
    trajPoint.twist.linear.y = 0;
    trajPoint.twist.linear.z = 0;
    returnTrajectoryRequest->trajectory.points.push_back(trajPoint);
  }

  //  wait for service
  while (!returnTrajectoryClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Act
  returnTrajectoryClient->async_send_request(returnTrajectoryRequest, returnTrajResponseCallback);
  // wait for service to respond and callback to execute
  test_rclcpp::wait_for_future(executor, service_called_future, fail_after_timeout);

  // Assert
  ASSERT_EQ(receivedResponse, true);
}