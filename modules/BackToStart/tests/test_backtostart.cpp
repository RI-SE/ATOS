#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "testUtils/testUtils.hpp"
#include "atos_interfaces/srv/get_object_return_trajectory.hpp"
#include "atos_interfaces/msg/cartesian_trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
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

  std::shared_ptr<BackToStart> backToStart;
  std::shared_ptr<rclcpp::Node> helperNode;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() override
  {
    helperNode = rclcpp::Node::make_shared("BackToStartTestHelper_node");
    backToStart = std::make_shared<BackToStart>();
    executor.add_node(helperNode);
    executor.add_node(backToStart);
  }

  void TearDown() override
  {
    executor.remove_node(helperNode);
    executor.remove_node(backToStart);
    helperNode.reset();
    backToStart.reset();
  }
};


TEST_F(BackToStartTest, testServiceCallWithoutTrajectory){
  // Setup
  std::promise<void> serviceCalled;
  std::shared_future serviceCalledFuture(serviceCalled.get_future());
  auto failAfterTimeout = std::chrono::milliseconds(5000);

  bool receivedResponse = false; 
  auto returnTrajResponseCallback = [&receivedResponse, &serviceCalled](const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future) {
    receivedResponse = future.get()->success;
    serviceCalled.set_value();
  };

  std::string serviceName = ServiceNames::getObjectReturnTrajectory;
  rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr returnTrajectoryClient;
  returnTrajectoryClient = helperNode->create_client<atos_interfaces::srv::GetObjectReturnTrajectory>(serviceName);
  
  auto returnTrajectoryRequest = std::make_shared<atos_interfaces::srv::GetObjectReturnTrajectory::Request>();
  testUtils::waitForService(returnTrajectoryClient, std::chrono::milliseconds(1000));

  // Act
  returnTrajectoryClient->async_send_request(returnTrajectoryRequest, returnTrajResponseCallback);
  // wait for service to respond and callback to execute
  testUtils::waitForFuture(executor, serviceCalledFuture, failAfterTimeout);

  // Assert
  ASSERT_EQ(receivedResponse, false);
}

TEST_F(BackToStartTest, testServiceCallWithTrajectory){
  // Setup
  std::promise<void> serviceCalled;
  std::shared_future serviceCalledFuture(serviceCalled.get_future());
  auto failAfterTimeout = std::chrono::milliseconds(5000);

  bool receivedResponse = false; 
  geometry_msgs::msg::Pose returnTrajEndPoint;
  auto returnTrajResponseCallback = [&receivedResponse, &returnTrajEndPoint, &serviceCalled](const rclcpp::Client<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedFuture future) {
    receivedResponse = future.get()->success;
    returnTrajEndPoint = future.get()->trajectory.points.back().pose;
    serviceCalled.set_value();
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
  auto trajStartPoint = returnTrajectoryRequest->trajectory.points[0].pose;
  testUtils::waitForService(returnTrajectoryClient, std::chrono::milliseconds(1000));

  // Act
  returnTrajectoryClient->async_send_request(returnTrajectoryRequest, returnTrajResponseCallback);
  // wait for service to respond and callback to execute
  testUtils::waitForFuture(executor, serviceCalledFuture, failAfterTimeout);

  // Assert
  ASSERT_EQ(receivedResponse, true);
  ASSERT_EQ(returnTrajEndPoint.position.x, trajStartPoint.position.x);
  ASSERT_EQ(returnTrajEndPoint.position.y, trajStartPoint.position.y);
  ASSERT_EQ(returnTrajEndPoint.position.z, trajStartPoint.position.z);
  ASSERT_EQ(returnTrajEndPoint.orientation.x, trajStartPoint.orientation.x);
  ASSERT_EQ(returnTrajEndPoint.orientation.y, trajStartPoint.orientation.y);
  ASSERT_EQ(returnTrajEndPoint.orientation.z, trajStartPoint.orientation.z);
  ASSERT_EQ(returnTrajEndPoint.orientation.w, trajStartPoint.orientation.w);
}