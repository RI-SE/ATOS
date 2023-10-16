/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <cinttypes>
#include <memory>
#include <stdexcept>
#include <string>
#include "module.hpp"

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

namespace testUtils
{

// Sleep for timeout ms or until a subscriber has registered for the topic
// if toBeAvailable is true, then it will wait for the number of
// subscribers to be > 0, if false it will wait for the number of
// subscribers to be 0
void waitForSubscriber(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topicName,
  bool toBeAvailable = true,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1),
  std::chrono::microseconds sleepPeriod = std::chrono::seconds(1))
{
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  using std::chrono::steady_clock;
  auto start = steady_clock::now();
  microseconds timeSlept(0);
  auto predicate = [&node, &topicName, &toBeAvailable]() -> bool {
      if (toBeAvailable) {
        // the subscriber is available if the count is gt 0
        return node->count_subscribers(topicName) > 0;
      } else {
        // the subscriber is no longer available when the count is 0
        return node->count_subscribers(topicName) == 0;
      }
    };
  while (!predicate() && timeSlept < duration_cast<microseconds>(timeout)) {
    rclcpp::Event::SharedPtr graphEvent = node->get_graph_event();
    node->wait_for_graph_change(graphEvent, sleepPeriod);
    timeSlept = duration_cast<std::chrono::microseconds>(steady_clock::now() - start);
  }
  int64_t timeSleptCount =
    std::chrono::duration_cast<std::chrono::microseconds>(timeSlept).count();
  printf(
    "Waited %" PRId64 " microseconds for the subscriber to %s topic '%s'\n",
    timeSleptCount, toBeAvailable ? "connect to" : "disconnect from",
    topicName.c_str());
}

  // Sleep for timeout ms or until the service is available to the client
  template<typename DurationT, typename ServiceT>
  void waitForService(
  std::shared_ptr<rclcpp::Client<ServiceT>> client,
  const DurationT & timeout)
{
  while (!client->wait_for_service(timeout)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
  }
}

template<typename DurationT>
void waitForFuture(
  rclcpp::Executor & executor,
  std::shared_future<void> & future,
  const DurationT & timeout)
{
  using rclcpp::FutureReturnCode;
  rclcpp::FutureReturnCode futureRet;
  auto startTime = std::chrono::steady_clock::now();
  futureRet = executor.spin_until_future_complete(future, timeout);
  auto elapsedTime = std::chrono::steady_clock::now() - startTime;
  EXPECT_EQ(FutureReturnCode::SUCCESS, futureRet) <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime).count() <<
    " milliseconds\n";
}


template<typename ModuleUnderTest>
  rclcpp::executors::SingleThreadedExecutor getTestNodeExecutor(
  std::string const & testNodeName
  )
{
  auto moduleUnderTest =  std::make_shared<ModuleUnderTest>();
  auto testNode = rclcpp::Node::make_shared(testNodeName);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(moduleUnderTest);
  executor.add_node(testNode);
  return executor;
}

}  // namespace testUtils