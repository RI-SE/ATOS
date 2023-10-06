// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_RCLCPP__UTILS_HPP_
#define TEST_RCLCPP__UTILS_HPP_

#include <cinttypes>
#include <memory>
#include <stdexcept>
#include <string>
#include "module.hpp"

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

namespace test_rclcpp
{

// Sleep for timeout ms or until a subscriber has registered for the topic
// if to_be_available is true, then it will wait for the number of
// subscribers to be > 0, if false it will wait for the number of
// subscribers to be 0
void wait_for_subscriber(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  bool to_be_available = true,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1),
  std::chrono::microseconds sleep_period = std::chrono::seconds(1))
{
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  using std::chrono::steady_clock;
  auto start = steady_clock::now();
  microseconds time_slept(0);
  auto predicate = [&node, &topic_name, &to_be_available]() -> bool {
      if (to_be_available) {
        // the subscriber is available if the count is gt 0
        return node->count_subscribers(topic_name) > 0;
      } else {
        // the subscriber is no longer available when the count is 0
        return node->count_subscribers(topic_name) == 0;
      }
    };
  while (!predicate() && time_slept < duration_cast<microseconds>(timeout)) {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = duration_cast<std::chrono::microseconds>(steady_clock::now() - start);
  }
  int64_t time_slept_count =
    std::chrono::duration_cast<std::chrono::microseconds>(time_slept).count();
  printf(
    "Waited %" PRId64 " microseconds for the subscriber to %s topic '%s'\n",
    time_slept_count, to_be_available ? "connect to" : "disconnect from",
    topic_name.c_str());
}

template<typename DurationT>
void wait_for_future(
  rclcpp::Executor & executor,
  std::shared_future<void> & future,
  const DurationT & timeout)
{
  using rclcpp::FutureReturnCode;
  rclcpp::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret) <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
    " milliseconds\n";
}


template<typename ModuleUnderTest>
  rclcpp::executors::SingleThreadedExecutor get_test_node_executor(
  std::string const & testNodeName
  )
{
  auto module_under_test =  std::make_shared<ModuleUnderTest>();
  auto test_node = rclcpp::Node::make_shared(testNodeName);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(module_under_test);
  executor.add_node(test_node);
  return executor;
}

}  // namespace test_rclcpp

#endif  // TEST_RCLCPP__UTILS_HPP_