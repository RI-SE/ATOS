/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "atos_interfaces/msg/story_board_element_state_change.hpp"
#include "roschannel.hpp"

namespace ROSChannels {
namespace StoryBoardElementStateChange {
const std::string topicName = "story_board_element_state_change";
using message_type = atos_interfaces::msg::StoryBoardElementStateChange;

class Pub : public BasePub<message_type> {
public:
  Pub(rclcpp::Node &node)
      : BasePub<message_type>(
            node, topicName,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
};

class Sub : public BaseSub<message_type> {
public:
  Sub(rclcpp::Node &node,
      std::function<void(const message_type::SharedPtr)> callback)
      : BaseSub<message_type>(
            node, topicName, callback,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
};
} // namespace StoryBoardElementStateChange
} // namespace ROSChannels