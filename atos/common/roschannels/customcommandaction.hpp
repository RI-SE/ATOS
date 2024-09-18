/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "atos_interfaces/msg/custom_command_action.hpp"
#include "roschannel.hpp"

namespace ROSChannels {
namespace CustomCommandAction {
const std::string topicName = "custom_command_action";
using message_type = atos_interfaces::msg::CustomCommandAction;
const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepLast(1));

class Pub : public BasePub<message_type> {
public:
  Pub(rclcpp::Node &node, const rclcpp::QoS &qos = defaultQoS)
      : BasePub<message_type>(node, topicName, qos) {}
};

class Sub : public BaseSub<message_type> {
public:
  Sub(rclcpp::Node &node,
      std::function<void(const message_type::SharedPtr)> callback,
      const rclcpp::QoS &qos = defaultQoS)
      : BaseSub<message_type>(node, topicName, callback, qos) {}
};
} // namespace CustomCommandAction
} // namespace ROSChannels