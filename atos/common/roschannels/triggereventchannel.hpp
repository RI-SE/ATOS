/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "atos_interfaces/msg/trigger_event_occurred.hpp"
#include "roschannel.hpp"

namespace ROSChannels {
namespace TriggerEventOccurred {
const std::string topicName = "trigger_event_occurred";
using message_type = atos_interfaces::msg::TriggerEventOccurred;

class Pub : public BasePub<message_type> {
public:
  explicit Pub(rclcpp::Node &node) : BasePub<message_type>(node, topicName) {}
};

class Sub : public BaseSub<message_type> {
public:
  Sub(rclcpp::Node &node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
};
} // namespace TriggerEventOccurred
} // namespace ROSChannels