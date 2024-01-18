/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/state_change.hpp"

namespace ROSChannels {
    namespace StateChange {
        const std::string topicName = "state_change";
        using message_type = atos_interfaces::msg::StateChange;
        const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

        class Pub : public BasePub<message_type> {
        public:
            Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
        };
    }
}