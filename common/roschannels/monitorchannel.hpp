/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/monitor.hpp"

namespace ROSChannels {
    namespace Monitor {
        const std::string topicName = "object_monitor";
        using message_type = atos_interfaces::msg::Monitor;
        const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepLast(1));

        class Pub : public BasePub<message_type> {
        public:
            const uint32_t objectId;
            Pub(rclcpp::Node& node, const uint32_t id, const rclcpp::QoS& qos = defaultQoS) : 
                objectId(id),
                BasePub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, qos) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            const uint32_t objectId;
            Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : 
                objectId(id),
                BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback, qos) {}
        };
    }
}