#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/v2x.hpp"

namespace ROSChannels {
    namespace V2X {
        const std::string topicName = "v2x_message";
        using message_type = atos_interfaces::msg::V2x;

        class Pub : public BasePub<message_type> {
        public:
            Pub(rclcpp::Node& node) :
                BasePub<message_type>(node, topicName) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
        };
    }
}