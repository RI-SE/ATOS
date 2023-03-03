#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/trigger_configuration.hpp"
#include "atos_interfaces/msg/execute_action.hpp"
#include "atos_interfaces/msg/action_configuration.hpp"
#include "atos_interfaces/msg/trigger_event_occurred.hpp"

namespace ROSChannels {
    namespace TriggerConfiguration {
        const std::string topicName = "trigger_configuration";
        using message_type = atos_interfaces::msg::TriggerConfiguration;
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

    namespace ExecuteAction {
        const std::string topicName = "execute_action";
        using message_type = atos_interfaces::msg::ExecuteAction;
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

    namespace ActionConfiguration {
        const std::string topicName = "action_configuration";
        using message_type = atos_interfaces::msg::ActionConfiguration;
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

    namespace TriggerEventOccurred {
        const std::string topicName = "trigger_event_occurred";
        using message_type = atos_interfaces::msg::TriggerEventOccurred;

        class Pub : public BasePub<message_type> {
        public:
            Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
        };
    }
}