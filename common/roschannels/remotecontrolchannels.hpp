#pragma once

#include "roschannel.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"
#include "atos_interfaces/msg/manoeuvre_command.hpp"


namespace ROSChannels {
    namespace RemoteControlEnable {
        const std::string topicName = "remote_control_enable";
        using message_type = std_msgs::msg::Empty;
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

    namespace RemoteControlDisable {
        const std::string topicName = "remote_control_disable";
        using message_type = std_msgs::msg::Empty;
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

    namespace RemoteControlManoeuvre {
        const std::string topicName = "remote_control_manoeuvre";
        using message_type = atos_interfaces::msg::ManoeuvreCommand;
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

    namespace BackToStart {
        const std::string topicName = "back_to_start";
        using message_type = atos_interfaces::msg::ManoeuvreCommand;
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

    namespace BackToStartResponse {
        const std::string topicName = "back_to_start_response";
        using message_type = std_msgs::msg::Int8;
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