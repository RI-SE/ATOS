#pragma once

#include "roschannel.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "atos_interfaces/msg/object_id_array.hpp"
#include "atos_interfaces/msg/object_trigger_start.hpp"

namespace ROSChannels {
    namespace Init {
        const std::string topicName = "init";
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

    namespace Connect {
        const std::string topicName = "connect";
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

    namespace Arm {
        const std::string topicName = "arm";
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

    namespace Disarm {
        const std::string topicName = "disarm";
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

    namespace Start {
        const std::string topicName = "start";
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

    namespace StartObject {
        const std::string topicName = "start_object";
        using message_type = atos_interfaces::msg::ObjectTriggerStart;

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

    namespace Stop {
        const std::string topicName = "stop";
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

    namespace Disconnect {
        const std::string topicName = "disconnect";
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

    namespace Failure {
        const std::string topicName = "failure";
        using message_type = std_msgs::msg::UInt8;
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

    namespace GetStatus {
        const std::string topicName = "get_status";
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

    namespace GetStatusResponse {
        const std::string topicName = "get_status_response";
        using message_type = std_msgs::msg::String;
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

    namespace AllClear {
        const std::string topicName = "all_clear";
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

    namespace Abort {
        const std::string topicName = "abort";
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

    namespace Exit {
        const std::string topicName = "exit";
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

    namespace Replay {
        const std::string topicName = "replay";
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

    namespace ObjectsConnected {
        const std::string topicName = "objects_connected";
        using message_type = std_msgs::msg::Empty;
        const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

        class Pub : public BasePub<message_type> {
        public:
            Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
        };
    }

    namespace ConnectedObjectIds {
        const std::string topicName = "connected_object_ids";
        using message_type = atos_interfaces::msg::ObjectIdArray;
        const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

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