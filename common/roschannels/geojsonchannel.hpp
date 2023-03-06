#pragma once

#include "roschannel.hpp"
#include "foxglove_msgs/msg/geo_json.hpp"

namespace ROSChannels {
    namespace GNSSPath {
        const std::string topicName = "gnss_path";
        using message_type = foxglove_msgs::msg::GeoJSON;

        class Pub : public BasePub<message_type> {
        public:
            const uint32_t objectId;
            Pub(rclcpp::Node& node, const uint32_t id) :
                objectId(id),
                BasePub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            const uint32_t objectId;
            Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback) : 
                objectId(id),
                BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
        };
    }
}