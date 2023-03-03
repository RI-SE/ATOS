
#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/cartesian_trajectory.hpp"

namespace ROSChannels {
    namespace CartesianTrajectory {
        const std::string topicName = "cartesian_trajectory";
        using message_type = atos_interfaces::msg::CartesianTrajectory;

        class Pub : public BasePub<message_type> {
        public:
            const uint32_t objectId;
            Pub(rclcpp::Node& node, const uint32_t id) :
                objectId(id),
                BasePub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            const uint32_t objectId;
            Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback) : 
                objectId(id),
                BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback) {}
        };
    }
}