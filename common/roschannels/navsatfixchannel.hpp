/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "monitorchannel.hpp" // TODO: remove this when making translator node that translates monr to various other msg types..
#include "util/coordinateutils.hpp" // TODO: also remove this

namespace ROSChannels {
    namespace NavSatFix {
        const std::string topicName = "gnss_fix";
        using message_type = sensor_msgs::msg::NavSatFix;
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
        // TODO: Remove below..
        static message_type fromROSMonr(std::array<double,3> origin, const ROSChannels::Monitor::message_type &monr) {
            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = monr.atos_header.header.stamp;

            // Local coordinates to global coordinates
            double offset[3] = {monr.pose.pose.position.x, monr.pose.pose.position.y, monr.pose.pose.position.z};
            double llh_0[3] = {origin[0], origin[1], origin[2]};
            llhOffsetMeters(llh_0,offset);
            msg.header.frame_id = "map"; // TODO

            // Fill in the rest of the message
            msg.latitude = llh_0[0];
            msg.longitude = llh_0[1];
            msg.altitude = llh_0[2];
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            return msg;
        }
    }
}