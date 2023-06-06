/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include "atos_interfaces/msg/monitor.hpp"
#include "positioning.h"
#include <tf2/LinearMath/Quaternion.h>
#if ROS_FOXY
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#elif ROS_HUMBLE
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif


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
        static message_type fromMonr(const uint32_t id, const ObjectMonitorType& monrMessage) {
            atos_interfaces::msg::Monitor msg;
            auto txid = id;
            auto indata = monrMessage;
            auto stamp = rclcpp::Time(indata.timestamp.tv_sec, indata.timestamp.tv_usec*1000);
            
            // Set stamp for all subtypes
            msg.atos_header.header.stamp = stamp;
            msg.pose.header.stamp = stamp;
            msg.velocity.header.stamp = stamp;
            msg.acceleration.header.stamp = stamp;

            // Set frame ids
            msg.atos_header.header.frame_id = "map"; // TODO
            msg.pose.header.frame_id = "map"; // TODO
            msg.velocity.header.frame_id = "map"; // TODO vehicle local
            msg.acceleration.header.frame_id = "map"; // TODO vehicle local

            msg.atos_header.object_id = txid;
            msg.object_state.state = indata.state;
            if (indata.position.isPositionValid) {
                msg.pose.pose.position.x = indata.position.xCoord_m;
                msg.pose.pose.position.y = indata.position.yCoord_m;
                msg.pose.pose.position.z = indata.position.isZcoordValid ? indata.position.zCoord_m : 0.0;
            }
            if (indata.position.isHeadingValid) {
                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, indata.position.heading_rad);
                msg.pose.pose.orientation = tf2::toMsg(orientation);
            }
            msg.velocity.twist.linear.x = indata.speed.isLongitudinalValid ? indata.speed.longitudinal_m_s : 0;
            msg.velocity.twist.linear.y = indata.speed.isLateralValid ? indata.speed.lateral_m_s : 0;
            msg.velocity.twist.linear.z = 0;
            msg.velocity.twist.angular.x = 0;
            msg.velocity.twist.angular.y = 0;
            msg.velocity.twist.angular.z = 0;
            msg.acceleration.accel.linear.x = indata.acceleration.isLongitudinalValid ? indata.acceleration.longitudinal_m_s2 : 0;
            msg.acceleration.accel.linear.y = indata.acceleration.isLateralValid ? indata.acceleration.lateral_m_s2 : 0;
            msg.acceleration.accel.linear.z = 0;
            msg.acceleration.accel.angular.x = 0;
            msg.acceleration.accel.angular.y = 0;
            msg.acceleration.accel.angular.z = 0;
            return msg;
        }
    }
}