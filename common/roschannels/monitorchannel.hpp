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

        class AnchorPub : public BasePub<message_type> {
        public:
            AnchorPub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) :
                BasePub<message_type>(node, "object_anchor/" + topicName, qos) {}
        };

        class Sub : public BaseSub<message_type> {
        public:
            const uint32_t objectId;
            Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : 
                objectId(id),
                BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback, qos) {}
        };

        class AnchorSub : public BaseSub<message_type> {
        public:
            AnchorSub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : 
                BaseSub<message_type>(node, "object_anchor/" + topicName, callback, qos) {}
        };

        static message_type fromISOMonr(const uint32_t id, const ObjectMonitorType& indata) {
            atos_interfaces::msg::Monitor outdata;
            auto txid = id;
            auto stamp = rclcpp::Time(indata.timestamp.tv_sec, indata.timestamp.tv_usec*1000);
            
            // Set stamp for all subtypes
            outdata.atos_header.header.stamp = stamp;
            outdata.pose.header.stamp = stamp;
            outdata.velocity.header.stamp = stamp;
            outdata.acceleration.header.stamp = stamp;

            // Set frame ids
            outdata.atos_header.header.frame_id = "map"; // TODO
            outdata.pose.header.frame_id = "map"; // TODO
            outdata.velocity.header.frame_id = "map"; // TODO vehicle local
            outdata.acceleration.header.frame_id = "map"; // TODO vehicle local

            outdata.atos_header.object_id = txid;
            outdata.object_state.state = indata.state;
            if (indata.position.isPositionValid) {
                outdata.pose.pose.position.x = indata.position.xCoord_m;
                outdata.pose.pose.position.y = indata.position.yCoord_m;
                outdata.pose.pose.position.z = indata.position.isZcoordValid ? indata.position.zCoord_m : 0.0;
            }
            if (indata.position.isHeadingValid) {
                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, indata.position.heading_rad);
                outdata.pose.pose.orientation = tf2::toMsg(orientation);
            }
            outdata.velocity.twist.linear.x = indata.speed.isLongitudinalValid ? indata.speed.longitudinal_m_s : 0;
            outdata.velocity.twist.linear.y = indata.speed.isLateralValid ? indata.speed.lateral_m_s : 0;
            outdata.velocity.twist.linear.z = 0;
            outdata.velocity.twist.angular.x = 0;
            outdata.velocity.twist.angular.y = 0;
            outdata.velocity.twist.angular.z = 0;
            outdata.acceleration.accel.linear.x = indata.acceleration.isLongitudinalValid ? indata.acceleration.longitudinal_m_s2 : 0;
            outdata.acceleration.accel.linear.y = indata.acceleration.isLateralValid ? indata.acceleration.lateral_m_s2 : 0;
            outdata.acceleration.accel.linear.z = 0;
            outdata.acceleration.accel.angular.x = 0;
            outdata.acceleration.accel.angular.y = 0;
            outdata.acceleration.accel.angular.z = 0;
            return outdata;
        }

        static ObjectMonitorType toISOMonr(message_type& indata){
            ObjectMonitorType outdata;
            outdata.timestamp.tv_sec = indata.atos_header.header.stamp.sec;
            outdata.timestamp.tv_usec = indata.atos_header.header.stamp.nanosec / 1000;
            //TODO: Add state..
            outdata.position.isPositionValid = true;
            outdata.position.isXcoordValid = true;
            outdata.position.isYcoordValid = true;
            outdata.position.isZcoordValid = true;
            outdata.position.xCoord_m = indata.pose.pose.position.x;
            outdata.position.yCoord_m = indata.pose.pose.position.y;
            outdata.position.zCoord_m = indata.pose.pose.position.z;
            outdata.position.isHeadingValid = true;
            tf2::Quaternion quat_tf;
            geometry_msgs::msg::Quaternion quat_msg = indata.pose.pose.orientation;
            tf2::fromMsg(quat_msg, quat_tf);
            double r{}, p{}, y{};
            tf2::Matrix3x3 m(quat_tf);
            m.getRPY(r, p, y);
            outdata.position.heading_rad = y;
            outdata.speed.isLongitudinalValid = true;
            outdata.speed.longitudinal_m_s = indata.velocity.twist.linear.x;
            outdata.speed.isLateralValid = true;
            outdata.speed.lateral_m_s = indata.velocity.twist.linear.y;
            outdata.acceleration.isLongitudinalValid = true;
            outdata.acceleration.longitudinal_m_s2 = indata.acceleration.accel.linear.x;
            outdata.acceleration.isLateralValid = true;
            outdata.acceleration.lateral_m_s2 = indata.acceleration.accel.linear.y;
            return outdata;
        }
    }
}