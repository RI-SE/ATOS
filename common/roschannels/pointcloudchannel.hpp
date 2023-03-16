/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ROSChannels {
  namespace Pointcloud {
    const std::string topicName = "pointcloud";
    using message_type = sensor_msgs::msg::PointCloud2;
    
    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) :
          BasePub<message_type>(node, "pointcloud/" + topicName, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
      Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : 
          BaseSub<message_type>(node, "pointcloud/" + topicName, callback, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
    };
  }
}