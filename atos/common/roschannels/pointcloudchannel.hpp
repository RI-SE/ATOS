/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "roschannel.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ROSChannels {
  namespace Pointcloud {
    const std::string topicName = "site_scan";
    using message_type = sensor_msgs::msg::PointCloud2;
    
    class Pub : public BasePub<message_type> {
    public:
        const std::string fileName;
        Pub(rclcpp::Node& node, const std::string fileName) :
          BasePub<message_type>(node, topicName + "/" + fileName, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
      const std::string fileName;
      Sub(rclcpp::Node& node, const std::string fileName, std::function<void(const message_type::SharedPtr)> callback) : 
          BaseSub<message_type>(node, topicName + "/" + fileName, callback, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()) {}
    };
  }
}