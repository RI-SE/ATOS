/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>

namespace ROSChannels {

template<typename T>
class BasePub {
public:
    BasePub(rclcpp::Node& node,
        const std::string& topicName,
        const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepAll()))
        : pub(node.create_publisher<T>(topicName, qos)) {}
    BasePub() = delete;
    typename rclcpp::Publisher<T>::SharedPtr pub;
    inline virtual void publish(const T& msg) { assert(pub); pub->publish(msg); };
};

template<typename T>
class BaseSub {
public:
    BaseSub(rclcpp::Node& node,
        const std::string& topicName,
        std::function<void(const typename T::SharedPtr)> callback,
        const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepAll()))
        : sub(node.create_subscription<T>(topicName, qos, callback)) {}
    BaseSub() = delete;
    typename rclcpp::Subscription<T>::SharedPtr sub;
};

} // namespace ROSChannels