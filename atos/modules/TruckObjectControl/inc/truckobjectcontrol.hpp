/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_map>

class TruckObjectControl : public rclcpp::Node {
public:
  TruckObjectControl();

private:
  struct TruckState {
    double distance_along_trajectory_m = 0.0;
    bool tcp_connected = false;
    rclcpp::Time last_cot_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  };

  struct CotObservation {
    std::string truck_id;
    double distance_along_trajectory_m = 0.0;
    bool tcp_connected = false;
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cot_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_command_pub_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;

  std::unordered_map<std::string, TruckState> trucks_;

  double warning_distance_m_ = 400.0;
  double stop_distance_m_ = 200.0;
  double warning_speed_kmh_ = 30.0;
  double stop_speed_kmh_ = 0.0;
  double cot_timeout_seconds_ = 2.0;

  double last_published_speed_kmh_ = -1.0;

  void onCotMessage(const std_msgs::msg::String::SharedPtr msg);
  void evaluateAndPublishSpeedCommand();

  bool parseCotPlaceholder(const std::string &payload, CotObservation &out) const;
  bool isCotFresh(const TruckState &state, const rclcpp::Time &now) const;
};
