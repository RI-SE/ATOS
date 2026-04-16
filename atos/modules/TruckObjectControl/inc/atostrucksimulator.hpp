/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>

class AtosTruckSimulator : public rclcpp::Node {
public:
  AtosTruckSimulator();

private:
  struct GeoPoint {
    double lat = 0.0;
    double lon = 0.0;
    double distance_m = 0.0;
  };

  std::string uid_ = "L5S-TRUCK-SIM";
  std::string tcp_host_ = "127.0.0.1";
  int tcp_port_ = 8114;
  std::string trajectory_geojson_path_ = "";
  std::string trajectory_path_name_ = "";
  int start_index_ = 0;
  double initial_speed_kmh_ = 0.0;
  double target_speed_kmh_ = 40.0;
  double acceleration_mps2_ = 2.0;
  double publish_hz_ = 5.0;
  bool loop_path_ = true;
  bool ignore_warning_speed_commands_ = false;

  double current_distance_m_ = 0.0;
  double current_speed_mps_ = 0.0;

  int tcp_fd_ = -1;
  std::string tcp_rx_buffer_;
  std::vector<GeoPoint> trajectory_path_;

  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speed_command_sub_;

  rclcpp::Time last_step_time_;

  bool loadTrajectoryPath();
  bool ensureTcpConnected();
  void closeTcp();
  void pollTcpCommands();
  void simulationStep();
  bool pointAtDistance(double distance_m, double &lat, double &lon, double &course_deg,
                       int &path_index) const;
  static std::string utcIso8601FromRosTime(const rclcpp::Time &time);
  void applySpeedCommandPayload(const std::string &payload);
  void onSpeedCommand(const std_msgs::msg::String::SharedPtr msg);
};
