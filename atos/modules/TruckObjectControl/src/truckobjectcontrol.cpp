/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "truckobjectcontrol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

using std::placeholders::_1;

TruckObjectControl::TruckObjectControl() : Node("truck_object_control") {
  declare_parameter("warning_distance_m", warning_distance_m_);
  declare_parameter("stop_distance_m", stop_distance_m_);
  declare_parameter("warning_speed_kmh", warning_speed_kmh_);
  declare_parameter("stop_speed_kmh", stop_speed_kmh_);
  declare_parameter("cot_timeout_seconds", cot_timeout_seconds_);

  warning_distance_m_ = get_parameter("warning_distance_m").as_double();
  stop_distance_m_ = get_parameter("stop_distance_m").as_double();
  warning_speed_kmh_ = get_parameter("warning_speed_kmh").as_double();
  stop_speed_kmh_ = get_parameter("stop_speed_kmh").as_double();
  cot_timeout_seconds_ = get_parameter("cot_timeout_seconds").as_double();

  cot_sub_ = create_subscription<std_msgs::msg::String>(
      "truck_objects/cot", 50, std::bind(&TruckObjectControl::onCotMessage, this, _1));

  speed_command_pub_ = create_publisher<std_msgs::msg::String>("truck_objects/speed_command", 20);

  evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&TruckObjectControl::evaluateAndPublishSpeedCommand, this));

  RCLCPP_INFO(get_logger(),
              "TruckObjectControl placeholder started. Waiting for COT on 'truck_objects/cot'.");
  RCLCPP_INFO(get_logger(),
              "Expected placeholder payload: id=<truck_id>;distance_m=<value>;tcp_connected=<0|1>");
}

void TruckObjectControl::onCotMessage(const std_msgs::msg::String::SharedPtr msg) {
  CotObservation observation;
  if (!parseCotPlaceholder(msg->data, observation)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Failed to parse COT payload. Keeping placeholder behavior.");
    return;
  }

  auto &state = trucks_[observation.truck_id];
  state.distance_along_trajectory_m = observation.distance_along_trajectory_m;
  state.tcp_connected = observation.tcp_connected;
  state.last_cot_stamp = now();
}

bool TruckObjectControl::parseCotPlaceholder(const std::string &payload, CotObservation &out) const {
  // Placeholder parser for early development only.
  // Format: id=<truck_id>;distance_m=<value>;tcp_connected=<0|1>
  std::unordered_map<std::string, std::string> fields;
  std::stringstream ss(payload);
  std::string token;

  while (std::getline(ss, token, ';')) {
    const auto sep = token.find('=');
    if (sep == std::string::npos) {
      continue;
    }
    const std::string key = token.substr(0, sep);
    const std::string value = token.substr(sep + 1);
    fields[key] = value;
  }

  if (fields.find("id") == fields.end() || fields.find("distance_m") == fields.end() ||
      fields.find("tcp_connected") == fields.end()) {
    return false;
  }

  out.truck_id = fields["id"];
  if (out.truck_id.empty()) {
    return false;
  }

  try {
    out.distance_along_trajectory_m = std::stod(fields["distance_m"]);
    const auto &tcp = fields["tcp_connected"];
    out.tcp_connected = (tcp == "1" || tcp == "true" || tcp == "TRUE");
  } catch (...) {
    return false;
  }

  return true;
}

bool TruckObjectControl::isCotFresh(const TruckState &state, const rclcpp::Time &now_time) const {
  const auto age = (now_time - state.last_cot_stamp).seconds();
  return age >= 0.0 && age <= cot_timeout_seconds_;
}

void TruckObjectControl::evaluateAndPublishSpeedCommand() {
  std::vector<double> connected_positions;
  std::vector<std::string> connected_ids;

  const auto now_time = now();
  for (const auto &[id, state] : trucks_) {
    if (!state.tcp_connected || !isCotFresh(state, now_time)) {
      continue;
    }
    connected_positions.push_back(state.distance_along_trajectory_m);
    connected_ids.push_back(id);
  }

  if (connected_positions.size() < 2) {
    return;
  }

  std::sort(connected_positions.begin(), connected_positions.end());

  double min_gap_m = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < connected_positions.size(); ++i) {
    min_gap_m = std::min(min_gap_m, std::fabs(connected_positions[i] - connected_positions[i - 1]));
  }

  double target_speed_kmh = -1.0;
  std::string reason = "no_limit";

  if (min_gap_m < stop_distance_m_) {
    target_speed_kmh = stop_speed_kmh_;
    reason = "min_gap_below_stop_distance";
  } else if (min_gap_m < warning_distance_m_) {
    target_speed_kmh = warning_speed_kmh_;
    reason = "min_gap_below_warning_distance";
  }

  if (target_speed_kmh < 0.0) {
    return;
  }

  if (std::fabs(last_published_speed_kmh_ - target_speed_kmh) < 1e-6) {
    return;
  }

  std_msgs::msg::String command;
  command.data = "target_speed_kmh=" + std::to_string(target_speed_kmh) +
                 ";scope=all_connected_with_valid_tcp_and_fresh_cot" +
                 ";reason=" + reason +
                 ";min_gap_m=" + std::to_string(min_gap_m) +
                 ";connected_count=" + std::to_string(connected_ids.size());

  speed_command_pub_->publish(command);
  last_published_speed_kmh_ = target_speed_kmh;

  RCLCPP_WARN(get_logger(),
              "Published speed command %.1f km/h (reason=%s, min_gap=%.2f m, connected=%zu)",
              target_speed_kmh, reason.c_str(), min_gap_m, connected_ids.size());
}
