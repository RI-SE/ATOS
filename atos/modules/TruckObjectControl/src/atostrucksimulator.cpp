/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "atostrucksimulator.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

using json = nlohmann::json;

namespace {
constexpr double kEpsilon = 1e-9;

double degToRad(double value) {
  return value * M_PI / 180.0;
}

double radToDeg(double value) {
  return value * 180.0 / M_PI;
}

double geodesicDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double earth_radius_m = 6378137.0;
  const double dlat = degToRad(lat2 - lat1);
  const double dlon = degToRad(lon2 - lon1);
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(degToRad(lat1)) * std::cos(degToRad(lat2)) *
                       std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return earth_radius_m * c;
}
}

AtosTruckSimulator::AtosTruckSimulator() : Node("atos_truck_simulator") {
  declare_parameter("uid", uid_);
  declare_parameter("tcp_host", tcp_host_);
  declare_parameter("tcp_port", tcp_port_);
  declare_parameter("trajectory_geojson_path", trajectory_geojson_path_);
  declare_parameter("start_index", start_index_);
  declare_parameter("initial_speed_kmh", initial_speed_kmh_);
  declare_parameter("target_speed_kmh", target_speed_kmh_);
  declare_parameter("acceleration_mps2", acceleration_mps2_);
  declare_parameter("publish_hz", publish_hz_);
  declare_parameter("loop_path", loop_path_);

  uid_ = get_parameter("uid").as_string();
  tcp_host_ = get_parameter("tcp_host").as_string();
  tcp_port_ = get_parameter("tcp_port").as_int();
  trajectory_geojson_path_ = get_parameter("trajectory_geojson_path").as_string();
  start_index_ = get_parameter("start_index").as_int();
  initial_speed_kmh_ = get_parameter("initial_speed_kmh").as_double();
  target_speed_kmh_ = get_parameter("target_speed_kmh").as_double();
  acceleration_mps2_ = std::max(0.01, get_parameter("acceleration_mps2").as_double());
  publish_hz_ = std::max(1.0, get_parameter("publish_hz").as_double());
  loop_path_ = get_parameter("loop_path").as_bool();

  if (!loadTrajectoryPath()) {
    RCLCPP_ERROR(get_logger(), "Failed to load trajectory at '%s'", trajectory_geojson_path_.c_str());
    return;
  }

  if (start_index_ < 0) {
    start_index_ = 0;
  }
  if (static_cast<size_t>(start_index_) >= trajectory_path_.size()) {
    start_index_ = static_cast<int>(trajectory_path_.size() - 1);
  }

  current_distance_m_ = trajectory_path_[static_cast<size_t>(start_index_)].distance_m;
  current_speed_mps_ = std::max(0.0, initial_speed_kmh_ / 3.6);
  last_step_time_ = now();

  speed_command_sub_ = create_subscription<std_msgs::msg::String>(
      "truck_objects/speed_command", 20,
      std::bind(&AtosTruckSimulator::onSpeedCommand, this, std::placeholders::_1));

  const auto period_ms = static_cast<int>(1000.0 / publish_hz_);
  simulation_timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                                        std::bind(&AtosTruckSimulator::simulationStep, this));

  RCLCPP_INFO(get_logger(),
              "AtosTruckSimulator started (uid=%s, start_index=%d, target=%.1f km/h, accel=%.2f m/s^2, tcp=%s:%d)",
              uid_.c_str(), start_index_, target_speed_kmh_, acceleration_mps2_, tcp_host_.c_str(), tcp_port_);
}

bool AtosTruckSimulator::loadTrajectoryPath() {
  trajectory_path_.clear();

  std::ifstream input(trajectory_geojson_path_);
  if (!input.is_open()) {
    return false;
  }

  json root;
  try {
    input >> root;
  } catch (...) {
    return false;
  }

  if (!root.contains("features") || !root["features"].is_array()) {
    return false;
  }

  json line_feature;
  for (const auto &feature : root["features"]) {
    if (!feature.contains("geometry")) {
      continue;
    }
    const auto &geometry = feature["geometry"];
    if (!geometry.contains("type") || geometry["type"] != "LineString") {
      continue;
    }
    if (feature.contains("id") && feature["id"] == "PathSection") {
      line_feature = feature;
      break;
    }
    if (line_feature.is_null()) {
      line_feature = feature;
    }
  }

  if (line_feature.is_null()) {
    return false;
  }

  const auto &coords = line_feature["geometry"]["coordinates"];
  if (!coords.is_array() || coords.empty()) {
    return false;
  }

  double cumulative = 0.0;
  GeoPoint prev;
  bool has_prev = false;

  for (const auto &coord : coords) {
    if (!coord.is_array() || coord.size() < 2) {
      continue;
    }
    GeoPoint p;
    p.lon = coord[0].get<double>();
    p.lat = coord[1].get<double>();
    p.distance_m = cumulative;

    if (has_prev) {
      cumulative += geodesicDistanceMeters(prev.lat, prev.lon, p.lat, p.lon);
      p.distance_m = cumulative;
    }

    trajectory_path_.push_back(p);
    prev = p;
    has_prev = true;
  }

  return trajectory_path_.size() >= 2;
}

bool AtosTruckSimulator::ensureTcpConnected() {
  if (tcp_fd_ >= 0) {
    return true;
  }

  tcp_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(tcp_port_));
  if (::inet_pton(AF_INET, tcp_host_.c_str(), &addr.sin_addr) != 1) {
    closeTcp();
    return false;
  }

  if (::connect(tcp_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    closeTcp();
    return false;
  }

  return true;
}

void AtosTruckSimulator::closeTcp() {
  if (tcp_fd_ >= 0) {
    ::shutdown(tcp_fd_, SHUT_RDWR);
    ::close(tcp_fd_);
    tcp_fd_ = -1;
  }
}

std::string AtosTruckSimulator::utcIso8601FromRosTime(const rclcpp::Time &time) {
  const auto seconds = static_cast<time_t>(time.seconds());
  std::tm tm_utc{};
  gmtime_r(&seconds, &tm_utc);
  std::ostringstream out;
  out << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

bool AtosTruckSimulator::pointAtDistance(double distance_m, double &lat, double &lon, double &course_deg) const {
  if (trajectory_path_.size() < 2) {
    return false;
  }

  const double max_distance = trajectory_path_.back().distance_m;
  double d = distance_m;
  if (loop_path_ && max_distance > kEpsilon) {
    d = std::fmod(distance_m, max_distance);
    if (d < 0.0) {
      d += max_distance;
    }
  } else {
    d = std::clamp(d, 0.0, max_distance);
  }

  size_t segment_index = 1;
  while (segment_index < trajectory_path_.size() && trajectory_path_[segment_index].distance_m < d) {
    ++segment_index;
  }
  if (segment_index >= trajectory_path_.size()) {
    segment_index = trajectory_path_.size() - 1;
  }
  if (segment_index == 0) {
    segment_index = 1;
  }

  const auto &a = trajectory_path_[segment_index - 1];
  const auto &b = trajectory_path_[segment_index];
  const double segment_length = std::max(kEpsilon, b.distance_m - a.distance_m);
  const double t = std::clamp((d - a.distance_m) / segment_length, 0.0, 1.0);

  lat = a.lat + (b.lat - a.lat) * t;
  lon = a.lon + (b.lon - a.lon) * t;

  const double y = std::sin(degToRad(b.lon - a.lon)) * std::cos(degToRad(b.lat));
  const double x = std::cos(degToRad(a.lat)) * std::sin(degToRad(b.lat)) -
                   std::sin(degToRad(a.lat)) * std::cos(degToRad(b.lat)) *
                       std::cos(degToRad(b.lon - a.lon));
  course_deg = std::fmod(radToDeg(std::atan2(y, x)) + 360.0, 360.0);
  return true;
}

void AtosTruckSimulator::simulationStep() {
  const auto now_time = now();
  const double dt = std::max(0.001, (now_time - last_step_time_).seconds());
  last_step_time_ = now_time;

  const double target_speed_mps = std::max(0.0, target_speed_kmh_ / 3.6);
  const double max_delta = acceleration_mps2_ * dt;
  const double delta = std::clamp(target_speed_mps - current_speed_mps_, -max_delta, max_delta);
  current_speed_mps_ += delta;

  current_distance_m_ += current_speed_mps_ * dt;
  if (!loop_path_) {
    current_distance_m_ = std::clamp(current_distance_m_, 0.0, trajectory_path_.back().distance_m);
  }

  double lat = 0.0;
  double lon = 0.0;
  double course_deg = 0.0;
  if (!pointAtDistance(current_distance_m_, lat, lon, course_deg)) {
    return;
  }

  if (!ensureTcpConnected()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Unable to connect to TruckObjectControl TCP %s:%d",
                         tcp_host_.c_str(), tcp_port_);
    return;
  }

  const std::string time_str = utcIso8601FromRosTime(now_time);
  const std::string stale_str = utcIso8601FromRosTime(now_time + rclcpp::Duration::from_seconds(10.0));

  std::ostringstream cot;
  cot << "<event version=\"2.0\" uid=\"" << uid_ << "\" type=\"a-f-G-U-C-I\" time=\"" << time_str
      << "\" start=\"" << time_str << "\" stale=\"" << stale_str
      << "\" how=\"h-e\"><point lat=\"" << std::setprecision(15) << lat << "\" lon=\"" << lon
      << "\" speed=\"" << current_speed_mps_
      << "\" hae=\"10\" ce=\"5\" le=\"5\"/><detail><contact callsign=\"" << uid_
      << "\"/><__group name=\"Dark Green\" role=\"Test Vehicle\"/><takv device=\"AtosTruckSimulator\" "
         "platform=\"ATOSFleetManagement\" os=\"Linux\" version=\"1.0\"/><track speed=\""
      << current_speed_mps_ << "\" course=\"" << course_deg << "\"/></detail></event>";

  const std::string payload = cot.str();
  const ssize_t sent = ::send(tcp_fd_, payload.data(), payload.size(), MSG_NOSIGNAL);
  if (sent < 0) {
    RCLCPP_WARN(get_logger(), "TCP send failed, reconnecting...");
    closeTcp();
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "Sim uid=%s distance=%.1f m speed=%.1f km/h target=%.1f km/h lat=%.7f lon=%.7f",
                       uid_.c_str(), current_distance_m_, current_speed_mps_ * 3.6, target_speed_kmh_, lat, lon);
}

void AtosTruckSimulator::onSpeedCommand(const std_msgs::msg::String::SharedPtr msg) {
  const auto key = std::string("target_speed_kmh=");
  const auto pos = msg->data.find(key);
  if (pos == std::string::npos) {
    return;
  }
  const auto value_start = pos + key.size();
  const auto value_end = msg->data.find(';', value_start);
  const auto value = msg->data.substr(value_start, value_end - value_start);
  try {
    target_speed_kmh_ = std::max(0.0, std::stod(value));
  } catch (...) {
    return;
  }
}
