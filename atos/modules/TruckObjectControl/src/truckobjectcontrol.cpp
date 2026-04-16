/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "truckobjectcontrol.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <vector>

using std::placeholders::_1;
using json = nlohmann::json;

namespace {
constexpr size_t kReceiveBufferSize = 4096;
constexpr int kAcceptPollSleepMs = 100;
constexpr const char *kDefaultGeoJsonName = "RuralRoad_center_of_driving_lane_ccw.geojson";

double degToRad(double value) {
  return value * M_PI / 180.0;
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

std::string resolveDefaultTrajectoryPath(const std::string &configured_path) {
  namespace fs = std::filesystem;
  if (!configured_path.empty() && fs::exists(configured_path)) {
    return configured_path;
  }

  std::vector<fs::path> candidates;
  candidates.emplace_back(fs::current_path() / "conf" / "conf" / kDefaultGeoJsonName);
  candidates.emplace_back(fs::current_path() / ".." / "conf" / "conf" / kDefaultGeoJsonName);

  if (const char *home = std::getenv("HOME")) {
    candidates.emplace_back(fs::path(home) / "atos_ws" / "src" / "atos" / "conf" / "conf" / kDefaultGeoJsonName);
    candidates.emplace_back(fs::path(home) / "Documents" / "repos" / "ATOS" / "conf" / "conf" / kDefaultGeoJsonName);
  }

  try {
    const auto prefix = fs::path(ament_index_cpp::get_package_prefix("atos"));
    candidates.emplace_back(prefix / "etc" / "conf" / kDefaultGeoJsonName);
  } catch (...) {
  }

  for (const auto &candidate : candidates) {
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }
  return configured_path;
}
} // namespace

TruckObjectControl::TruckObjectControl() : Node("truck_object_control") {
  declare_parameter("warning_distance_m", warning_distance_m_);
  declare_parameter("stop_distance_m", stop_distance_m_);
  declare_parameter("warning_speed_kmh", warning_speed_kmh_);
  declare_parameter("stop_speed_kmh", stop_speed_kmh_);
  declare_parameter("cot_timeout_seconds", cot_timeout_seconds_);
  declare_parameter("cot_tcp_port", cot_tcp_port_);
  declare_parameter("cot_tcp_bind_address", cot_tcp_bind_address_);
  declare_parameter("trajectory_geojson_path", trajectory_geojson_path_);

  warning_distance_m_ = get_parameter("warning_distance_m").as_double();
  stop_distance_m_ = get_parameter("stop_distance_m").as_double();
  warning_speed_kmh_ = get_parameter("warning_speed_kmh").as_double();
  stop_speed_kmh_ = get_parameter("stop_speed_kmh").as_double();
  cot_timeout_seconds_ = get_parameter("cot_timeout_seconds").as_double();
  cot_tcp_port_ = get_parameter("cot_tcp_port").as_int();
  cot_tcp_bind_address_ = get_parameter("cot_tcp_bind_address").as_string();
  trajectory_geojson_path_ = get_parameter("trajectory_geojson_path").as_string();
  trajectory_geojson_path_ = resolveDefaultTrajectoryPath(trajectory_geojson_path_);
  default_path_name_ = std::filesystem::path(trajectory_geojson_path_).filename().string();

  cot_sub_ = create_subscription<std_msgs::msg::String>(
      "truck_objects/cot", 50, std::bind(&TruckObjectControl::onCotMessage, this, _1));

  speed_command_pub_ = create_publisher<std_msgs::msg::String>("truck_objects/speed_command", 20);
  truck_state_pub_ = create_publisher<std_msgs::msg::String>("truck_objects/state", 50);

  evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&TruckObjectControl::evaluateAndPublishSpeedCommand, this));

  if (!loadTrajectoryPath()) {
    RCLCPP_WARN(get_logger(),
                "Failed to load trajectory path from '%s'. Distance along trajectory will stay 0.",
                trajectory_geojson_path_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Loaded trajectory path with %zu points from %s", trajectory_path_.size(),
                trajectory_geojson_path_.c_str());
  }

  startTcpServer();

  RCLCPP_INFO(get_logger(),
              "TruckObjectControl started. Listening for COT XML on tcp://%s:%d and placeholder topic 'truck_objects/cot'.",
              cot_tcp_bind_address_.c_str(), cot_tcp_port_);
}

TruckObjectControl::~TruckObjectControl() {
  stopTcpServer();
}

void TruckObjectControl::publishTruckState(const std::string &truck_id, const TruckState &state) {
  json payload;
  payload["uid"] = truck_id;
  payload["distance_m"] = state.distance_along_trajectory_m;
  payload["lat"] = state.lat;
  payload["lon"] = state.lon;
  payload["speed_mps"] = state.speed_mps;
  payload["speed_kmh"] = state.speed_mps * 3.6;
  payload["course_deg"] = state.course_deg;
  payload["tcp_connected"] = state.tcp_connected;
  payload["path_name"] = state.path_name;
  payload["path_index"] = state.path_index;
  payload["stamp_sec"] = now().seconds();

  std_msgs::msg::String msg;
  msg.data = payload.dump();
  truck_state_pub_->publish(msg);
}

void TruckObjectControl::onCotMessage(const std_msgs::msg::String::SharedPtr msg) {
  CotObservation observation;
  if (!parseCotPlaceholder(msg->data, observation) && !parseCotXml(msg->data, observation)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Failed to parse COT payload from ROS topic.");
    return;
  }

  TruckState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto &entry = trucks_[observation.truck_id];
    entry.distance_along_trajectory_m = observation.distance_along_trajectory_m;
    entry.lat = observation.lat;
    entry.lon = observation.lon;
    entry.speed_mps = observation.speed_mps;
    entry.course_deg = observation.course_deg;
    entry.tcp_connected = observation.tcp_connected;
    entry.path_name = observation.path_name;
    entry.path_index = observation.path_index;
    entry.last_cot_stamp = now();
    state = entry;
  }

  publishTruckState(observation.truck_id, state);
}

bool TruckObjectControl::parseCotPlaceholder(const std::string &payload, CotObservation &out) const {
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

  out.truck_id = fields.at("id");
  if (out.truck_id.empty()) {
    return false;
  }

  try {
    out.distance_along_trajectory_m = std::stod(fields.at("distance_m"));
    const auto &tcp = fields.at("tcp_connected");
    out.tcp_connected = (tcp == "1" || tcp == "true" || tcp == "TRUE");

    if (fields.find("lat") != fields.end() && fields.find("lon") != fields.end()) {
      out.lat = std::stod(fields.at("lat"));
      out.lon = std::stod(fields.at("lon"));
    }
    if (fields.find("speed_mps") != fields.end()) {
      out.speed_mps = std::stod(fields.at("speed_mps"));
    } else if (fields.find("speed_kmh") != fields.end()) {
      out.speed_mps = std::stod(fields.at("speed_kmh")) / 3.6;
    }
    if (fields.find("course_deg") != fields.end()) {
      out.course_deg = std::stod(fields.at("course_deg"));
    }
    if (fields.find("path_name") != fields.end()) {
      out.path_name = fields.at("path_name");
    }
    if (fields.find("path_index") != fields.end()) {
      out.path_index = std::stoi(fields.at("path_index"));
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool TruckObjectControl::parseCotXml(const std::string &payload, CotObservation &out) {
  static const std::regex uid_re(R"re(uid="([^"]+)")re");
  static const std::regex point_re(R"re(<point[^>]*\blat="([^"]+)"[^>]*\blon="([^"]+)")re");
  static const std::regex track_re(R"re(<track[^>]*\bspeed="([^"]+)"[^>]*\bcourse="([^"]+)")re");
  static const std::regex path_name_re(R"re(<atos[^>]*\bpath_name="([^"]+)")re");
  static const std::regex path_index_re(R"re(<atos[^>]*\bpath_index="([^"]+)")re");

  std::smatch m;
  if (!std::regex_search(payload, m, uid_re) || m.size() < 2) {
    return false;
  }
  out.truck_id = m[1].str();
  if (out.truck_id.empty()) {
    return false;
  }

  if (!std::regex_search(payload, m, point_re) || m.size() < 3) {
    return false;
  }

  try {
    out.lat = std::stod(m[1].str());
    out.lon = std::stod(m[2].str());
  } catch (...) {
    return false;
  }

  out.speed_mps = 0.0;
  out.course_deg = 0.0;
  if (std::regex_search(payload, m, track_re) && m.size() >= 3) {
    try {
      out.speed_mps = std::stod(m[1].str());
      out.course_deg = std::stod(m[2].str());
    } catch (...) {
      out.speed_mps = 0.0;
      out.course_deg = 0.0;
    }
  }

  out.path_name.clear();
  out.path_index = -1;
  if (std::regex_search(payload, m, path_name_re) && m.size() >= 2) {
    out.path_name = m[1].str();
  }
  if (std::regex_search(payload, m, path_index_re) && m.size() >= 2) {
    try {
      out.path_index = std::stoi(m[1].str());
    } catch (...) {
      out.path_index = -1;
    }
  }

  const std::vector<GeoPoint> *trajectory = nullptr;
  if (!out.path_name.empty()) {
    trajectory = getTrajectoryForPath(out.path_name);
  }
  if (!trajectory) {
    trajectory = &trajectory_path_;
    if (out.path_name.empty()) {
      out.path_name = default_path_name_;
    }
  }

  if (trajectory && out.path_index >= 0 && static_cast<size_t>(out.path_index) < trajectory->size()) {
    out.distance_along_trajectory_m = (*trajectory)[static_cast<size_t>(out.path_index)].distance_m;
  } else {
    out.distance_along_trajectory_m = projectDistanceAlongTrajectory(out.lat, out.lon, trajectory);
  }

  out.tcp_connected = true;
  return true;
}

bool TruckObjectControl::isCotFresh(const TruckState &state, const rclcpp::Time &now_time) const {
  const auto age = (now_time - state.last_cot_stamp).seconds();
  return age >= 0.0 && age <= cot_timeout_seconds_;
}

bool TruckObjectControl::loadTrajectoryPathFromFile(const std::string &path, std::vector<GeoPoint> &out) const {
  out.clear();

  std::ifstream input(path);
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

    out.push_back(p);
    prev = p;
    has_prev = true;
  }

  return out.size() >= 2;
}

bool TruckObjectControl::loadTrajectoryPath() {
  if (!loadTrajectoryPathFromFile(trajectory_geojson_path_, trajectory_path_)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(trajectory_cache_mutex_);
  trajectory_cache_[default_path_name_] = trajectory_path_;
  return true;
}

std::string TruckObjectControl::resolveTrajectoryPathByName(const std::string &path_name) const {
  namespace fs = std::filesystem;

  if (path_name.empty()) {
    return trajectory_geojson_path_;
  }

  const fs::path raw(path_name);
  if (raw.is_absolute() && fs::exists(raw)) {
    return raw.string();
  }

  const fs::path base_name = raw.filename();
  if (base_name.empty()) {
    return "";
  }

  const fs::path configured = fs::path(trajectory_geojson_path_).parent_path() / base_name;
  if (fs::exists(configured)) {
    return configured.string();
  }

  std::vector<fs::path> candidates;
  candidates.emplace_back(fs::current_path() / "conf" / "conf" / base_name);
  candidates.emplace_back(fs::current_path() / ".." / "conf" / "conf" / base_name);

  if (const char *home = std::getenv("HOME")) {
    candidates.emplace_back(fs::path(home) / "atos_ws" / "src" / "atos" / "conf" / "conf" / base_name);
    candidates.emplace_back(fs::path(home) / "Documents" / "repos" / "ATOS" / "conf" / "conf" / base_name);
  }

  try {
    const auto prefix = fs::path(ament_index_cpp::get_package_prefix("atos"));
    candidates.emplace_back(prefix / "etc" / "conf" / base_name);
  } catch (...) {
  }

  for (const auto &candidate : candidates) {
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }

  return "";
}

const std::vector<TruckObjectControl::GeoPoint> *TruckObjectControl::getTrajectoryForPath(const std::string &path_name) {
  const std::string key = std::filesystem::path(path_name).filename().string();
  if (key.empty()) {
    return nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(trajectory_cache_mutex_);
    const auto it = trajectory_cache_.find(key);
    if (it != trajectory_cache_.end()) {
      return &it->second;
    }
  }

  const std::string resolved_path = resolveTrajectoryPathByName(key);
  if (resolved_path.empty()) {
    return nullptr;
  }

  std::vector<GeoPoint> loaded;
  if (!loadTrajectoryPathFromFile(resolved_path, loaded)) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(trajectory_cache_mutex_);
  auto [it, inserted] = trajectory_cache_.emplace(key, std::move(loaded));
  if (inserted) {
    RCLCPP_INFO(get_logger(), "Loaded trajectory '%s' with %zu points from %s", key.c_str(), it->second.size(),
                resolved_path.c_str());
  }
  return &it->second;
}

double TruckObjectControl::projectDistanceAlongTrajectory(double lat, double lon,
                                                          const std::vector<GeoPoint> *trajectory) const {
  const std::vector<GeoPoint> *path = trajectory;
  if (!path || path->empty()) {
    path = &trajectory_path_;
  }
  if (!path || path->empty()) {
    return 0.0;
  }

  double best_distance_to_path_m = std::numeric_limits<double>::infinity();
  double best_distance_along_m = 0.0;
  for (const auto &p : *path) {
    const double d = geodesicDistanceMeters(lat, lon, p.lat, p.lon);
    if (d < best_distance_to_path_m) {
      best_distance_to_path_m = d;
      best_distance_along_m = p.distance_m;
    }
  }
  return best_distance_along_m;
}

void TruckObjectControl::startTcpServer() {
  tcp_server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create TCP socket for COT listener.");
    return;
  }

  int enable = 1;
  (void)setsockopt(tcp_server_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  (void)setsockopt(tcp_server_fd_, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(cot_tcp_port_));
  if (::inet_pton(AF_INET, cot_tcp_bind_address_.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "Invalid bind address '%s' for COT TCP listener.",
                 cot_tcp_bind_address_.c_str());
    ::close(tcp_server_fd_);
    tcp_server_fd_ = -1;
    return;
  }

  if (::bind(tcp_server_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind COT TCP listener on %s:%d", cot_tcp_bind_address_.c_str(),
                 cot_tcp_port_);
    ::close(tcp_server_fd_);
    tcp_server_fd_ = -1;
    return;
  }

  if (::listen(tcp_server_fd_, 8) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to listen on COT TCP listener.");
    ::close(tcp_server_fd_);
    tcp_server_fd_ = -1;
    return;
  }

  tcp_running_.store(true);
  tcp_accept_thread_ = std::thread(&TruckObjectControl::acceptTcpClients, this);
}

void TruckObjectControl::stopTcpServer() {
  tcp_running_.store(false);

  if (tcp_server_fd_ >= 0) {
    ::shutdown(tcp_server_fd_, SHUT_RDWR);
    ::close(tcp_server_fd_);
    tcp_server_fd_ = -1;
  }

  if (tcp_accept_thread_.joinable()) {
    tcp_accept_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(tcp_threads_mutex_);
    for (const int fd : tcp_client_fds_) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    tcp_client_fds_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(tcp_command_mutex_);
    uid_to_client_fd_.clear();
  }

  std::lock_guard<std::mutex> lock(tcp_threads_mutex_);
  for (auto &thread : tcp_client_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
  tcp_client_threads_.clear();
}

void TruckObjectControl::acceptTcpClients() {
  while (tcp_running_.load()) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    const int client_fd =
        ::accept(tcp_server_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
    if (client_fd < 0) {
      if (!tcp_running_.load()) {
        break;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "TCP accept failed (errno=%d). Listener will continue.", errno);
      std::this_thread::sleep_for(std::chrono::milliseconds(kAcceptPollSleepMs));
      continue;
    }

    int keepalive = 1;
    (void)setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    timeval recv_timeout{};
    recv_timeout.tv_sec = 1;
    recv_timeout.tv_usec = 0;
    (void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

    char ip_buf[INET_ADDRSTRLEN] = {0};
    const char *peer_ip = ::inet_ntop(AF_INET, &client_addr.sin_addr, ip_buf, sizeof(ip_buf));
    std::ostringstream peer;
    peer << (peer_ip ? peer_ip : "unknown") << ":" << ntohs(client_addr.sin_port);

    RCLCPP_INFO(get_logger(), "Accepted TruckObject TCP client %s", peer.str().c_str());

    std::lock_guard<std::mutex> lock(tcp_threads_mutex_);
    tcp_client_fds_.insert(client_fd);
    tcp_client_threads_.emplace_back(&TruckObjectControl::handleTcpClient, this, client_fd, peer.str());
  }
}

void TruckObjectControl::handleTcpClient(int client_fd, const std::string &peer_name) {
  std::string buffer;
  buffer.reserve(8 * 1024);
  std::set<std::string> seen_uids;

  try {
    char receive_buffer[kReceiveBufferSize];
    while (tcp_running_.load()) {
      const ssize_t received = ::recv(client_fd, receive_buffer, sizeof(receive_buffer), 0);
      if (received == 0) {
        break;
      }
      if (received < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_WARN(get_logger(),
                    "TCP receive error from %s (errno=%d). Closing this connection only.",
                    peer_name.c_str(), errno);
        break;
      }

      buffer.append(receive_buffer, static_cast<size_t>(received));

      while (true) {
        const size_t start_pos = buffer.find("<event");
        if (start_pos == std::string::npos) {
          if (buffer.size() > 32 * 1024) {
            buffer.clear();
          }
          break;
        }
        const size_t end_pos = buffer.find("</event>", start_pos);
        if (end_pos == std::string::npos) {
          if (start_pos > 0) {
            buffer.erase(0, start_pos);
          }
          break;
        }

        const size_t event_end = end_pos + std::string("</event>").size();
        const std::string xml = buffer.substr(start_pos, event_end - start_pos);
        buffer.erase(0, event_end);

        CotObservation observation;
        if (!parseCotXml(xml, observation)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Failed to parse COT XML from TCP client %s", peer_name.c_str());
          continue;
        }

        TruckState state;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          auto &entry = trucks_[observation.truck_id];
          entry.distance_along_trajectory_m = observation.distance_along_trajectory_m;
          entry.lat = observation.lat;
          entry.lon = observation.lon;
          entry.speed_mps = observation.speed_mps;
          entry.course_deg = observation.course_deg;
          entry.tcp_connected = true;
          entry.path_name = observation.path_name;
          entry.path_index = observation.path_index;
          entry.last_cot_stamp = now();
          state = entry;
        }
        {
          std::lock_guard<std::mutex> lock(tcp_command_mutex_);
          uid_to_client_fd_[observation.truck_id] = client_fd;
        }

        seen_uids.insert(observation.truck_id);
        publishTruckState(observation.truck_id, state);
      }
    }
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Unexpected exception while handling client %s. Connection will be closed.",
                peer_name.c_str());
  }

  for (const auto &uid : seen_uids) {
    TruckState state;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto it = trucks_.find(uid);
      if (it != trucks_.end()) {
        it->second.tcp_connected = false;
        it->second.last_cot_stamp = now();
        state = it->second;
        found = true;
      }
    }
    if (found) {
      publishTruckState(uid, state);
    }
    {
      std::lock_guard<std::mutex> lock(tcp_command_mutex_);
      const auto it = uid_to_client_fd_.find(uid);
      if (it != uid_to_client_fd_.end() && it->second == client_fd) {
        uid_to_client_fd_.erase(it);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(tcp_threads_mutex_);
    tcp_client_fds_.erase(client_fd);
  }
  ::shutdown(client_fd, SHUT_RDWR);
  ::close(client_fd);
  RCLCPP_INFO(get_logger(), "TruckObject TCP client disconnected: %s", peer_name.c_str());
}

void TruckObjectControl::evaluateAndPublishSpeedCommand() {
  struct ConnectedTruck {
    std::string id;
    double distance_m = 0.0;
    int path_index = -1;
  };
  std::vector<ConnectedTruck> connected;

  const auto now_time = now();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto &[id, state] : trucks_) {
      if (!state.tcp_connected || !isCotFresh(state, now_time)) {
        continue;
      }
      connected.push_back(ConnectedTruck{id, state.distance_along_trajectory_m, state.path_index});
    }
  }

  if (connected.size() < 2) {
    return;
  }

  std::sort(connected.begin(), connected.end(),
            [](const ConnectedTruck &a, const ConnectedTruck &b) { return a.distance_m < b.distance_m; });

  double min_gap_m = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < connected.size(); ++i) {
    min_gap_m = std::min(min_gap_m, std::fabs(connected[i].distance_m - connected[i - 1].distance_m));
  }

  double target_speed_mps = -1.0;
  std::string reason = "no_limit";

  if (min_gap_m < stop_distance_m_) {
    target_speed_mps = stop_speed_kmh_ / 3.6;
    reason = "min_gap_below_stop_distance";
  } else if (min_gap_m < warning_distance_m_) {
    target_speed_mps = warning_speed_kmh_ / 3.6;
    reason = "min_gap_below_warning_distance";
  }

  if (target_speed_mps < 0.0) {
    return;
  }

  if (std::fabs(last_published_speed_mps_ - target_speed_mps) < 1e-6) {
    return;
  }

  // Keep ROS command as a global signal while TCP commands are individualized per truck.
  std_msgs::msg::String command;
  command.data = "target_speed_mps=" + std::to_string(target_speed_mps) +
                 ";scope=all_connected_with_valid_tcp_and_fresh_cot" + ";reason=" + reason +
                 ";min_gap_m=" + std::to_string(min_gap_m) +
                 ";connected_count=" + std::to_string(connected.size());

  speed_command_pub_->publish(command);
  for (size_t i = 0; i < connected.size(); ++i) {
    const bool has_ahead = (i + 1) < connected.size();
    const std::string ahead_uid = has_ahead ? connected[i + 1].id : "none";
    const double ahead_gap = has_ahead ? (connected[i + 1].distance_m - connected[i].distance_m) : -1.0;
    const int ahead_path_index = has_ahead ? connected[i + 1].path_index : -1;

    const std::string tcp_command =
        "target_speed_mps=" + std::to_string(target_speed_mps) + ";distance_to_truck_ahead_m=" +
        std::to_string(ahead_gap) + ";truck_ahead_path_index=" + std::to_string(ahead_path_index) +
        ";truck_ahead_uid=" + ahead_uid + ";reason=" + reason + ";min_gap_m=" + std::to_string(min_gap_m) +
        ";connected_count=" + std::to_string(connected.size());
    sendSpeedCommandToTcpClient(connected[i].id, tcp_command);
  }
  last_published_speed_mps_ = target_speed_mps;

  RCLCPP_WARN(get_logger(),
              "Published speed command %.2f m/s (reason=%s, min_gap=%.2f m, connected=%zu, first_id=%s)",
              target_speed_mps, reason.c_str(), min_gap_m, connected.size(),
              connected.empty() ? "-" : connected.front().id.c_str());
}

void TruckObjectControl::sendSpeedCommandToTcpClient(const std::string &target_id, const std::string &command) {
  int target_fd = -1;
  {
    std::lock_guard<std::mutex> lock(tcp_command_mutex_);
    const auto it = uid_to_client_fd_.find(target_id);
    if (it != uid_to_client_fd_.end()) {
      target_fd = it->second;
    }
  }

  if (target_fd < 0) {
    return;
  }

  const std::string payload = command + "\n";
  const ssize_t sent = ::send(target_fd, payload.data(), payload.size(), MSG_NOSIGNAL);
  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                         "Failed to send TCP speed command to uid=%s fd=%d (errno=%d)", target_id.c_str(),
                         target_fd, errno);
  }
}
