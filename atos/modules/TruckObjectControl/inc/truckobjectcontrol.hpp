/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <openssl/ssl.h>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <cstdint>

class TruckObjectControl : public rclcpp::Node {
public:
  TruckObjectControl();
  ~TruckObjectControl() override;

private:
  struct GeoPoint {
    double lat = 0.0;
    double lon = 0.0;
    double distance_m = 0.0;
  };

  struct TruckState {
    double distance_along_trajectory_m = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double speed_mps = 0.0;
    double course_deg = 0.0;
    bool tcp_connected = false;
    std::string path_name = "";
    int path_index = -1;
    std::string last_control_command = "";
    std::string last_tcp_command = "";
    std::string last_tcp_warning = "";
    std::string last_cot_message = "";
    rclcpp::Time last_cot_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  };

  struct CotObservation {
    std::string truck_id;
    double distance_along_trajectory_m = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double speed_mps = 0.0;
    double course_deg = 0.0;
    bool tcp_connected = false;
    std::string path_name = "";
    int path_index = -1;
  };

  struct TcpClientSession {
    int fd = -1;
    std::string peer_name = "";
    SSL *ssl = nullptr;
    std::mutex io_mutex;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::thread sender_thread;
    std::string queued_uid = "";
    std::string queued_command = "";
    bool has_queued_command = false;
    bool stop_sender = false;
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cot_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr truck_state_pub_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;

  std::mutex state_mutex_;
  std::unordered_map<std::string, TruckState> trucks_;
  std::vector<GeoPoint> trajectory_path_;
  std::unordered_map<std::string, std::vector<GeoPoint>> trajectory_cache_;
  std::mutex trajectory_cache_mutex_;

  double warning_distance_m_ = 400.0;
  double stop_distance_m_ = 200.0;
  double warning_speed_kmh_ = 30.0;
  double stop_speed_kmh_ = 0.0;
  double cot_timeout_seconds_ = 2.0;
  int cot_tcp_port_ = 8114;
  std::string cot_tcp_bind_address_ = "0.0.0.0";
  bool cot_tls_require_client_cert_ = false;
  std::string cot_tls_cert_path_ = "";
  std::string cot_tls_key_path_ = "";
  std::string cot_tls_ca_path_ = "";
  bool cot_tls_enabled_ = false;
  std::string trajectory_geojson_path_ = "";
  std::string default_path_name_ = "";

  std::atomic<bool> tcp_running_{false};
  int tcp_server_fd_ = -1;
  std::thread tcp_accept_thread_;
  std::vector<std::thread> tcp_client_threads_;
  std::mutex tcp_threads_mutex_;
  std::set<int> tcp_client_fds_;
  std::mutex tcp_command_mutex_;
  std::mutex tcp_sessions_mutex_;
  std::unordered_map<int, std::shared_ptr<TcpClientSession>> tcp_sessions_;
  std::unordered_map<std::string, int> uid_to_client_fd_;
  std::unordered_map<std::string, SSL *> uid_to_ssl_;
  SSL_CTX *ssl_ctx_ = nullptr;
  uint64_t tcp_command_seq_ = 0;

  void onCotMessage(const std_msgs::msg::String::SharedPtr msg);
  void evaluateAndPublishSpeedCommand();
  void publishTruckState(const std::string &truck_id, const TruckState &state);

  bool parseCotPlaceholder(const std::string &payload, CotObservation &out) const;
  bool parseCotXml(const std::string &payload, CotObservation &out);
  bool isCotFresh(const TruckState &state, const rclcpp::Time &now) const;
  bool loadTrajectoryPath();
  bool loadTrajectoryPathFromFile(const std::string &path, std::vector<GeoPoint> &out) const;
  std::string resolveTrajectoryPathByName(const std::string &path_name) const;
  const std::vector<GeoPoint> *getTrajectoryForPath(const std::string &path_name);
  double projectDistanceAlongTrajectory(double lat, double lon,
                                        const std::vector<GeoPoint> *trajectory = nullptr,
                                        int *projected_path_index = nullptr) const;

  void startTcpServer();
  void stopTcpServer();
  void acceptTcpClients();
  void handleTcpClient(int client_fd, const std::string &peer_name);
  void clientSenderLoop(const std::shared_ptr<TcpClientSession> &session);
  void disconnectClientSession(const std::shared_ptr<TcpClientSession> &session, const std::string &reason);
  void updateTruckTcpStatus(const std::string &target_id, const std::string &command, const std::string &warning,
                            bool mark_disconnected = false);
  void sendSpeedCommandToTcpClient(const std::string &target_id, const std::string &command);
  bool initializeTlsContext();
};
