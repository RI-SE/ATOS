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
#include <cstdint>
#include <memory>
#include <mutex>
#include <openssl/ssl.h>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class TruckObjectControl : public rclcpp::Node {
public:
	TruckObjectControl();
	~TruckObjectControl() override;
	TruckObjectControl(const TruckObjectControl&) = delete;
	TruckObjectControl& operator=(const TruckObjectControl&) = delete;
	TruckObjectControl(TruckObjectControl&&) = delete;
	TruckObjectControl& operator=(TruckObjectControl&&) = delete;

private:
	struct GeoPoint {
		double lat		  = 0.0;
		double lon		  = 0.0;
		double distance_m = 0.0;
	};

	struct TruckState {
		double distance_along_trajectory_m = 0.0;
		double lat						   = 0.0;
		double lon						   = 0.0;
		double speed_mps				   = 0.0;
		double course_deg				   = 0.0;
		bool tcp_connected				   = false;
		std::string path_name			   = "";
		int path_index					   = -1;
		std::string last_control_command   = "";
		std::string last_tcp_command	   = "";
		std::string last_tcp_warning	   = "";
		std::string last_cot_message	   = "";
		rclcpp::Time last_cot_stamp		   = rclcpp::Time(0, 0, RCL_ROS_TIME);
	};

	struct CotObservation {
		std::string truck_id;
		double distance_along_trajectory_m = 0.0;
		double lat						   = 0.0;
		double lon						   = 0.0;
		double speed_mps				   = 0.0;
		double course_deg				   = 0.0;
		bool tcp_connected				   = false;
		std::string path_name			   = "";
		int path_index					   = -1;
	};

	struct TcpClientSession {
		int fd				  = -1;
		std::string peer_name = "";
		SSL* ssl			  = nullptr;
		std::mutex io_mutex;
		std::mutex queue_mutex;
		std::condition_variable queue_cv;
		std::thread sender_thread;
		std::string queued_uid	   = "";
		std::string queued_command = "";
		bool has_queued_command	   = false;
		bool stop_sender		   = false;
	};

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_cot_sub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_speed_command_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_truck_state_pub;
	rclcpp::TimerBase::SharedPtr m_evaluation_timer;

	std::mutex m_state_mutex;
	std::unordered_map<std::string, TruckState> m_trucks;
	std::vector<GeoPoint> m_trajectory_path;
	std::unordered_map<std::string, std::vector<GeoPoint>> m_trajectory_cache;
	std::mutex m_trajectory_cache_mutex;

	double m_warning_distance_m			 = 400.0;
	double m_stop_distance_m				 = 200.0;
	double m_warning_speed_kmh			 = 30.0;
	double m_stop_speed_kmh				 = 0.0;
	double m_cot_timeout_seconds			 = 2.0;
	int m_cot_tcp_port					 = 8114;
	std::string m_cot_tcp_bind_address	 = "0.0.0.0";
	bool m_cot_tls_require_client_cert	 = false;
	std::string m_cot_tls_cert_path		 = "";
	std::string m_cot_tls_key_path		 = "";
	std::string m_cot_tls_ca_path		 = "";
	bool m_cot_tls_enabled				 = false;
	std::string m_trajectory_geojson_path = "";
	std::string m_default_path_name		 = "";

	std::atomic<bool> m_tcp_running{false};
	int m_tcp_server_fd = -1;
	std::thread m_tcp_accept_thread;
	std::vector<std::thread> m_tcp_client_threads;
	std::mutex m_tcp_threads_mutex;
	std::set<int> m_tcp_client_fds;
	std::mutex m_tcp_command_mutex;
	std::mutex m_tcp_sessions_mutex;
	std::unordered_map<int, std::shared_ptr<TcpClientSession>> m_tcp_sessions;
	std::unordered_map<std::string, int> m_uid_to_client_fd;
	std::unordered_map<std::string, SSL*> m_uid_to_ssl;
	SSL_CTX* m_ssl_ctx		  = nullptr;
	uint64_t m_tcp_command_seq = 0;

	void onCotMessage(const std_msgs::msg::String::SharedPtr msg);
	void evaluateAndPublishSpeedCommand();
	void publishTruckState(const std::string& truck_id, const TruckState& state);

	bool parseCotPlaceholder(const std::string& payload, CotObservation& out) const;
	bool parseCotXml(const std::string& payload, CotObservation& out);
	bool isCotFresh(const TruckState& state, const rclcpp::Time& now) const;
	bool loadTrajectoryPath();
	bool loadTrajectoryPathFromFile(const std::string& path, std::vector<GeoPoint>& out) const;
	std::string resolveTrajectoryPathByName(const std::string& path_name) const;
	const std::vector<GeoPoint>* getTrajectoryForPath(const std::string& path_name);
	double projectDistanceAlongTrajectory(const double lat,
										  const double lon,
										  const std::vector<GeoPoint>* trajectory = nullptr,
										  int* projected_path_index				  = nullptr) const;

	void startTcpServer();
	void stopTcpServer();
	void acceptTcpClients();
	void handleTcpClient(const int client_fd, const std::string& peer_name);
	void clientSenderLoop(const std::shared_ptr<TcpClientSession>& session);
	void disconnectClientSession(const std::shared_ptr<TcpClientSession>& session, const std::string& reason);
	void updateTruckTcpStatus(const std::string& target_id,
							  const std::string& command,
							  const std::string& warning,
							  bool mark_disconnected = false);
	void sendSpeedCommandToTcpClient(const std::string& target_id, const std::string& command);
	bool initializeTlsContext();
};
