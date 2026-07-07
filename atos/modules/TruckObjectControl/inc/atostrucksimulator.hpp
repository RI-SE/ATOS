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
		double lat					 = 0.0;
		double lon					 = 0.0;
		double distance_along_path_m = 0.0;
	};

	std::string m_uid					  = "L5S-TRUCK-SIM";
	std::string m_tcp_host				  = "127.0.0.1";
	int m_tcp_port						  = 8114;
	std::string m_trajectory_geojson_path = "";
	std::string m_trajectory_path_name	  = "";
	int m_start_index					  = 0;
	double m_initial_speed_kmh			  = 0.0;
	double m_cruise_target_speed_kmh	  = 40.0;
	double m_target_speed_kmh			  = 40.0;
	double m_acceleration_mps2			  = 2.0;
	double m_publish_hz					  = 5.0;
	double m_lateral_offset_m			  = 0.0;
	bool m_loop_path					  = true;
	bool m_ignore_warning_speed_commands  = false;

	double m_current_distance_m = 0.0;
	double m_current_speed_mps	= 0.0;

	int m_tcp_fd = -1;
	std::string m_tcp_rx_buffer;
	std::vector<GeoPoint> m_trajectory_path;

	rclcpp::TimerBase::SharedPtr m_simulation_timer;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_speed_command_sub;

	rclcpp::Time m_last_step_time;

	bool loadTrajectoryPath();
	bool ensureTcpConnected();
	void closeTcp();
	void pollTcpCommands();
	void simulationStep();
	bool pointAtDistance(double distance_m, double& lat, double& lon, double& course_deg, int& path_index) const;
	static std::string utcIso8601FromRosTime(const rclcpp::Time& time);
	void applySpeedCommandPayload(const std::string& payload);
	void onSpeedCommand(const std_msgs::msg::String::SharedPtr msg);
};
