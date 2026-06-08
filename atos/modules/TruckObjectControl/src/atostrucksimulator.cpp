/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "atostrucksimulator.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>

using json = nlohmann::json;

namespace {
constexpr double kEpsilon		   = 1e-9;
constexpr const char* kGeoJsonName = "RuralRoad_center_of_driving_lane_ccw.geojson";

double degToRad(double value) {
	return value * M_PI / 180.0;
}

double radToDeg(double value) {
	return value * 180.0 / M_PI;
}

double geodesicDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
	const double earth_radius_m = 6378137.0;
	const double dlat			= degToRad(lat2 - lat1);
	const double dlon			= degToRad(lon2 - lon1);
	const double a				= std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
					 std::cos(degToRad(lat1)) * std::cos(degToRad(lat2)) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
	const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
	return earth_radius_m * c;
}

std::string resolveTrajectoryPath(const std::string& configured_path) {
	namespace fs = std::filesystem;
	if (!configured_path.empty() && fs::exists(configured_path)) {
		return configured_path;
	}

	std::vector<fs::path> candidates;
	candidates.emplace_back(fs::current_path() / "conf" / "conf" / kGeoJsonName);
	candidates.emplace_back(fs::current_path() / ".." / "conf" / "conf" / kGeoJsonName);

	if (const char* home = std::getenv("HOME")) {
		candidates.emplace_back(fs::path(home) / "atos_ws" / "src" / "atos" / "conf" / "conf" / kGeoJsonName);
		candidates.emplace_back(fs::path(home) / "Documents" / "repos" / "ATOS" / "conf" / "conf" / kGeoJsonName);
	}

	try {
		const auto prefix = fs::path(ament_index_cpp::get_package_prefix("atos"));
		candidates.emplace_back(prefix / "etc" / "conf" / kGeoJsonName);
	} catch (...) {}

	for (const auto& candidate : candidates) {
		if (fs::exists(candidate)) {
			return candidate.string();
		}
	}
	return configured_path;
}
} // namespace

AtosTruckSimulator::AtosTruckSimulator() :
  Node("atos_truck_simulator") {
	declare_parameter("uid", m_uid);
	declare_parameter("tcp_host", m_tcp_host);
	declare_parameter("tcp_port", m_tcp_port);
	declare_parameter("trajectory_geojson_path", m_trajectory_geojson_path);
	declare_parameter("start_index", m_start_index);
	declare_parameter("initial_speed_kmh", m_initial_speed_kmh);
	declare_parameter("target_speed_kmh", m_target_speed_kmh);
	declare_parameter("acceleration_mps2", m_acceleration_mps2);
	declare_parameter("publish_hz", m_publish_hz);
	declare_parameter("loop_path", m_loop_path);
	declare_parameter("ignore_warning_speed_commands", m_ignore_warning_speed_commands);

	m_uid						   = get_parameter("uid").as_string();
	m_tcp_host					   = get_parameter("tcp_host").as_string();
	m_tcp_port					   = get_parameter("tcp_port").as_int();
	m_trajectory_geojson_path	   = get_parameter("trajectory_geojson_path").as_string();
	m_trajectory_geojson_path	   = resolveTrajectoryPath(m_trajectory_geojson_path);
	m_trajectory_path_name		   = std::filesystem::path(m_trajectory_geojson_path).filename().string();
	m_start_index				   = get_parameter("start_index").as_int();
	m_initial_speed_kmh			   = get_parameter("initial_speed_kmh").as_double();
	m_target_speed_kmh			   = get_parameter("target_speed_kmh").as_double();
	m_acceleration_mps2			   = std::max(0.01, get_parameter("acceleration_mps2").as_double());
	m_publish_hz					   = std::max(1.0, get_parameter("publish_hz").as_double());
	m_loop_path					   = get_parameter("loop_path").as_bool();
	m_ignore_warning_speed_commands = get_parameter("ignore_warning_speed_commands").as_bool();

	if (!loadTrajectoryPath()) {
		RCLCPP_ERROR(get_logger(), "Failed to load trajectory at '%s'", m_trajectory_geojson_path.c_str());
		return;
	}

	if (m_start_index < 0) {
		m_start_index = 0;
	}
	if (static_cast<size_t>(m_start_index) >= m_trajectory_path.size()) {
		m_start_index = static_cast<int>(m_trajectory_path.size() - 1);
	}

	m_current_distance_m = m_trajectory_path[static_cast<size_t>(m_start_index)].distance_along_path_m;
	m_current_speed_mps	 = std::max(0.0, m_initial_speed_kmh / 3.6);
	m_last_step_time		 = now();

	m_speed_command_sub = create_subscription<std_msgs::msg::String>(
	  "truck_objects/speed_command", 20, std::bind(&AtosTruckSimulator::onSpeedCommand, this, std::placeholders::_1));

	const auto period_ms = static_cast<int>(1000.0 / m_publish_hz);
	m_simulation_timer =
	  create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&AtosTruckSimulator::simulationStep, this));

	RCLCPP_INFO(get_logger(),
				"AtosTruckSimulator started (uid=%s, path=%s, start_index=%d, target=%.1f km/h, accel=%.2f m/s^2, "
				"tcp=%s:%d, ignore_warning=%s)",
				m_uid.c_str(),
				m_trajectory_path_name.c_str(),
				m_start_index,
				m_target_speed_kmh,
				m_acceleration_mps2,
				m_tcp_host.c_str(),
				m_tcp_port,
				m_ignore_warning_speed_commands ? "true" : "false");
}

bool AtosTruckSimulator::loadTrajectoryPath() {
	m_trajectory_path.clear();

	std::ifstream input(m_trajectory_geojson_path);
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
	for (const auto& feature : root["features"]) {
		if (!feature.contains("geometry")) {
			continue;
		}
		const auto& geometry = feature["geometry"];
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

	const auto& coords = line_feature["geometry"]["coordinates"];
	if (!coords.is_array() || coords.empty()) {
		return false;
	}

	double cumulative = 0.0;
	GeoPoint prev;
	bool has_prev = false;

	for (const auto& coord : coords) {
		if (!coord.is_array() || coord.size() < 2) {
			continue;
		}
		GeoPoint p;
		p.lon					 = coord[0].get<double>();
		p.lat					 = coord[1].get<double>();
		p.distance_along_path_m = cumulative;

		if (has_prev) {
			cumulative += geodesicDistanceMeters(prev.lat, prev.lon, p.lat, p.lon);
			p.distance_along_path_m = cumulative;
		}

		m_trajectory_path.push_back(p);
		prev	 = p;
		has_prev = true;
	}

	return m_trajectory_path.size() >= 2;
}

bool AtosTruckSimulator::ensureTcpConnected() {
	if (m_tcp_fd >= 0) {
		return true;
	}

	m_tcp_fd = ::socket(AF_INET, SOCK_STREAM, 0);
	if (m_tcp_fd < 0) {
		return false;
	}

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port	= htons(static_cast<uint16_t>(m_tcp_port));
	if (::inet_pton(AF_INET, m_tcp_host.c_str(), &addr.sin_addr) != 1) {
		closeTcp();
		return false;
	}

	if (::connect(m_tcp_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		closeTcp();
		return false;
	}

	const int flags = fcntl(m_tcp_fd, F_GETFL, 0);
	if (flags >= 0) {
		(void)fcntl(m_tcp_fd, F_SETFL, flags | O_NONBLOCK);
	}

	return true;
}

void AtosTruckSimulator::closeTcp() {
	if (m_tcp_fd >= 0) {
		::shutdown(m_tcp_fd, SHUT_RDWR);
		::close(m_tcp_fd);
		m_tcp_fd = -1;
		m_tcp_rx_buffer.clear();
	}
}

std::string AtosTruckSimulator::utcIso8601FromRosTime(const rclcpp::Time& time) {
	const auto seconds = static_cast<time_t>(time.seconds());
	std::tm tm_utc{};
	gmtime_r(&seconds, &tm_utc);
	std::ostringstream out;
	out << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
	return out.str();
}

bool AtosTruckSimulator::pointAtDistance(double distance_m,
										 double& lat,
										 double& lon,
										 double& course_deg,
										 int& path_index) const {
	if (m_trajectory_path.size() < 2) {
		return false;
	}

	const double max_distance = m_trajectory_path.back().distance_along_path_m;
	double d				  = distance_m;
	if (m_loop_path && max_distance > kEpsilon) {
		d = std::fmod(distance_m, max_distance);
		if (d < 0.0) {
			d += max_distance;
		}
	} else {
		d = std::clamp(d, 0.0, max_distance);
	}

	size_t segment_index = 1;
	while (segment_index < m_trajectory_path.size() && m_trajectory_path[segment_index].distance_along_path_m < d) {
		++segment_index;
	}
	if (segment_index >= m_trajectory_path.size()) {
		segment_index = m_trajectory_path.size() - 1;
	}
	if (segment_index == 0) {
		segment_index = 1;
	}

	const auto& a				= m_trajectory_path[segment_index - 1];
	const auto& b				= m_trajectory_path[segment_index];
	const double segment_length = std::max(kEpsilon, b.distance_along_path_m - a.distance_along_path_m);
	const double t				= std::clamp((d - a.distance_along_path_m) / segment_length, 0.0, 1.0);

	lat = a.lat + (b.lat - a.lat) * t;
	lon = a.lon + (b.lon - a.lon) * t;

	const double y = std::sin(degToRad(b.lon - a.lon)) * std::cos(degToRad(b.lat));
	const double x = std::cos(degToRad(a.lat)) * std::sin(degToRad(b.lat)) -
					 std::sin(degToRad(a.lat)) * std::cos(degToRad(b.lat)) * std::cos(degToRad(b.lon - a.lon));
	course_deg = std::fmod(radToDeg(std::atan2(y, x)) + 360.0, 360.0);

	path_index = static_cast<int>(t < 0.5 ? (segment_index - 1) : segment_index);
	return true;
}

void AtosTruckSimulator::simulationStep() {
	const auto now_time = now();
	const double dt		= std::max(0.001, (now_time - m_last_step_time).seconds());
	m_last_step_time		= now_time;

	const double target_speed_mps = std::max(0.0, m_target_speed_kmh / 3.6);
	const double max_delta		  = m_acceleration_mps2 * dt;
	const double delta			  = std::clamp(target_speed_mps - m_current_speed_mps, -max_delta, max_delta);
	m_current_speed_mps += delta;

	m_current_distance_m += m_current_speed_mps * dt;
	if (!m_loop_path) {
		m_current_distance_m = std::clamp(m_current_distance_m, 0.0, m_trajectory_path.back().distance_along_path_m);
	}

	double lat		  = 0.0;
	double lon		  = 0.0;
	double course_deg = 0.0;
	int path_index	  = 0;
	if (!pointAtDistance(m_current_distance_m, lat, lon, course_deg, path_index)) {
		return;
	}

	if (!ensureTcpConnected()) {
		RCLCPP_WARN_THROTTLE(get_logger(),
							 *get_clock(),
							 2000,
							 "Unable to connect to TruckObjectControl TCP %s:%d",
							 m_tcp_host.c_str(),
							 m_tcp_port);
		return;
	}
	pollTcpCommands();

	const std::string time_str	= utcIso8601FromRosTime(now_time);
	const std::string stale_str = utcIso8601FromRosTime(now_time + rclcpp::Duration::from_seconds(10.0));

	std::ostringstream cot;
	cot << "<event version=\"2.0\" uid=\"" << m_uid << "\" type=\"a-f-G-U-C-I\" time=\"" << time_str << "\" start=\""
		<< time_str << "\" stale=\"" << stale_str << "\" how=\"h-e\"><point lat=\"" << std::setprecision(15) << lat
		<< "\" lon=\"" << lon << "\" speed=\"" << (m_current_speed_mps * 3.6)
		<< "\" hae=\"10\" ce=\"5\" le=\"5\"/><detail><contact callsign=\"" << m_uid
		<< "\"/><__group name=\"Dark Green\" role=\"Test Vehicle\"/><takv device=\"AtosTruckSimulator\" "
		   "platform=\"ATOSFleetManagement\" os=\"Linux\" version=\"1.0\"/><track speed=\""
		<< (m_current_speed_mps * 3.6) << "\" course=\"" << course_deg << "\"/><atos path_name=\""
		<< m_trajectory_path_name << "\" path_index=\"" << path_index << "\"/></detail></event>";

	const std::string payload = cot.str();
	const ssize_t sent		  = ::send(m_tcp_fd, payload.data(), payload.size(), MSG_NOSIGNAL);
	if (sent < 0) {
		RCLCPP_WARN(get_logger(), "TCP send failed, reconnecting...");
		closeTcp();
		return;
	}

	RCLCPP_INFO_THROTTLE(get_logger(),
						 *get_clock(),
						 1000,
						 "Sim uid=%s path=%s idx=%d distance=%.1f m speed=%.1f km/h target=%.1f km/h lat=%.7f lon=%.7f",
						 m_uid.c_str(),
						 m_trajectory_path_name.c_str(),
						 path_index,
						 m_current_distance_m,
						 m_current_speed_mps * 3.6,
						 m_target_speed_kmh,
						 lat,
						 lon);
}

void AtosTruckSimulator::onSpeedCommand(const std_msgs::msg::String::SharedPtr msg) {
	applySpeedCommandPayload(msg->data);
}

void AtosTruckSimulator::applySpeedCommandPayload(const std::string& payload) {
	auto parseField = [&](const std::string& key) -> std::optional<double> {
		const auto pos = payload.find(key);
		if (pos == std::string::npos) {
			return std::nullopt;
		}
		const auto value_start = pos + key.size();
		const auto value_end   = payload.find(';', value_start);
		const auto value	   = payload.substr(value_start, value_end - value_start);
		try {
			return std::stod(value);
		} catch (...) {
			return std::nullopt;
		}
	};

	double commanded_speed_kmh = 0.0;
	if (const auto speed_mps = parseField("target_speed_mps="); speed_mps.has_value()) {
		commanded_speed_kmh = std::max(0.0, *speed_mps) * 3.6;
	} else if (const auto speed_kmh = parseField("target_speed_kmh="); speed_kmh.has_value()) {
		commanded_speed_kmh = std::max(0.0, *speed_kmh);
	} else {
		return;
	}

	if (m_ignore_warning_speed_commands && commanded_speed_kmh > 0.0) {
		return;
	}
	m_target_speed_kmh = commanded_speed_kmh;
}

void AtosTruckSimulator::pollTcpCommands() {
	if (m_tcp_fd < 0) {
		return;
	}

	char rx[1024];
	while (true) {
		const ssize_t n = ::recv(m_tcp_fd, rx, sizeof(rx), 0);
		if (n == 0) {
			closeTcp();
			return;
		}
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				break;
			}
			closeTcp();
			return;
		}

		m_tcp_rx_buffer.append(rx, static_cast<size_t>(n));
		while (true) {
			const auto nl = m_tcp_rx_buffer.find('\n');
			if (nl == std::string::npos) {
				if (m_tcp_rx_buffer.size() > 4096) {
					m_tcp_rx_buffer.clear();
				}
				break;
			}
			const std::string line = m_tcp_rx_buffer.substr(0, nl);
			m_tcp_rx_buffer.erase(0, nl + 1);
			if (!line.empty()) {
				applySpeedCommandPayload(line);
			}
		}
	}
}
