/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "truckobjectcontrol.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <openssl/err.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <nlohmann/json.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

using std::placeholders::_1;
using json = nlohmann::json;

namespace {
constexpr std::size_t kReceiveBufferSize	   = 4096;
constexpr int kAcceptPollSleepMs			   = 100;
constexpr int kTlsHandshakeRetrySleepMs		   = 10;
constexpr int kTlsReadPollTimeoutMs			   = 1000;
constexpr int kTlsIoRetrySleepMs			   = 10;
constexpr int kTlsWriteMaxTransientRetries	   = 100;
constexpr std::string_view kDefaultGeoJsonName = "RuralRoad_center_of_driving_lane_ccw.geojson";

double degToRad(const double value) {
	return value * M_PI / 180.0;
}

double geodesicDistanceMeters(const double lat1, const double lon1, const double lat2, const double lon2) {
	const double earth_radius_m = 6378137.0;
	const double dlat			= degToRad(lat2 - lat1);
	const double dlon			= degToRad(lon2 - lon1);
	const double a				= std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
					 std::cos(degToRad(lat1)) * std::cos(degToRad(lat2)) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
	const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
	return earth_radius_m * c;
}

struct LocalXY {
	double x = 0.0;
	double y = 0.0;
};

LocalXY toLocalXY(const double ref_lat_deg, const double ref_lon_deg, const double lat_deg, const double lon_deg) {
	constexpr double earth_radius_m = 6378137.0;
	const double ref_lat_rad		= degToRad(ref_lat_deg);
	const double dlat				= degToRad(lat_deg - ref_lat_deg);
	const double dlon				= degToRad(lon_deg - ref_lon_deg);
	LocalXY out;
	out.x = earth_radius_m * dlon * std::cos(ref_lat_rad);
	out.y = earth_radius_m * dlat;
	return out;
}

std::string describeTlsError(const int ssl_error) {
	char ssl_error_text[256]		 = {0};
	const unsigned long queued_error = ERR_get_error();
	if (queued_error != 0) {
		ERR_error_string_n(queued_error, ssl_error_text, sizeof(ssl_error_text));
	}

	std::ostringstream out;
	out << "ssl_error=" << ssl_error;
	if (ssl_error_text[0] != '\0') {
		out << ", openssl='" << ssl_error_text << "'";
	}
	if (errno != 0) {
		out << ", errno=" << errno << " (" << std::strerror(errno) << ")";
	}
	return out.str();
}

std::string resolveDefaultTrajectoryPath(const std::string& configured_path) {
	namespace fs = std::filesystem;
	if (!configured_path.empty() && fs::exists(configured_path)) {
		return configured_path;
	}

	std::vector<fs::path> candidates;
	candidates.emplace_back(fs::current_path() / "conf" / "conf" / kDefaultGeoJsonName);
	candidates.emplace_back(fs::current_path() / ".." / "conf" / "conf" / kDefaultGeoJsonName);

	try {
		const auto prefix = fs::path(ament_index_cpp::get_package_prefix("atos"));
		candidates.emplace_back(prefix / "etc" / "conf" / kDefaultGeoJsonName);
	} catch (...) {}

	for (const auto& candidate : candidates) {
		if (fs::exists(candidate)) {
			return candidate.string();
		}
	}
	return configured_path;
}
} // namespace

TruckObjectControl::TruckObjectControl() :
  Node("truck_object_control") {
	declare_parameter("warning_distance_m", m_warning_distance_m);
	declare_parameter("stop_distance_m", m_stop_distance_m);
	declare_parameter("warning_speed_kmh", m_warning_speed_kmh);
	declare_parameter("stop_speed_kmh", m_stop_speed_kmh);
	declare_parameter("cot_timeout_seconds", m_cot_timeout_seconds);
	declare_parameter("cot_tcp_port", m_cot_tcp_port);
	declare_parameter("cot_tcp_bind_address", m_cot_tcp_bind_address);
	declare_parameter("cot_tls_require_client_cert", m_cot_tls_require_client_cert);
	declare_parameter("cot_tls_cert_path", m_cot_tls_cert_path);
	declare_parameter("cot_tls_key_path", m_cot_tls_key_path);
	declare_parameter("cot_tls_ca_path", m_cot_tls_ca_path);
	declare_parameter("trajectory_geojson_path", m_trajectory_geojson_path);

	m_warning_distance_m		  = get_parameter("warning_distance_m").as_double();
	m_stop_distance_m			  = get_parameter("stop_distance_m").as_double();
	m_warning_speed_kmh			  = get_parameter("warning_speed_kmh").as_double();
	m_stop_speed_kmh			  = get_parameter("stop_speed_kmh").as_double();
	m_cot_timeout_seconds		  = get_parameter("cot_timeout_seconds").as_double();
	m_cot_tcp_port				  = get_parameter("cot_tcp_port").as_int();
	m_cot_tcp_bind_address		  = get_parameter("cot_tcp_bind_address").as_string();
	m_cot_tls_require_client_cert = get_parameter("cot_tls_require_client_cert").as_bool();
	m_cot_tls_cert_path			  = get_parameter("cot_tls_cert_path").as_string();
	m_cot_tls_key_path			  = get_parameter("cot_tls_key_path").as_string();
	m_cot_tls_ca_path			  = get_parameter("cot_tls_ca_path").as_string();
	m_cot_tls_enabled			  = (!m_cot_tls_cert_path.empty() && !m_cot_tls_key_path.empty());
	if (!m_cot_tls_enabled && (!m_cot_tls_cert_path.empty() || !m_cot_tls_key_path.empty())) {
		RCLCPP_WARN(get_logger(),
					"Incomplete TLS config for COT listener (cert or key missing). Falling back to plain TCP.");
	}
	m_trajectory_geojson_path = get_parameter("trajectory_geojson_path").as_string();
	m_trajectory_geojson_path = resolveDefaultTrajectoryPath(m_trajectory_geojson_path);
	m_default_path_name		  = std::filesystem::path(m_trajectory_geojson_path).filename().string();

	m_cot_sub = create_subscription<std_msgs::msg::String>(
	  "truck_objects/cot", 50, std::bind(&TruckObjectControl::onCotMessage, this, _1));

	m_speed_command_pub = create_publisher<std_msgs::msg::String>("truck_objects/speed_command", 20);
	m_truck_state_pub	= create_publisher<std_msgs::msg::String>("truck_objects/state", 50);

	m_evaluation_timer = create_wall_timer(std::chrono::milliseconds(200),
										   std::bind(&TruckObjectControl::evaluateAndPublishSpeedCommand, this));

	if (!loadTrajectoryPath()) {
		RCLCPP_WARN(get_logger(),
					"Failed to load trajectory path from '%s'. Distance along trajectory will stay 0.",
					m_trajectory_geojson_path.c_str());
	} else {
		RCLCPP_INFO(get_logger(),
					"Loaded trajectory path with %zu points from %s",
					m_trajectory_path.size(),
					m_trajectory_geojson_path.c_str());
	}

	startTcpServer();

	RCLCPP_INFO(
	  get_logger(),
	  "TruckObjectControl started. Listening for COT XML on %s://%s:%d and placeholder topic 'truck_objects/cot'.",
	  m_cot_tls_enabled ? "tls" : "tcp",
	  m_cot_tcp_bind_address.c_str(),
	  m_cot_tcp_port);
}

TruckObjectControl::~TruckObjectControl() {
	stopTcpServer();
	if (m_ssl_ctx != nullptr) {
		SSL_CTX_free(m_ssl_ctx);
		m_ssl_ctx = nullptr;
	}
}

void TruckObjectControl::publishTruckState(const std::string& truck_id, const TruckState& state) {
	json payload;
	payload["uid"]				= truck_id;
	payload["distance_m"]		= state.distance_along_trajectory_m;
	payload["distance_to_path_m"] = state.distance_to_path_m;
	payload["lat"]				= state.lat;
	payload["lon"]				= state.lon;
	payload["speed_mps"]		= state.speed_mps;
	payload["speed_kmh"]		= state.speed_mps * 3.6;
	payload["course_deg"]		= state.course_deg;
	payload["tcp_connected"]	= state.tcp_connected;
	payload["path_name"]		= state.path_name;
	payload["path_index"]		= state.path_index;
	payload["last_cot_message"] = state.last_cot_message;
	payload["last_tcp_command"] = state.last_tcp_command;
	payload["last_tcp_warning"] = state.last_tcp_warning;
	payload["stamp_sec"]		= now().seconds();

	std_msgs::msg::String msg;
	msg.data = payload.dump();
	m_truck_state_pub->publish(msg);
}

void TruckObjectControl::onCotMessage(const std_msgs::msg::String::SharedPtr msg) {
	CotObservation observation;
	if (!parseCotPlaceholder(msg->data, observation) && !parseCotXml(msg->data, observation)) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to parse COT payload from ROS topic.");
		return;
	}

	TruckState state;
	{
		std::lock_guard<std::mutex> lock(m_state_mutex);
		auto& entry						  = m_trucks[observation.truck_id];
		entry.distance_along_trajectory_m = observation.distance_along_trajectory_m;
		entry.distance_to_path_m		  = observation.distance_to_path_m;
		entry.lat						  = observation.lat;
		entry.lon						  = observation.lon;
		entry.speed_mps					  = observation.speed_mps;
		entry.course_deg				  = observation.course_deg;
		entry.tcp_connected				  = observation.tcp_connected;
		entry.path_name					  = observation.path_name;
		entry.path_index				  = observation.path_index;
		entry.last_cot_message			  = msg->data;
		entry.last_tcp_warning			  = "";
		entry.last_cot_stamp			  = now();
		state							  = entry;
	}

	publishTruckState(observation.truck_id, state);
}

bool TruckObjectControl::parseCotPlaceholder(const std::string& payload, CotObservation& out) const {
	std::unordered_map<std::string, std::string> fields;
	std::stringstream ss(payload);
	std::string token;

	while (std::getline(ss, token, ';')) {
		const auto sep = token.find('=');
		if (sep == std::string::npos) {
			continue;
		}
		const std::string key	= token.substr(0, sep);
		const std::string value = token.substr(sep + 1);
		fields[key]				= value;
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
		const auto& tcp					= fields.at("tcp_connected");
		out.tcp_connected				= (tcp == "1" || tcp == "true" || tcp == "TRUE");

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

bool TruckObjectControl::parseCotXml(const std::string& payload, CotObservation& out) {
	static const std::regex uid_re(R"re(uid="([^"]+)")re");
	static const std::regex point_re(R"re(<point[^>]*\blat="([^"]+)"[^>]*\blon="([^"]+)")re");
	static const std::regex track_re(R"re(<track[^>]*\bspeed="([^"]+)"[^>]*\bcourse="([^"]+)")re");
	static const std::regex path_name_re(R"re(<atos[^>]*\bpath_name="([^"]+)")re");

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

	out.speed_mps  = 0.0;
	out.course_deg = 0.0;
	if (std::regex_search(payload, m, track_re) && m.size() >= 3) {
		try {
			// Incoming CoT track speed from PathFollower is in km/h; convert to m/s for ATOS internals.
			out.speed_mps  = std::stod(m[1].str()) / 3.6;
			out.course_deg = std::stod(m[2].str());
		} catch (...) {
			out.speed_mps  = 0.0;
			out.course_deg = 0.0;
		}
	}

	out.path_name.clear();
	out.path_index = -1;
	if (std::regex_search(payload, m, path_name_re) && m.size() >= 2) {
		out.path_name = m[1].str();
	}

	const std::vector<GeoPoint>* trajectory = nullptr;
	if (!out.path_name.empty()) {
		trajectory = getTrajectoryForPath(out.path_name);
	}
	if (!trajectory) {
		trajectory = &m_trajectory_path;
		if (out.path_name.empty()) {
			out.path_name = m_default_path_name;
		}
	}

	int projected_path_index = -1;
	double lateral_dist_m	 = 0.0;
	out.distance_along_trajectory_m =
	  projectDistanceAlongTrajectory(out.lat, out.lon, trajectory, &projected_path_index, &lateral_dist_m);
	out.path_index = projected_path_index;
	out.distance_to_path_m = lateral_dist_m;

	out.tcp_connected = true;
	return true;
}

bool TruckObjectControl::isCotFresh(const TruckState& state, const rclcpp::Time& now_time) const {
	const auto age = (now_time - state.last_cot_stamp).seconds();
	return age >= 0.0 && age <= m_cot_timeout_seconds;
}

bool TruckObjectControl::loadTrajectoryPathFromFile(const std::string& path, std::vector<GeoPoint>& out) const {
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
		p.lon		 = coord[0].get<double>();
		p.lat		 = coord[1].get<double>();
		p.distance_m = cumulative;

		if (has_prev) {
			cumulative += geodesicDistanceMeters(prev.lat, prev.lon, p.lat, p.lon);
			p.distance_m = cumulative;
		}

		out.push_back(p);
		prev	 = p;
		has_prev = true;
	}

	return out.size() >= 2;
}

bool TruckObjectControl::loadTrajectoryPath() {
	if (!loadTrajectoryPathFromFile(m_trajectory_geojson_path, m_trajectory_path)) {
		return false;
	}

	std::lock_guard<std::mutex> lock(m_trajectory_cache_mutex);
	m_trajectory_cache[m_default_path_name] = m_trajectory_path;
	return true;
}

std::string TruckObjectControl::resolveTrajectoryPathByName(const std::string& path_name) const {
	namespace fs = std::filesystem;

	if (path_name.empty()) {
		return m_trajectory_geojson_path;
	}

	const fs::path raw(path_name);
	if (raw.is_absolute() && fs::exists(raw)) {
		return raw.string();
	}

	const fs::path base_name = raw.filename();
	if (base_name.empty()) {
		return "";
	}

	const fs::path configured = fs::path(m_trajectory_geojson_path).parent_path() / base_name;
	if (fs::exists(configured)) {
		return configured.string();
	}

	std::vector<fs::path> candidates;
	candidates.emplace_back(fs::current_path() / "conf" / "conf" / base_name);
	candidates.emplace_back(fs::current_path() / ".." / "conf" / "conf" / base_name);

	try {
		const auto prefix = fs::path(ament_index_cpp::get_package_prefix("atos"));
		candidates.emplace_back(prefix / "etc" / "conf" / base_name);
	} catch (...) {}

	for (const auto& candidate : candidates) {
		if (fs::exists(candidate)) {
			return candidate.string();
		}
	}

	return "";
}

const std::vector<TruckObjectControl::GeoPoint>* TruckObjectControl::getTrajectoryForPath(
  const std::string& path_name) {
	const std::string key = std::filesystem::path(path_name).filename().string();
	if (key.empty()) {
		return nullptr;
	}

	{
		std::lock_guard<std::mutex> lock(m_trajectory_cache_mutex);
		const auto it = m_trajectory_cache.find(key);
		if (it != m_trajectory_cache.end()) {
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

	std::lock_guard<std::mutex> lock(m_trajectory_cache_mutex);
	auto [it, inserted] = m_trajectory_cache.emplace(key, std::move(loaded));
	if (inserted) {
		RCLCPP_INFO(get_logger(),
					"Loaded trajectory '%s' with %zu points from %s",
					key.c_str(),
					it->second.size(),
					resolved_path.c_str());
	}
	return &it->second;
}

double TruckObjectControl::projectDistanceAlongTrajectory(const double lat,
														  const double lon,
														  const std::vector<GeoPoint>* trajectory,
														  int* projected_path_index,
														  double* lateral_distance_m) const {
	const std::vector<GeoPoint>* path = trajectory;
	if (!path || path->empty()) {
		path = &m_trajectory_path;
	}
	if (!path || path->empty()) {
		if (projected_path_index != nullptr) {
			*projected_path_index = -1;
		}
		return 0.0;
	}

	// Project onto each polyline segment to get a continuous along-track distance.
	// This avoids quantizing multiple trucks to the same nearest vertex.
	double best_distance_to_path_m = std::numeric_limits<double>::infinity();
	double best_distance_along_m   = 0.0;
	int best_index				   = -1;

	for (size_t i = 0; i + 1 < path->size(); ++i) {
		const auto& a				  = (*path)[i];
		const auto& b				  = (*path)[i + 1];
		const double segment_length_m = std::max(1e-6, b.distance_m - a.distance_m);

		const LocalXY a_xy{0.0, 0.0};
		const LocalXY b_xy = toLocalXY(a.lat, a.lon, b.lat, b.lon);
		const LocalXY q_xy = toLocalXY(a.lat, a.lon, lat, lon);

		const double vx	   = b_xy.x - a_xy.x;
		const double vy	   = b_xy.y - a_xy.y;
		const double wx	   = q_xy.x - a_xy.x;
		const double wy	   = q_xy.y - a_xy.y;
		const double denom = vx * vx + vy * vy;
		double t		   = 0.0;
		if (denom > 1e-12) {
			t = std::clamp((wx * vx + wy * vy) / denom, 0.0, 1.0);
		}

		const double proj_lat = a.lat + (b.lat - a.lat) * t;
		const double proj_lon = a.lon + (b.lon - a.lon) * t;
		const double d		  = geodesicDistanceMeters(lat, lon, proj_lat, proj_lon);

		if (d < best_distance_to_path_m) {
			best_distance_to_path_m = d;
			best_distance_along_m	= a.distance_m + t * segment_length_m;
			best_index				= static_cast<int>(t < 0.5 ? i : (i + 1));
		}
	}

	if (projected_path_index != nullptr) {
		*projected_path_index = best_index;
	}
	if (lateral_distance_m != nullptr) {
		*lateral_distance_m = best_distance_to_path_m;
	}
	return best_distance_along_m;
}

bool TruckObjectControl::initializeTlsContext() {
	if (m_ssl_ctx != nullptr) {
		return true;
	}

	if (m_cot_tls_cert_path.empty() || m_cot_tls_key_path.empty()) {
		RCLCPP_ERROR(get_logger(), "TLS enabled but cot_tls_cert_path or cot_tls_key_path is empty.");
		return false;
	}

	SSL_load_error_strings();
	OpenSSL_add_ssl_algorithms();

	m_ssl_ctx = SSL_CTX_new(TLS_server_method());
	if (m_ssl_ctx == nullptr) {
		RCLCPP_ERROR(get_logger(), "Failed to create TLS server context.");
		return false;
	}

	SSL_CTX_set_min_proto_version(m_ssl_ctx, TLS1_2_VERSION);

	if (SSL_CTX_use_certificate_file(m_ssl_ctx, m_cot_tls_cert_path.c_str(), SSL_FILETYPE_PEM) != 1) {
		RCLCPP_ERROR(get_logger(), "Failed to load TLS certificate from '%s'.", m_cot_tls_cert_path.c_str());
		SSL_CTX_free(m_ssl_ctx);
		m_ssl_ctx = nullptr;
		return false;
	}

	if (SSL_CTX_use_PrivateKey_file(m_ssl_ctx, m_cot_tls_key_path.c_str(), SSL_FILETYPE_PEM) != 1) {
		RCLCPP_ERROR(get_logger(), "Failed to load TLS private key from '%s'.", m_cot_tls_key_path.c_str());
		SSL_CTX_free(m_ssl_ctx);
		m_ssl_ctx = nullptr;
		return false;
	}

	if (SSL_CTX_check_private_key(m_ssl_ctx) != 1) {
		RCLCPP_ERROR(get_logger(), "TLS private key does not match certificate.");
		SSL_CTX_free(m_ssl_ctx);
		m_ssl_ctx = nullptr;
		return false;
	}

	if (!m_cot_tls_ca_path.empty()) {
		if (SSL_CTX_load_verify_locations(m_ssl_ctx, m_cot_tls_ca_path.c_str(), nullptr) != 1) {
			RCLCPP_ERROR(get_logger(), "Failed to load TLS CA file from '%s'.", m_cot_tls_ca_path.c_str());
			SSL_CTX_free(m_ssl_ctx);
			m_ssl_ctx = nullptr;
			return false;
		}
	}

	if (m_cot_tls_require_client_cert) {
		SSL_CTX_set_verify(m_ssl_ctx, SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT, nullptr);
	} else {
		SSL_CTX_set_verify(m_ssl_ctx, SSL_VERIFY_NONE, nullptr);
	}

	return true;
}

void TruckObjectControl::startTcpServer() {
	if (m_cot_tls_enabled && !initializeTlsContext()) {
		RCLCPP_ERROR(get_logger(), "COT listener startup failed: TLS setup failed.");
		return;
	}

	m_tcp_server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
	if (m_tcp_server_fd < 0) {
		RCLCPP_ERROR(get_logger(), "Failed to create TCP socket for COT listener.");
		return;
	}

	int enable = 1;
	(void)setsockopt(m_tcp_server_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
	(void)setsockopt(m_tcp_server_fd, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port	= htons(static_cast<uint16_t>(m_cot_tcp_port));
	if (::inet_pton(AF_INET, m_cot_tcp_bind_address.c_str(), &addr.sin_addr) != 1) {
		RCLCPP_ERROR(get_logger(), "Invalid bind address '%s' for COT TCP listener.", m_cot_tcp_bind_address.c_str());
		::close(m_tcp_server_fd);
		m_tcp_server_fd = -1;
		return;
	}

	if (::bind(m_tcp_server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		RCLCPP_ERROR(
		  get_logger(), "Failed to bind COT TCP listener on %s:%d", m_cot_tcp_bind_address.c_str(), m_cot_tcp_port);
		::close(m_tcp_server_fd);
		m_tcp_server_fd = -1;
		return;
	}

	if (::listen(m_tcp_server_fd, 8) < 0) {
		RCLCPP_ERROR(get_logger(), "Failed to listen on COT TCP listener.");
		::close(m_tcp_server_fd);
		m_tcp_server_fd = -1;
		return;
	}

	m_tcp_running.store(true);
	m_tcp_accept_thread = std::thread(&TruckObjectControl::acceptTcpClients, this);
}

void TruckObjectControl::stopTcpServer() {
	m_tcp_running.store(false);

	if (m_tcp_server_fd >= 0) {
		::shutdown(m_tcp_server_fd, SHUT_RDWR);
		::close(m_tcp_server_fd);
		m_tcp_server_fd = -1;
	}

	if (m_tcp_accept_thread.joinable()) {
		m_tcp_accept_thread.join();
	}

	{
		std::lock_guard<std::mutex> lock(m_tcp_threads_mutex);
		for (const int fd : m_tcp_client_fds) {
			::shutdown(fd, SHUT_RDWR);
			::close(fd);
		}
		m_tcp_client_fds.clear();
	}
	{
		std::lock_guard<std::mutex> lock(m_tcp_command_mutex);
		m_uid_to_client_fd.clear();
		m_uid_to_ssl.clear();
	}
	{
		std::lock_guard<std::mutex> lock(m_tcp_sessions_mutex);
		for (auto& [fd, session] : m_tcp_sessions) {
			{
				std::lock_guard<std::mutex> qlock(session->queue_mutex);
				session->stop_sender = true;
			}
			session->queue_cv.notify_all();
		}
	}

	std::lock_guard<std::mutex> lock(m_tcp_threads_mutex);
	for (auto& thread : m_tcp_client_threads) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	m_tcp_client_threads.clear();
	{
		std::lock_guard<std::mutex> session_lock(m_tcp_sessions_mutex);
		for (auto& [fd, session] : m_tcp_sessions) {
			if (session->sender_thread.joinable()) {
				session->sender_thread.join();
			}
		}
		m_tcp_sessions.clear();
	}
}

void TruckObjectControl::acceptTcpClients() {
	while (m_tcp_running.load()) {
		sockaddr_in client_addr{};
		socklen_t client_len = sizeof(client_addr);
		const int client_fd	 = ::accept(m_tcp_server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
		if (client_fd < 0) {
			if (!m_tcp_running.load()) {
				break;
			}
			RCLCPP_WARN_THROTTLE(
			  get_logger(), *get_clock(), 3000, "TCP accept failed (errno=%d). Listener will continue.", errno);
			std::this_thread::sleep_for(std::chrono::milliseconds(kAcceptPollSleepMs));
			continue;
		}

		int keepalive = 1;
		(void)setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

		int tcp_no_delay = 1;
		if (::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &tcp_no_delay, sizeof(tcp_no_delay)) < 0) {
			RCLCPP_WARN(get_logger(),
						"Failed to enable TCP_NODELAY for client socket (errno=%d). Commands may be delayed.",
						errno);
		}

		const int flags = ::fcntl(client_fd, F_GETFL, 0);
		if (flags >= 0) {
			(void)::fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
		}

		timeval recv_timeout{};
		recv_timeout.tv_sec	 = 1;
		recv_timeout.tv_usec = 0;
		(void)setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof(recv_timeout));

		char ip_buf[INET_ADDRSTRLEN] = {0};
		const char* peer_ip			 = ::inet_ntop(AF_INET, &client_addr.sin_addr, ip_buf, sizeof(ip_buf));
		std::ostringstream peer;
		peer << (peer_ip ? peer_ip : "unknown") << ":" << ntohs(client_addr.sin_port);

		RCLCPP_INFO(get_logger(), "Accepted TruckObject TCP client %s", peer.str().c_str());

		{
			std::lock_guard<std::mutex> lock(m_tcp_sessions_mutex);
			auto session			  = std::make_shared<TcpClientSession>();
			session->fd				  = client_fd;
			session->peer_name		  = peer.str();
			m_tcp_sessions[client_fd] = session;
		}

		std::lock_guard<std::mutex> lock(m_tcp_threads_mutex);
		m_tcp_client_fds.insert(client_fd);
		m_tcp_client_threads.emplace_back(&TruckObjectControl::handleTcpClient, this, client_fd, peer.str());
	}
}

void TruckObjectControl::handleTcpClient(const int client_fd, const std::string& peer_name) {
	std::string buffer;
	buffer.reserve(8 * 1024);
	std::set<std::string> seen_uids;
	SSL* ssl = nullptr;
	std::shared_ptr<TcpClientSession> session;

	{
		std::lock_guard<std::mutex> lock(m_tcp_sessions_mutex);
		const auto it = m_tcp_sessions.find(client_fd);
		if (it != m_tcp_sessions.end()) {
			session = it->second;
		}
	}
	if (!session) {
		RCLCPP_WARN(get_logger(), "Missing TCP client session for %s fd=%d", peer_name.c_str(), client_fd);
		::shutdown(client_fd, SHUT_RDWR);
		::close(client_fd);
		return;
	}

	try {
		if (m_cot_tls_enabled) {
			ssl = SSL_new(m_ssl_ctx);
			if (ssl == nullptr) {
				RCLCPP_WARN(get_logger(), "Failed to create TLS session for %s.", peer_name.c_str());
				throw std::runtime_error("SSL_new failed");
			}

			if (SSL_set_fd(ssl, client_fd) != 1) {
				RCLCPP_WARN(get_logger(), "Failed to bind TLS session to socket for %s.", peer_name.c_str());
				throw std::runtime_error("SSL_set_fd failed");
			}

			SSL_set_mode(ssl, SSL_MODE_ENABLE_PARTIAL_WRITE | SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);

			while (m_tcp_running.load()) {
				const int accept_result = SSL_accept(ssl);
				if (accept_result == 1) {
					break;
				}

				const int ssl_accept_error = SSL_get_error(ssl, accept_result);
				if (ssl_accept_error == SSL_ERROR_WANT_READ || ssl_accept_error == SSL_ERROR_WANT_WRITE) {
					std::this_thread::sleep_for(std::chrono::milliseconds(kTlsHandshakeRetrySleepMs));
					continue;
				}

				char ssl_error[256] = {0};
				ERR_error_string_n(ERR_get_error(), ssl_error, sizeof(ssl_error));
				RCLCPP_WARN(get_logger(),
							"TLS handshake failed for %s: %s (ssl_error=%d)",
							peer_name.c_str(),
							ssl_error[0] == '\0' ? "unknown error" : ssl_error,
							ssl_accept_error);
				throw std::runtime_error("SSL_accept failed");
			}
		}

		session->ssl		   = ssl;
		session->sender_thread = std::thread(&TruckObjectControl::clientSenderLoop, this, session);

		char receive_buffer[kReceiveBufferSize];
		while (m_tcp_running.load()) {
			ssize_t received = 0;
			if (m_cot_tls_enabled) {
				pollfd pfd{};
				pfd.fd				  = client_fd;
				pfd.events			  = POLLIN;
				const int poll_result = ::poll(&pfd, 1, kTlsReadPollTimeoutMs);
				if (poll_result == 0) {
					continue;
				}
				if (poll_result < 0) {
					if (errno == EINTR) {
						continue;
					}
					RCLCPP_WARN(get_logger(),
								"TLS receive poll error from %s (errno=%d: %s). Closing this connection only.",
								peer_name.c_str(),
								errno,
								std::strerror(errno));
					break;
				}
				if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
					RCLCPP_INFO(get_logger(),
								"TLS client %s socket closed or errored (revents=0x%x).",
								peer_name.c_str(),
								pfd.revents);
					break;
				}

				ERR_clear_error();
				errno = 0;
				{
					std::lock_guard<std::mutex> lock(session->io_mutex);
					received = SSL_read(ssl, receive_buffer, static_cast<int>(sizeof(receive_buffer)));
				}
				if (received <= 0) {
					const int ssl_read_error = SSL_get_error(ssl, static_cast<int>(received));
					if (ssl_read_error == SSL_ERROR_WANT_READ || ssl_read_error == SSL_ERROR_WANT_WRITE) {
						std::this_thread::sleep_for(std::chrono::milliseconds(kTlsIoRetrySleepMs));
						continue;
					}
					if (ssl_read_error == SSL_ERROR_ZERO_RETURN) {
						break;
					}
					if (ssl_read_error == SSL_ERROR_SYSCALL &&
						(errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
						continue;
					}
					RCLCPP_WARN(get_logger(),
								"TLS receive error from %s (%s). Closing this connection only.",
								peer_name.c_str(),
								describeTlsError(ssl_read_error).c_str());
					break;
				}
			} else {
				received = ::recv(client_fd, receive_buffer, sizeof(receive_buffer), 0);
				if (received == 0) {
					break;
				}
				if (received < 0) {
					if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
						continue;
					}
					RCLCPP_WARN(get_logger(),
								"TCP receive error from %s (errno=%d). Closing this connection only.",
								peer_name.c_str(),
								errno);
					break;
				}
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
				const std::string xml  = buffer.substr(start_pos, event_end - start_pos);
				buffer.erase(0, event_end);

				CotObservation observation;
				if (!parseCotXml(xml, observation)) {
					RCLCPP_WARN_THROTTLE(get_logger(),
										 *get_clock(),
										 5000,
										 "Failed to parse COT XML from TCP client %s",
										 peer_name.c_str());
					continue;
				}

				TruckState state;
				{
					std::lock_guard<std::mutex> lock(m_state_mutex);
					auto& entry						  = m_trucks[observation.truck_id];
					entry.distance_along_trajectory_m = observation.distance_along_trajectory_m;
					entry.distance_to_path_m		  = observation.distance_to_path_m;
					entry.lat						  = observation.lat;
					entry.lon						  = observation.lon;
					entry.speed_mps					  = observation.speed_mps;
					entry.course_deg				  = observation.course_deg;
					entry.tcp_connected				  = true;
					entry.path_name					  = observation.path_name;
					entry.path_index				  = observation.path_index;
					entry.last_cot_message			  = xml;
					entry.last_tcp_warning			  = "";
					entry.last_cot_stamp			  = now();
					state							  = entry;
				}
				{
					std::lock_guard<std::mutex> lock(m_tcp_command_mutex);
					const auto existing = m_uid_to_client_fd.find(observation.truck_id);
					if (existing != m_uid_to_client_fd.end() && existing->second != client_fd) {
						RCLCPP_WARN(get_logger(),
									"UID '%s' already mapped to fd=%d; remapping to fd=%d from %s. "
									"Ensure each sender uses a unique uid.",
									observation.truck_id.c_str(),
									existing->second,
									client_fd,
									peer_name.c_str());
					}
					m_uid_to_client_fd[observation.truck_id] = client_fd;
					if (m_cot_tls_enabled) {
						m_uid_to_ssl[observation.truck_id] = ssl;
					}
				}

				seen_uids.insert(observation.truck_id);
				publishTruckState(observation.truck_id, state);
			}
		}
	} catch (...) {
		RCLCPP_WARN(
		  get_logger(), "Unexpected exception while handling client %s. Connection will be closed.", peer_name.c_str());
	}

	disconnectClientSession(session, "TCP client disconnected");
}

void TruckObjectControl::evaluateAndPublishSpeedCommand() {
	constexpr double kMaxDistanceToPathM = 7.5;

	struct ConnectedTruck {
		std::string id;
		std::string path_name;
		double distance_m = 0.0;
		double speed_mps  = 0.0;
		double distance_to_path_m = 0.0;
		int path_index	  = -1;
		std::string previous_command;
	};
	std::vector<ConnectedTruck> connected;

	{
		std::lock_guard<std::mutex> lock(m_state_mutex);
		for (const auto& [id, state] : m_trucks) {
			if (!state.tcp_connected) {
				RCLCPP_WARN_THROTTLE(get_logger(),
									 *get_clock(),
									 3000,
									 "Skipping uid=%s from command eval: tcp_connected=false, path=%s, distance=%.1fm",
									 id.c_str(),
									 state.path_name.c_str(),
									 state.distance_along_trajectory_m);
				continue;
			}
			if (state.distance_to_path_m > kMaxDistanceToPathM) {
				RCLCPP_WARN_THROTTLE(get_logger(),
									 *get_clock(),
									 3000,
									 "Skipping uid=%s from command eval: off-path by %.1f m (max %.1f m)",
									 id.c_str(),
									 state.distance_to_path_m,
									 kMaxDistanceToPathM);
				continue;
			}
			connected.push_back(ConnectedTruck{id,
											   state.path_name,
											   state.distance_along_trajectory_m,
											   state.speed_mps,
											   state.distance_to_path_m,
											   state.path_index,
											   state.last_control_command});
		}
	}

	if (connected.empty()) {
		RCLCPP_WARN_THROTTLE(
		  get_logger(), *get_clock(), 3000, "No TCP-connected trucks available for command evaluation.");
		return;
	}

	std::unordered_map<std::string, std::vector<ConnectedTruck>> by_path;
	for (const auto& truck : connected) {
		by_path[truck.path_name].push_back(truck);
	}

	{
		std::ostringstream summary;
		summary << "Connected trucks=" << connected.size() << " [";
		for (size_t i = 0; i < connected.size(); ++i) {
			const auto& t = connected[i];
			if (i > 0) {
				summary << " | ";
			}
			summary << "uid=" << t.id << ",path=" << t.path_name << ",d=" << std::fixed << std::setprecision(1)
					<< t.distance_m;
		}
		summary << "]";
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "%s", summary.str().c_str());
	}

	double min_gap_m	 = std::numeric_limits<double>::infinity();
	bool any_stop		 = false;
	bool any_slowdown	 = false;
	size_t sent_commands = 0;

	for (auto& [path_name, group] : by_path) {
		std::sort(group.begin(), group.end(), [](const ConnectedTruck& a, const ConnectedTruck& b) {
			return a.distance_m < b.distance_m;
		});

		double path_length_m					= 0.0;
		const std::vector<GeoPoint>* trajectory = nullptr;
		if (!path_name.empty()) {
			trajectory = getTrajectoryForPath(path_name);
		}
		if ((!trajectory || trajectory->empty()) && !m_trajectory_path.empty()) {
			trajectory = &m_trajectory_path;
		}
		if (trajectory && !trajectory->empty()) {
			path_length_m = trajectory->back().distance_m;
		}

		for (size_t i = 0; i < group.size(); ++i) {
			const bool has_peers  = group.size() > 1;
			bool has_ahead		  = has_peers;
			std::string ahead_uid = "none";
			double ahead_gap	  = -1.0;
			int ahead_path_index  = -1;

			if (has_peers) {
				const size_t ahead_index  = (i + 1) % group.size();
				ahead_uid				  = group[ahead_index].id;
				ahead_path_index		  = group[ahead_index].path_index;
				const int curr_path_index = group[i].path_index;

				// Prefer index-based circular gap when both indices are valid for this trajectory.
				bool used_index_gap = false;
				if (trajectory && !trajectory->empty() && curr_path_index >= 0 && ahead_path_index >= 0 &&
					static_cast<size_t>(curr_path_index) < trajectory->size() &&
					static_cast<size_t>(ahead_path_index) < trajectory->size() && path_length_m > 0.0) {
					const double curr_d	 = (*trajectory)[static_cast<size_t>(curr_path_index)].distance_m;
					const double ahead_d = (*trajectory)[static_cast<size_t>(ahead_path_index)].distance_m;
					ahead_gap			 = ahead_d - curr_d;
					if (ahead_gap <= 0.0) {
						// Wraparound: distance to end + distance from beginning.
						ahead_gap += path_length_m;
					}
					used_index_gap = true;
				}

				if (!used_index_gap) {
					ahead_gap = group[ahead_index].distance_m - group[i].distance_m;
					if (ahead_gap <= 0.0 && path_length_m > 0.0) {
						ahead_gap += path_length_m;
					}
				}
				if (ahead_gap <= 0.0) {
					has_ahead		 = false;
					ahead_uid		 = "none";
					ahead_path_index = -1;
					ahead_gap		 = -1.0;
				}
			}

			if (has_ahead) {
				min_gap_m = std::min(min_gap_m, ahead_gap);
			}

			const std::string previous_command = group[i].previous_command;
			std::string control_command		   = previous_command.empty() ? "DRIVE" : previous_command;

			const bool in_warning_band = has_ahead && (ahead_gap <= 500.0 && ahead_gap > 200.0);
			const bool in_stop_band	   = has_ahead && (ahead_gap <= 200.0);
			if (in_warning_band) {
				control_command = "SLOWDOWN";
			}
			if (in_stop_band) {
				control_command = "STOP";
			}
			if (has_ahead && ahead_gap > 300.0 && previous_command == "STOP") {
				control_command = "SLOWDOWN";
			}
			if (has_ahead && ahead_gap > 700.0 && previous_command == "SLOWDOWN") {
				control_command = "DRIVE";
			}

			std::string target_speed_mps_value = std::to_string(std::max(0.0, group[i].speed_mps));
			if (control_command == "STOP") {
				target_speed_mps_value = std::to_string(m_stop_speed_kmh / 3.6);
				any_stop			   = true;
			} else if (control_command == "SLOWDOWN") {
				target_speed_mps_value = std::to_string(m_warning_speed_kmh / 3.6);
				any_slowdown		   = true;
			}

			{
				std::lock_guard<std::mutex> lock(m_state_mutex);
				auto it = m_trucks.find(group[i].id);
				if (it != m_trucks.end()) {
					it->second.last_control_command = control_command;
				}
			}

			const std::string reason =
			  (control_command == "STOP")
				? "truck_ahead_below_stop_distance"
				: (control_command == "SLOWDOWN" ? "truck_ahead_slowdown_state" : "truck_ahead_drive_state");
			uint64_t cmd_seq = 0;
			{
				std::lock_guard<std::mutex> lock(m_state_mutex);
				cmd_seq = ++m_tcp_command_seq;
			}
			const std::string tcp_command =
			  "command=" + control_command + ";target_speed_mps=" + target_speed_mps_value +
			  ";distance_to_truck_ahead_m=" + std::to_string(ahead_gap) +
			  ";truck_ahead_path_index=" + std::to_string(ahead_path_index) + ";truck_ahead_uid=" + ahead_uid +
			  ";reason=" + reason + ";min_gap_m=" + (std::isfinite(min_gap_m) ? std::to_string(min_gap_m) : "-1") +
			  ";connected_count=" + std::to_string(connected.size()) + ";path_name=" + path_name +
			  ";cmd_seq=" + std::to_string(cmd_seq);

			RCLCPP_INFO(get_logger(), "Prepared TCP command for uid=%s: %s", group[i].id.c_str(), tcp_command.c_str());

			// Keep UI state in sync with latest generated command, even if TCP/TLS send fails.
			TruckState state_copy;
			bool has_state = false;
			{
				std::lock_guard<std::mutex> lock(m_state_mutex);
				auto it = m_trucks.find(group[i].id);
				if (it != m_trucks.end()) {
					it->second.last_tcp_command = tcp_command;
					state_copy					= it->second;
					has_state					= true;
				}
			}
			if (has_state) {
				publishTruckState(group[i].id, state_copy);
			}

			sendSpeedCommandToTcpClient(group[i].id, tcp_command);
			sent_commands += 1;
		}
	}

	if (!std::isfinite(min_gap_m)) {
		min_gap_m = -1.0;
	}

	std::string fleet_target_speed_mps_value = "nochange";
	std::string fleet_reason				 = "no_limit";
	if (any_stop) {
		fleet_target_speed_mps_value = std::to_string(m_stop_speed_kmh / 3.6);
		fleet_reason				 = "min_gap_below_stop_distance";
	} else if (any_slowdown) {
		fleet_target_speed_mps_value = std::to_string(m_warning_speed_kmh / 3.6);
		fleet_reason				 = "min_gap_below_warning_distance";
	}

	std_msgs::msg::String command;
	command.data = "target_speed_mps=" + fleet_target_speed_mps_value + ";scope=all_connected_with_valid_tcp" +
				   ";reason=" + fleet_reason + ";min_gap_m=" + std::to_string(min_gap_m) +
				   ";connected_count=" + std::to_string(connected.size());
	m_speed_command_pub->publish(command);

	RCLCPP_WARN(
	  get_logger(),
	  "Published fleet speed command target_speed_mps=%s (reason=%s, min_gap=%.2f m, connected=%zu, tcp_cmds=%zu)",
	  fleet_target_speed_mps_value.c_str(),
	  fleet_reason.c_str(),
	  min_gap_m,
	  connected.size(),
	  sent_commands);
}

void TruckObjectControl::updateTruckTcpStatus(const std::string& target_id,
											  const std::string& command,
											  const std::string& warning,
											  bool mark_disconnected) {
	TruckState state_copy;
	bool has_state = false;
	{
		std::lock_guard<std::mutex> lock(m_state_mutex);
		const auto it = m_trucks.find(target_id);
		if (it != m_trucks.end()) {
			it->second.last_tcp_command = command;
			it->second.last_tcp_warning = warning;
			if (mark_disconnected) {
				it->second.tcp_connected  = false;
				it->second.last_cot_stamp = now();
			}
			state_copy = it->second;
			has_state  = true;
		}
	}
	if (has_state) {
		publishTruckState(target_id, state_copy);
	}
}

void TruckObjectControl::clientSenderLoop(const std::shared_ptr<TcpClientSession>& session) {
	while (m_tcp_running.load()) {
		std::string target_id;
		std::string command;
		{
			std::unique_lock<std::mutex> lock(session->queue_mutex);
			session->queue_cv.wait(lock, [&]() { return session->stop_sender || session->has_queued_command; });
			if (session->stop_sender) {
				break;
			}
			target_id					= session->queued_uid;
			command						= session->queued_command;
			session->has_queued_command = false;
		}

		if (target_id.empty() || command.empty()) {
			continue;
		}

		const std::string payload = command + "\n";
		if (m_cot_tls_enabled) {
			if (session->ssl == nullptr) {
				updateTruckTcpStatus(target_id, command, "TLS session missing for this truck", true);
				::shutdown(session->fd, SHUT_RDWR);
				break;
			}

			size_t total_sent	  = 0;
			int transient_retries = 0;
			bool send_failed	  = false;
			std::string send_warning;
			while (total_sent < payload.size() && m_tcp_running.load()) {
				{
					std::lock_guard<std::mutex> lock(session->queue_mutex);
					if (session->stop_sender) {
						send_failed	 = true;
						send_warning = "TLS sender stopped";
						break;
					}
				}

				int sent = 0;
				ERR_clear_error();
				errno = 0;
				{
					std::lock_guard<std::mutex> lock(session->io_mutex);
					sent = SSL_write(
					  session->ssl, payload.data() + total_sent, static_cast<int>(payload.size() - total_sent));
				}

				if (sent > 0) {
					total_sent += static_cast<size_t>(sent);
					transient_retries = 0;
					continue;
				}

				const int ssl_write_error = SSL_get_error(session->ssl, sent);
				const bool transient =
				  ssl_write_error == SSL_ERROR_WANT_READ || ssl_write_error == SSL_ERROR_WANT_WRITE ||
				  (ssl_write_error == SSL_ERROR_SYSCALL && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK));
				if (transient && transient_retries < kTlsWriteMaxTransientRetries) {
					transient_retries += 1;
					std::this_thread::sleep_for(std::chrono::milliseconds(kTlsIoRetrySleepMs));
					continue;
				}

				send_failed	 = true;
				send_warning = "TLS send failed (" + describeTlsError(ssl_write_error) + ")";
				break;
			}

			if (!send_failed && total_sent < payload.size()) {
				updateTruckTcpStatus(target_id, command, "TLS sender stopped before payload was fully sent", true);
				::shutdown(session->fd, SHUT_RDWR);
				break;
			}

			if (send_failed) {
				updateTruckTcpStatus(target_id, command, send_warning, true);
				::shutdown(session->fd, SHUT_RDWR);
				break;
			}

			RCLCPP_INFO(get_logger(),
						"Sent TLS speed command to uid=%s fd=%d: %s",
						target_id.c_str(),
						session->fd,
						command.c_str());
			updateTruckTcpStatus(target_id, command, "");
			continue;
		}

		const ssize_t sent = ::send(session->fd, payload.data(), payload.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
		if (sent < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				updateTruckTcpStatus(target_id, command, "TCP backpressure drop");
				continue;
			}
			updateTruckTcpStatus(target_id, command, "TCP send failed (errno=" + std::to_string(errno) + ")", true);
			::shutdown(session->fd, SHUT_RDWR);
			break;
		}

		if (static_cast<size_t>(sent) < payload.size()) {
			updateTruckTcpStatus(target_id, command, "TCP partial send drop");
			continue;
		}

		RCLCPP_INFO(
		  get_logger(), "Sent TCP speed command to uid=%s fd=%d: %s", target_id.c_str(), session->fd, command.c_str());
		updateTruckTcpStatus(target_id, command, "");
	}
}

void TruckObjectControl::disconnectClientSession(const std::shared_ptr<TcpClientSession>& session,
												 const std::string& reason) {
	if (!session) {
		return;
	}

	{
		std::lock_guard<std::mutex> lock(session->queue_mutex);
		session->stop_sender		= true;
		session->has_queued_command = false;
	}
	session->queue_cv.notify_all();

	std::vector<std::string> affected_uids;
	{
		std::lock_guard<std::mutex> lock(m_tcp_command_mutex);
		for (auto it = m_uid_to_client_fd.begin(); it != m_uid_to_client_fd.end();) {
			if (it->second == session->fd) {
				affected_uids.push_back(it->first);
				m_uid_to_ssl.erase(it->first);
				it = m_uid_to_client_fd.erase(it);
			} else {
				++it;
			}
		}
	}

	for (const auto& uid : affected_uids) {
		updateTruckTcpStatus(uid, "", reason, true);
	}

	const int old_fd = session->fd;
	{
		std::lock_guard<std::mutex> lock(m_tcp_threads_mutex);
		m_tcp_client_fds.erase(old_fd);
	}

	if (session->sender_thread.joinable()) {
		session->sender_thread.join();
	}

	if (session->ssl != nullptr) {
		std::lock_guard<std::mutex> lock(session->io_mutex);
		(void)SSL_shutdown(session->ssl);
		SSL_free(session->ssl);
		session->ssl = nullptr;
	}

	if (old_fd >= 0) {
		::shutdown(old_fd, SHUT_RDWR);
		::close(old_fd);
		session->fd = -1;
	}

	{
		std::lock_guard<std::mutex> lock(m_tcp_sessions_mutex);
		m_tcp_sessions.erase(old_fd);
	}

	RCLCPP_INFO(
	  get_logger(), "TruckObject TCP client disconnected: %s (%s)", session->peer_name.c_str(), reason.c_str());
}

void TruckObjectControl::sendSpeedCommandToTcpClient(const std::string& target_id, const std::string& command) {
	int target_fd = -1;
	{
		std::lock_guard<std::mutex> lock(m_tcp_command_mutex);
		const auto it = m_uid_to_client_fd.find(target_id);
		if (it != m_uid_to_client_fd.end()) {
			target_fd = it->second;
		}
	}

	if (target_fd < 0) {
		updateTruckTcpStatus(target_id, command, "No active TCP socket for this truck");
		return;
	}

	std::shared_ptr<TcpClientSession> session;
	{
		std::lock_guard<std::mutex> lock(m_tcp_sessions_mutex);
		const auto it = m_tcp_sessions.find(target_fd);
		if (it != m_tcp_sessions.end()) {
			session = it->second;
		}
	}

	if (!session) {
		updateTruckTcpStatus(target_id, command, "No active TCP session for this truck");
		return;
	}

	{
		std::lock_guard<std::mutex> lock(session->queue_mutex);
		session->queued_uid			= target_id;
		session->queued_command		= command;
		session->has_queued_command = true;
	}
	session->queue_cv.notify_one();
}
