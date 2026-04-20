#pragma once

#include <algorithm>
#include <memory>
#include <regex>
#include <string>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/empty.hpp>

class Mqtt2RosUtils {

public:
	Mqtt2RosUtils() = delete;

	static sensor_msgs::msg::NavSatFix mqtt2navsatfix(const std::string& payload) {
		std::string payload_lowercase = payload;
		std::transform(payload_lowercase.begin(), payload_lowercase.end(), payload_lowercase.begin(), ::tolower);

		auto msg		 = sensor_msgs::msg::NavSatFix();
		msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
		msg.latitude	 = extract_value_from_payload(payload_lowercase, {"lat", "latitude"});
		msg.longitude	 = extract_value_from_payload(payload_lowercase, {"lon", "lng", "long", "longitude"});
		msg.altitude	 = extract_value_from_payload(payload_lowercase, {"alt", "altitude"});

		return msg;
	}

	static rclcpp::SerializedMessage get_serialized_msg(const std::string& msg_type, const std::string& payload) {
		if (msg_type == "std_msgs/msg/Empty") {
			auto msg		= std_msgs::msg::Empty();
			auto serialized = rclcpp::SerializedMessage();
			rclcpp::Serialization<std_msgs::msg::Empty> serializer;
			serializer.serialize_message(&msg, &serialized);
			return serialized;

		} else if (msg_type == "sensor_msgs/msg/NavSatFix") {
			auto msg		= mqtt2navsatfix(payload);
			auto serialized = rclcpp::SerializedMessage();
			rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
			serializer.serialize_message(&msg, &serialized);
			return serialized;
		}

		return rclcpp::SerializedMessage();
	}

private:
	static double extract_value_from_payload(const std::string& input, std::vector<std::string> keys) {
		// Lowercase input
		std::string str = input;
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);

		// Sort keys by length to avoid wrong match
		std::sort(
		  keys.begin(), keys.end(), [](const std::string& a, const std::string& b) { return a.size() > b.size(); });

		// Build regex so we can match any of the keys (key1|key2|...)
		std::string key_pattern;
		for (size_t i{0}; i < keys.size(); ++i) {
			key_pattern += keys.at(i);
			if (i != keys.size() - 1)
				key_pattern += "|";
		}

		// Extract numeric value for any matching key (supports JSON or key: value formats)
		std::regex re{"(?:^|[\\s,{])\"?(" + key_pattern + ")\"?\\s*[:=]\\s*([+-]?\\d+(\\.\\d+)?([eE][+-]?\\d+)?)"};

		std::smatch match;
		double value{0.0};

		// Iterate through all matches, keep the last one in case inputs are duplicated
		auto begin{str.cbegin()};
		auto end{str.cend()};
		while (std::regex_search(begin, end, match, re)) {
			try {
				value = std::stod(match[2].str());
			} catch (...) {
				value = 0.0;
			}
			begin = match.suffix().first;
		}

		return value;
	}
};
