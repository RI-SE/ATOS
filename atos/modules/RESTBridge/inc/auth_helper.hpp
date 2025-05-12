/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <curl/curl.h>
#include <functional>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <time.h>

// Helper struct for CURL write callback
struct WriteCallback {
	std::string data;
	static size_t callback(void* contents, size_t size, size_t nmemb, void* userp) {
		((WriteCallback*)userp)->data.append((char*)contents, size * nmemb);
		return size * nmemb;
	}
};

class AuthHelper {
public:
	AuthHelper(rclcpp::Node* node,
			   CURL* curl_handle,
			   const std::string& auth_url,
			   const std::string& client_id,
			   const std::string& client_secret);

	~AuthHelper();

	// Authenticate and get a new token
	bool authenticate();

	// Check if we have a valid token
	bool hasValidToken() const;

	// Get the current access token
	const std::string& getAccessToken() const;

	// Set up a callback for when the token is refreshed
	void setTokenRefreshCallback(std::function<void()> callback);

private:
	// Helper function for base64 encoding
	std::string base64_encode(const std::string& input);

	// Set up a timer to refresh the token before it expires
	void setupTokenRefreshTimer();

	rclcpp::Node* node_;
	CURL* curl_handle_;
	std::string auth_url_;
	std::string client_id_;
	std::string client_secret_;
	std::string access_token_;

	time_t token_expiry_time_		  = 0;
	const int refresh_buffer_seconds_ = 60; // Refresh token 60 seconds before expiry
	rclcpp::TimerBase::SharedPtr token_refresh_timer_;

	std::function<void()> token_refresh_callback_ = nullptr;
};
