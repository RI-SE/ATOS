/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include "module.hpp"
#include "roschannels/customcommandaction.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <time.h>

using json = nlohmann::json;
using session_handle = std::string;
using session_id = std::string;

// Helper struct for CURL write callback
struct WriteCallback {
	std::string data;
	static size_t callback(void* contents, size_t size, size_t nmemb, void* userp) {
		((WriteCallback*)userp)->data.append((char*)contents, size * nmemb);
		return size * nmemb;
	}
};


class RESTBridge : public Module {
public:
	static inline std::string const moduleName = "rest_bridge";
	RESTBridge();
	~RESTBridge();

protected:
	void onCustomCommandAction(const atos_interfaces::msg::CustomCommandAction::SharedPtr msg);

private:
	ROSChannels::CustomCommandAction::Sub customCommandActionMsgSub;

	// Auth related members
	bool auth_enabled_;
	std::string auth_url_;
	std::string client_id_;
	std::string client_secret_;
	std::string access_token_;
	std::map<session_handle, session_id> session_ids = {};

	json parseJsonData(std::string& msg);
	void POST(const std::string& endpoint, const json& data, const session_handle& session_handle);
	void DELETE(const std::string& endpoint, const session_handle& session_handle);
	bool authenticate();
	bool refreshToken();
	void setupCurlHandle();

	CURL* curl_handle;
	struct curl_slist* default_headers_;

	time_t token_expiry_time_		  = 0;
	const int refresh_buffer_seconds_ = 60; // Refresh token 60 seconds before expiry
	rclcpp::TimerBase::SharedPtr token_refresh_timer_;

	void setupTokenRefreshTimer();
};
