/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include "auth_helper.hpp"
#include "module.hpp"
#include "roschannels/customcommandaction.hpp"
#include <curl/curl.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using json			 = nlohmann::json;
using session_handle = std::string;
using session_id	 = std::string;

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
	std::unique_ptr<AuthHelper> auth_helper_;
	std::map<session_handle, session_id> session_ids = {};

	json parseJsonData(std::string& msg);
	void POST(const std::string& endpoint, const json& data, const session_handle& session_handle);
	void DELETE(const std::string& endpoint, const session_handle& session_handle);
	void setupCurlHandle();
	void updateAuthHeader();

	CURL* curl_handle;
	struct curl_slist* default_headers_;
};
