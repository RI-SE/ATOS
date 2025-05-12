#include "restbridge.hpp"
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

RESTBridge::RESTBridge() :
  Module(moduleName),
  customCommandActionMsgSub(*this, std::bind(&RESTBridge::onCustomCommandAction, this, _1)),
  default_headers_(nullptr) {

	// Initialize CURL
	curl_global_init(CURL_GLOBAL_ALL);
	curl_handle = curl_easy_init();

	// Get authentication parameters from ROS
	auth_enabled_  = declare_parameter("auth.enabled", false);
	auth_url_	   = declare_parameter("auth.url", "");
	client_id_	   = declare_parameter("auth.client_id", "");
	client_secret_ = declare_parameter("auth.client_secret", "");

	if (auth_enabled_) {
		// Initialize the auth helper
		auth_helper_ = std::make_unique<AuthHelper>(this, curl_handle, auth_url_, client_id_, client_secret_);

		// Set up a callback for when the token is refreshed
		auth_helper_->setTokenRefreshCallback([this]() {
			// Update the auth header in default_headers_
			updateAuthHeader();
		});

		if (!auth_helper_->authenticate()) {
			RCLCPP_ERROR(get_logger(), "Initial authentication failed!");
		}
	}

	setupCurlHandle();
}

RESTBridge::~RESTBridge() {
	if (default_headers_) {
		curl_slist_free_all(default_headers_);
	}
	curl_easy_cleanup(curl_handle);
	curl_global_cleanup();
}

void RESTBridge::setupCurlHandle() {
	if (!curl_handle)
		return;

	// Setup default headers
	default_headers_ = curl_slist_append(nullptr, "Accept: application/json");
	default_headers_ = curl_slist_append(default_headers_, "Content-Type: application/json");
	default_headers_ = curl_slist_append(default_headers_, "charset: utf-8");

	if (auth_enabled_ && auth_helper_ && !auth_helper_->getAccessToken().empty()) {
		std::string auth_header = "Authorization: Bearer " + auth_helper_->getAccessToken();
		default_headers_		= curl_slist_append(default_headers_, auth_header.c_str());
	}
}

void RESTBridge::updateAuthHeader() {
	if (!auth_enabled_ || !auth_helper_)
		return;

	// Free the old headers
	if (default_headers_) {
		curl_slist_free_all(default_headers_);
		default_headers_ = nullptr;
	}

	// Recreate headers with the new token
	setupCurlHandle();
}

void RESTBridge::onCustomCommandAction(const atos_interfaces::msg::CustomCommandAction::SharedPtr msg) {
	// POST
	if (msg->type == atos_interfaces::msg::CustomCommandAction::POST) {
		RCLCPP_INFO(get_logger(), "Received POST custom command action");
		RCLCPP_INFO(get_logger(), "Content: %s", msg->content.c_str());
		json jsonData = parseJsonData(msg->content);
		POST(jsonData["endpoint"].get<std::string>(), jsonData["data"], jsonData["session_handle"].get<std::string>());
	}
	// DELETE
	if (msg->type == atos_interfaces::msg::CustomCommandAction::DELETE) {
		RCLCPP_INFO(get_logger(), "Received DELETE custom command action");
		RCLCPP_INFO(get_logger(), "Content: %s", msg->content.c_str());
		json jsonData = parseJsonData(msg->content);
		DELETE(jsonData["endpoint"].get<std::string>(), jsonData["session_handle"].get<std::string>());
	}
}

json RESTBridge::parseJsonData(std::string& msg) {
	// Parse the message and return the REST API message
	std::replace(msg.begin(),
				 msg.end(),
				 '\'',
				 '\"'); // Replace single quotes with double quotes to be able to
						// parse the message
	json j = json::parse(msg);
	return j;
}

void RESTBridge::POST(const std::string& endpoint, const json& data, const session_handle& session_handle) {
	if (!curl_handle)
		return;

	// If authentication is enabled and we don't have a valid token, try to authenticate
	if (auth_enabled_ && auth_helper_ && !auth_helper_->hasValidToken()) {
		if (!auth_helper_->authenticate()) {
			RCLCPP_ERROR(get_logger(), "Failed to authenticate before POST request");
			return;
		}
		updateAuthHeader();
	}

	std::string json_str  = data.dump();
	const char* json_data = json_str.c_str();

	curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, json_data);
	curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);

	// Make sure we're using POST method (not a leftover DELETE or other custom request)
	curl_easy_setopt(curl_handle, CURLOPT_CUSTOMREQUEST, nullptr);
	curl_easy_setopt(curl_handle, CURLOPT_POST, 1L);

	// Callback function to handle the response
	WriteCallback writeCallback;

	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback::callback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &writeCallback);

	CURLcode res   = curl_easy_perform(curl_handle);
	long http_code = 0;
	curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);

	if (res != CURLE_OK) {
		RCLCPP_ERROR(get_logger(), "POST request failed: %s", curl_easy_strerror(res));

		// If we get an unauthorized error, try to refresh the token and retry
		if (http_code == 401 && auth_enabled_ && auth_helper_) {
			RCLCPP_INFO(get_logger(), "Unauthorized error, attempting to refresh token...");
			if (auth_helper_->authenticate()) {
				// Update headers with new token
				updateAuthHeader();

				// Retry the request with new token
				curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);
				res = curl_easy_perform(curl_handle);
				curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);
				if (res != CURLE_OK) {
					RCLCPP_ERROR(get_logger(), "Retry POST request failed: %s", curl_easy_strerror(res));
				}
			}
		}
	}

	// Print the response
	RCLCPP_DEBUG(get_logger(), "Response: %s", writeCallback.data.c_str());

	// Only try to parse the response if we got a successful status code
	if ((http_code >= 200 && http_code < 300) && !writeCallback.data.empty()) {
		try {
			// Add the session id to the map from sessionId in the json response data if it exists
			json response = parseJsonData(writeCallback.data);
			if (response.contains("sessionId")) {
				session_ids[session_handle] = response["sessionId"].get<std::string>();
				RCLCPP_INFO(get_logger(), "Stored session ID for handle %s", session_handle.c_str());
			}
		} catch (const std::exception& e) {
			RCLCPP_ERROR(get_logger(), "Failed to parse response JSON: %s", e.what());
		}
	}

	// Reset POST flag for subsequent requests
	curl_easy_setopt(curl_handle, CURLOPT_POST, 0L);
}

void RESTBridge::DELETE(const std::string& endpoint, const session_handle& session_handle) {
	if (session_ids.find(session_handle) == session_ids.end()) {
		RCLCPP_ERROR(get_logger(), "Session handle not found in map");
		return;
	}
	std::string session_id				 = session_ids[session_handle];
	std::string endpoint_with_session_id = endpoint + "/" + session_id;

	// If authentication is enabled and we don't have a valid token, try to authenticate
	if (auth_enabled_ && auth_helper_ && !auth_helper_->hasValidToken()) {
		if (!auth_helper_->authenticate()) {
			RCLCPP_ERROR(get_logger(), "Failed to authenticate before DELETE request");
			return;
		}
		updateAuthHeader();
	}

	WriteCallback writeCallback;

	curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint_with_session_id.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_CUSTOMREQUEST, "DELETE");
	curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);

	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback::callback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &writeCallback);

	CURLcode res   = curl_easy_perform(curl_handle);
	long http_code = 0;
	curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);

	if (res != CURLE_OK) {
		RCLCPP_ERROR(get_logger(), "DELETE request failed: %s", curl_easy_strerror(res));

		// If we get an unauthorized error, try to refresh the token and retry
		if (http_code == 401 && auth_enabled_ && auth_helper_) {
			RCLCPP_INFO(get_logger(), "Unauthorized error, attempting to refresh token...");
			if (auth_helper_->authenticate()) {
				// Update headers with new token
				updateAuthHeader();

				// Retry the request with new token
				curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);
				res = curl_easy_perform(curl_handle);
				curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);
				if (res != CURLE_OK) {
					RCLCPP_ERROR(get_logger(), "Retry DELETE request failed: %s", curl_easy_strerror(res));
				}
			}
		}
	} else {
		// Remove the session ID from the map on successful deletion
		session_ids.erase(session_handle);
		RCLCPP_INFO(get_logger(), "Successfully deleted session %s", session_id.c_str());
	}

	// Reset the custom request method
	curl_easy_setopt(curl_handle, CURLOPT_CUSTOMREQUEST, nullptr);

	RCLCPP_DEBUG(get_logger(), "Response: %s", writeCallback.data.c_str());
}
