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
		if (!authenticate()) {
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

	if (auth_enabled_ && !access_token_.empty()) {
		std::string auth_header = "Authorization: Bearer " + access_token_;
		default_headers_		= curl_slist_append(default_headers_, auth_header.c_str());
	}
}

// Helper function for base64 encoding
std::string base64_encode(const std::string& input) {
	BIO *bio, *b64;
	BUF_MEM* bufferPtr;

	b64 = BIO_new(BIO_f_base64());
	bio = BIO_new(BIO_s_mem());
	bio = BIO_push(b64, bio);

	BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
	BIO_write(bio, input.c_str(), input.length());
	BIO_flush(bio);
	BIO_get_mem_ptr(bio, &bufferPtr);

	std::string result(bufferPtr->data, bufferPtr->length);
	BIO_free_all(bio);

	return result;
}

// Add a timer to periodically check and refresh the token
void RESTBridge::setupTokenRefreshTimer() {
	// Calculate when to refresh the token (expires_in - buffer)
	int refresh_interval_ms =
	  std::max(1000, static_cast<int>((token_expiry_time_ - std::time(nullptr) - refresh_buffer_seconds_) * 1000));

	RCLCPP_INFO(get_logger(), "Setting up token refresh timer for %d ms from now", refresh_interval_ms);

	// Create a one-shot timer that will refresh the token
	token_refresh_timer_ = create_wall_timer(std::chrono::milliseconds(refresh_interval_ms), [this]() {
		RCLCPP_INFO(get_logger(), "Token refresh timer triggered");
		if (!authenticate()) {
			RCLCPP_ERROR(get_logger(), "Failed to refresh token");
		} else {
			RCLCPP_INFO(get_logger(), "Token refreshed successfully");
			// Set up the next refresh
			setupTokenRefreshTimer();
		}
	});
}

bool RESTBridge::authenticate() {
	if (!curl_handle)
		return false;

	WriteCallback writeCallback;

	// Create base64 encoded credentials for Basic auth
	std::string credentials		   = client_id_ + ":" + client_secret_;
	std::string base64_credentials = base64_encode(credentials);
	std::string auth_header		   = "Authorization: Basic " + base64_credentials;

	// Use form-urlencoded data instead of JSON
	std::string post_data = "grant_type=client_credentials";

	curl_easy_setopt(curl_handle, CURLOPT_URL, auth_url_.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, post_data.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback::callback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &writeCallback);

	// Set the auth headers
	struct curl_slist* auth_headers = nullptr;
	auth_headers = curl_slist_append(auth_headers, "Content-Type: application/x-www-form-urlencoded");
	auth_headers = curl_slist_append(auth_headers, auth_header.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, auth_headers);

	// Perform the authentication request
	CURLcode res = curl_easy_perform(curl_handle);

	curl_slist_free_all(auth_headers);

	if (res != CURLE_OK) {
		RCLCPP_ERROR(get_logger(), "Authentication request failed: %s", curl_easy_strerror(res));
		return false;
	}

	RCLCPP_DEBUG(get_logger(), "Authentication response: %s", writeCallback.data.c_str());

	// Parse the JSON response
	try {
		nlohmann::json auth_response = nlohmann::json::parse(writeCallback.data);

		if (auth_response.contains("access_token")) {
			access_token_ = auth_response["access_token"];

			// Calculate token expiry time
			if (auth_response.contains("expires_in")) {
				int expires_in = std::stoi(auth_response["expires_in"].get<std::string>());
				// Set expiry time to current time + expires_in seconds
				token_expiry_time_ = std::time(nullptr) + expires_in;
				RCLCPP_DEBUG(get_logger(), "Token will expire in %d seconds", expires_in);

				// Set up the refresh timer
				setupTokenRefreshTimer();
			}

			return true;
		} else if (auth_response.contains("error")) {
			RCLCPP_ERROR(
			  get_logger(), "Authentication error: %s - %s", auth_response["error"].get<std::string>().c_str());
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(get_logger(), "Failed to parse authentication response: %s", e.what());
	}

	return false;
}

void RESTBridge::onCustomCommandAction(const atos_interfaces::msg::CustomCommandAction::SharedPtr msg) {
	if (msg->type == atos_interfaces::msg::CustomCommandAction::POST_JSON) {
		RCLCPP_INFO(get_logger(), "Received POST_JSON command: %s", msg->content.c_str());
		json jsonData = parseJsonData(msg->content);
		POST(jsonData["endpoint"].get<std::string>(), jsonData["data"]);
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

void RESTBridge::POST(const std::string& endpoint, const json& data) {
	if (!curl_handle)
		return;

	// If authentication is enabled and we don't have a token, try to authenticate
	if (auth_enabled_ && access_token_.empty()) {
		if (!authenticate()) {
			RCLCPP_ERROR(get_logger(), "Failed to authenticate before POST request");
			return;
		}
	}

	std::string json_str  = data.dump();
	const char* json_data = json_str.c_str();

	curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, json_data);
	curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);

	CURLcode res = curl_easy_perform(curl_handle);

	if (res != CURLE_OK) {
		RCLCPP_ERROR(get_logger(), "POST request failed: %s", curl_easy_strerror(res));

		// If we get an unauthorized error, try to refresh the token and retry
		long http_code;
		curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);

		if (http_code == 401 && auth_enabled_) {
			RCLCPP_INFO(get_logger(), "Unauthorized error, attempting to refresh token...");
			if (authenticate()) {
				// Retry the request with new token
				curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);
				res = curl_easy_perform(curl_handle);
				if (res != CURLE_OK) {
					RCLCPP_ERROR(get_logger(), "Retry POST request failed: %s", curl_easy_strerror(res));
				}
			}
		}
	}
}
