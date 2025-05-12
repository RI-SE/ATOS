/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "auth_helper.hpp"
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>

using namespace std::chrono_literals;

AuthHelper::AuthHelper(rclcpp::Node* node,
					   CURL* curl_handle,
					   const std::string& auth_url,
					   const std::string& client_id,
					   const std::string& client_secret) :
  node_(node),
  curl_handle_(curl_handle),
  auth_url_(auth_url),
  client_id_(client_id),
  client_secret_(client_secret) {}

AuthHelper::~AuthHelper() {
	// Nothing to clean up here as we don't own curl_handle_
}

// Helper function for base64 encoding
std::string AuthHelper::base64_encode(const std::string& input) {
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
void AuthHelper::setupTokenRefreshTimer() {
	// Calculate when to refresh the token (expires_in - buffer)
	int refresh_interval_ms =
	  std::max(1000, static_cast<int>((token_expiry_time_ - std::time(nullptr) - refresh_buffer_seconds_) * 1000));

	RCLCPP_INFO(node_->get_logger(), "Setting up token refresh timer for %d ms from now", refresh_interval_ms);

	// Create a one-shot timer that will refresh the token
	token_refresh_timer_ = node_->create_wall_timer(std::chrono::milliseconds(refresh_interval_ms), [this]() {
		RCLCPP_INFO(node_->get_logger(), "Token refresh timer triggered");
		if (!authenticate()) {
			RCLCPP_ERROR(node_->get_logger(), "Failed to refresh token");
		} else {
			RCLCPP_INFO(node_->get_logger(), "Token refreshed successfully");
			// Set up the next refresh
			setupTokenRefreshTimer();

			// Call the token refresh callback if set
			if (token_refresh_callback_) {
				token_refresh_callback_();
			}
		}
	});
}

bool AuthHelper::authenticate() {
	if (!curl_handle_)
		return false;

	WriteCallback writeCallback;

	// Create base64 encoded credentials for Basic auth
	std::string credentials		   = client_id_ + ":" + client_secret_;
	std::string base64_credentials = base64_encode(credentials);
	std::string auth_header		   = "Authorization: Basic " + base64_credentials;

	// Use form-urlencoded data instead of JSON
	std::string post_data = "grant_type=client_credentials";

	curl_easy_setopt(curl_handle_, CURLOPT_URL, auth_url_.c_str());
	curl_easy_setopt(curl_handle_, CURLOPT_POSTFIELDS, post_data.c_str());
	curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, WriteCallback::callback);
	curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, &writeCallback);

	// Set the auth headers
	struct curl_slist* auth_headers = nullptr;
	auth_headers = curl_slist_append(auth_headers, "Content-Type: application/x-www-form-urlencoded");
	auth_headers = curl_slist_append(auth_headers, auth_header.c_str());
	curl_easy_setopt(curl_handle_, CURLOPT_HTTPHEADER, auth_headers);

	// Perform the authentication request
	CURLcode res = curl_easy_perform(curl_handle_);

	curl_slist_free_all(auth_headers);

	if (res != CURLE_OK) {
		RCLCPP_ERROR(node_->get_logger(), "Authentication request failed: %s", curl_easy_strerror(res));
		return false;
	}

	RCLCPP_DEBUG(node_->get_logger(), "Authentication response: %s", writeCallback.data.c_str());

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
				RCLCPP_DEBUG(node_->get_logger(), "Token will expire in %d seconds", expires_in);

				// Set up the refresh timer
				setupTokenRefreshTimer();
			}

			return true;
		} else if (auth_response.contains("error")) {
			RCLCPP_ERROR(node_->get_logger(),
						 "Authentication error: %s - %s",
						 auth_response["error"].get<std::string>().c_str(),
						 auth_response.contains("error_description")
						   ? auth_response["error_description"].get<std::string>().c_str()
						   : "No description");
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to parse authentication response: %s", e.what());
	}

	return false;
}

bool AuthHelper::hasValidToken() const {
	return !access_token_.empty() && (std::time(nullptr) < token_expiry_time_ - refresh_buffer_seconds_);
}

const std::string& AuthHelper::getAccessToken() const {
	return access_token_;
}

void AuthHelper::setTokenRefreshCallback(std::function<void()> callback) {
	token_refresh_callback_ = callback;
}
