#include "restbridge.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

RESTBridge::RESTBridge()
    : Module(moduleName),
      customCommandActionMsgSub(
          *this, std::bind(&RESTBridge::onCustomCommandAction, this, _1)),
      default_headers_(nullptr) {

  // Initialize CURL
  curl_global_init(CURL_GLOBAL_ALL);
  curl_handle = curl_easy_init();

  // Get authentication parameters from ROS
  auth_enabled_ = declare_parameter("auth.enabled", false);
  auth_url_ = declare_parameter("auth.url", "");
  client_id_ = declare_parameter("auth.client_id", "");
  client_secret_ = declare_parameter("auth.client_secret", "");

  if (auth_enabled_) {
    RCLCPP_INFO(get_logger(),
                "Authentication enabled, attempting to authenticate...");
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
  default_headers_ =
      curl_slist_append(default_headers_, "Content-Type: application/json");
  default_headers_ = curl_slist_append(default_headers_, "charset: utf-8");

  if (auth_enabled_ && !access_token_.empty()) {
    std::string auth_header = "Authorization: Bearer " + access_token_;
    default_headers_ = curl_slist_append(default_headers_, auth_header.c_str());
  }
}

bool RESTBridge::authenticate() {
  if (!curl_handle)
    return false;

  WriteCallback writeCallback;

  // Prepare authentication data
  json auth_data = {{"grant_type", "client_credentials"},
                    {"client_id", client_id_},
                    {"client_secret", client_secret_}};
  std::string auth_str = auth_data.dump();

  curl_easy_setopt(curl_handle, CURLOPT_URL, auth_url_.c_str());
  curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, auth_str.c_str());
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteCallback::callback);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &writeCallback);

  // Set basic headers for auth request
  struct curl_slist *auth_headers = nullptr;
  auth_headers =
      curl_slist_append(auth_headers, "Content-Type: application/json");
  curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, auth_headers);

  CURLcode res = curl_easy_perform(curl_handle);

  curl_slist_free_all(auth_headers);

  if (res != CURLE_OK) {
    RCLCPP_ERROR(get_logger(), "Authentication request failed: %s",
                 curl_easy_strerror(res));
    return false;
  }

  try {
    json response = json::parse(writeCallback.data);
    access_token_ = response["access_token"].get<std::string>();

    // Update headers with new token
    if (default_headers_) {
      curl_slist_free_all(default_headers_);
    }
    setupCurlHandle();

    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse authentication response: %s",
                 e.what());
    return false;
  }
}

void RESTBridge::onCustomCommandAction(
    const atos_interfaces::msg::CustomCommandAction::SharedPtr msg) {
  if (msg->type == atos_interfaces::msg::CustomCommandAction::POST_JSON) {
    RCLCPP_INFO(get_logger(), "Received POST_JSON command: %s",
                msg->content.c_str());
    json jsonData = parseJsonData(msg->content);
    POST(jsonData["endpoint"].get<std::string>(), jsonData["data"]);
  }
}

json RESTBridge::parseJsonData(std::string &msg) {
  // Parse the message and return the REST API message
  std::replace(msg.begin(), msg.end(), '\'',
               '\"'); // Replace single quotes with double quotes to be able to
                      // parse the message
  json j = json::parse(msg);
  return j;
}

void RESTBridge::POST(const std::string &endpoint, const json &data) {
  if (!curl_handle)
    return;

  // If authentication is enabled and we don't have a token, try to authenticate
  if (auth_enabled_ && access_token_.empty()) {
    if (!authenticate()) {
      RCLCPP_ERROR(get_logger(), "Failed to authenticate before POST request");
      return;
    }
  }

  std::string json_str = data.dump();
  const char *json_data = json_str.c_str();

  curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint.c_str());
  curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, json_data);
  curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);

  CURLcode res = curl_easy_perform(curl_handle);

  if (res != CURLE_OK) {
    RCLCPP_ERROR(get_logger(), "POST request failed: %s",
                 curl_easy_strerror(res));

    // If we get an unauthorized error, try to refresh the token and retry
    long http_code;
    curl_easy_getinfo(curl_handle, CURLINFO_RESPONSE_CODE, &http_code);

    if (http_code == 401 && auth_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "Unauthorized error, attempting to refresh token...");
      if (authenticate()) {
        // Retry the request with new token
        curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, default_headers_);
        res = curl_easy_perform(curl_handle);
        if (res != CURLE_OK) {
          RCLCPP_ERROR(get_logger(), "Retry POST request failed: %s",
                       curl_easy_strerror(res));
        }
      }
    }
  }
}
