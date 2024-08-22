#include "restbridge.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

RESTBridge::RESTBridge()
    : Module(moduleName),
      customCommandActionMsgSub(
          *this, std::bind(&RESTBridge::onCustomCommandAction, this, _1)) {
  curl_global_init(CURL_GLOBAL_ALL);
  curl_handle = curl_easy_init();
}

RESTBridge::~RESTBridge() {
  curl_easy_cleanup(curl_handle);
  curl_global_cleanup();
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
  // Send A POST request to the specified endpoint with the specified data, Use
  // hardcoded headers for now
  CURLcode res;

  if (curl_handle) {
    std::string json_str = data.dump();       // Store the JSON string
    const char *json_data = json_str.c_str(); // Get the C-string pointer
    curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, json_data);

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "charset: utf-8");
    curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, headers);

    // Perform the request, res will get the return code
    res = curl_easy_perform(curl_handle);

    // Check for errors
    if (res != CURLE_OK) {
      RCLCPP_ERROR(get_logger(), "curl_easy_perform() failed: %s\n",
                   curl_easy_strerror(res));
    }
  }
}
