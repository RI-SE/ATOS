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
  if (msg->type == "icdc") {
    RCLCPP_INFO(get_logger(), "Received ICDC command: %s",
                msg->content.c_str());
    json icdc = parseICDCCommand(msg->content);
    sendRESTMessages(icdc["endpoint"].get<std::string>(), icdc["data"]);
  }
}

json RESTBridge::parseICDCCommand(std::string &msg) {
  // Parse the message and return the REST API message
  std::replace(msg.begin(), msg.end(), '\'',
               '\"'); // Replace single quotes with double quotes to be able to
                      // parse the message
  json j = json::parse(msg);
  return j;
}

void RESTBridge::sendRESTMessages(const std::string &endpoint,
                                  const json &data) {
  // Send the message to the REST API
  CURLcode res;

  if (curl_handle) {
    curl_easy_setopt(curl_handle, CURLOPT_URL, endpoint.c_str());
    curl_easy_setopt(curl_handle, CURLOPT_POSTFIELDS, data.dump().c_str());

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "accept: application/json");
    headers = curl_slist_append(headers, "Content-Type: application/json");
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
