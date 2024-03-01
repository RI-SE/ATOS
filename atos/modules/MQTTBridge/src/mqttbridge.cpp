/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "mqttbridge.hpp"
#include <random>

using namespace ROSChannels;

using std::placeholders::_1;

MqttBridge::MqttBridge()
    : Module(MqttBridge::moduleName),
      v2xMsgSub(*this, std::bind(&MqttBridge::onV2xMsg, this, _1)),
      obcStateChangeSub(*this,
                        std::bind(&MqttBridge::onObcStateChangeMsg, this, _1)) {
  this->loadParameters();
  this->initialize();
}

void MqttBridge::loadParameters() {
  declare_parameter("broker.host", "");
  declare_parameter("broker.port", 1883);
  declare_parameter("client.username", "");
  declare_parameter("client.password", "");
  declare_parameter("client.id", "");
  declare_parameter("topic_prefix", "atos");

  get_parameter("broker.host", brokerIP);
  get_parameter("broker.port", port);
  get_parameter("client.username", username);
  get_parameter("client.password", password);
  get_parameter("client.id", clientId);
  get_parameter("topic_prefix", topic_prefix);

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "The list of topics to bridge from MQTT to ROS";
  const auto mqtt2ros_mqtt_topics = declare_parameter<std::vector<std::string>>(
      "mqtt2ros.mqtt_topics", std::vector<std::string>(), param_desc);
  for (const auto &mqtt_topic : mqtt2ros_mqtt_topics) {
    param_desc.description =
        "ROS topic on which corresponding MQTT messages are published";
    declare_parameter(fmt::format("mqtt2ros.{}.ros_topic", mqtt_topic),
                      rclcpp::ParameterType::PARAMETER_STRING, param_desc);
    param_desc.description = "MQTT QoS value";
    declare_parameter(fmt::format("mqtt2ros.{}.advanced.mqtt.qos", mqtt_topic),
                      rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
    param_desc.description = "ROS publisher queue size";
    declare_parameter(
        fmt::format("mqtt2ros.{}.advanced.ros.queue_size", mqtt_topic),
        rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
  }

  // mqtt2ros
  for (const auto &mqtt_topic : mqtt2ros_mqtt_topics) {

    rclcpp::Parameter ros_topic_param;
    if (get_parameter(fmt::format("mqtt2ros.{}.ros_topic", mqtt_topic),
                      ros_topic_param)) {

      // mqtt2ros[k]/mqtt_topic and mqtt2ros[k]/ros_topic
      const std::string ros_topic = ros_topic_param.as_string();
      Mqtt2RosInterface &mqtt2ros = mqtt2ros_[mqtt_topic];
      mqtt2ros.ros.topic = ros_topic;

      // mqtt2ros[k]/advanced/mqtt/qos
      rclcpp::Parameter qos_param;
      if (get_parameter(
              fmt::format("mqtt2ros.{}.advanced.mqtt.qos", mqtt_topic),
              qos_param))
        mqtt2ros.mqtt.qos = qos_param.as_int();

      // mqtt2ros[k]/advanced/ros/queue_size
      rclcpp::Parameter queue_size_param;
      if (get_parameter(
              fmt::format("mqtt2ros.{}.advanced.ros.queue_size", mqtt_topic),
              queue_size_param))
        mqtt2ros.ros.queue_size = queue_size_param.as_int();

      RCLCPP_INFO(get_logger(), "Bridging MQTT topic '%s' to ROS topic '%s'",
                  mqtt_topic.c_str(), mqtt2ros.ros.topic.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  fmt::format("Parameter 'ros2mqtt.{}' is missing subparameter "
                              "'ros_topic', will be ignored",
                              mqtt_topic)
                      .c_str());
    }
  }
}

/*!
 * \brief initializeModule Initializes this module by starting the mqtt client
 * with ros parameter settings.
 */
void MqttBridge::initialize() {
  if (this->brokerIP.empty()) {
    RCLCPP_INFO(this->get_logger(),
                "No Broker IP provided in configuration. Shutting down...");
    rclcpp::shutdown();
  } else {
    this->setupClient();
    this->connect();

    new_mqtt2ros_bridge_service_ =
        create_service<atos_interfaces::srv::NewMqtt2RosBridge>(
            "~/new_mqtt2ros_bridge",
            std::bind(&MqttBridge::newMqtt2RosBridge, this,
                      std::placeholders::_1, std::placeholders::_2));

    // Do a service call to create the MQTT2ROS bridges for each MQTT topic in
    // the parameter list
    for (const auto &mqtt2ros : mqtt2ros_) {
      atos_interfaces::srv::NewMqtt2RosBridge::Request::SharedPtr request =
          std::make_shared<atos_interfaces::srv::NewMqtt2RosBridge::Request>();
      request->mqtt_topic = mqtt2ros.first;
      request->ros_topic = mqtt2ros.second.ros.topic;
      request->mqtt_qos = mqtt2ros.second.mqtt.qos;
      request->ros_queue_size = mqtt2ros.second.ros.queue_size;
      atos_interfaces::srv::NewMqtt2RosBridge::Response::SharedPtr response =
          std::make_shared<atos_interfaces::srv::NewMqtt2RosBridge::Response>();
      newMqtt2RosBridge(request, response);
      if (!response->success) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create MQTT2ROS bridge for topic '%s'",
                     mqtt2ros.first.c_str());
      }
    }
  }
}

void MqttBridge::setupClient() {
  RCLCPP_INFO(this->get_logger(),
              "Setting up connection with clientID: %s, and broker IP: %s",
              clientId.c_str(), brokerIP.c_str());

  // basic client connection options
  connect_options_.set_automatic_reconnect(true);
  connect_options_.set_clean_session(true);
  connect_options_.set_keep_alive_interval(true);
  connect_options_.set_max_inflight(true);

  // user authentication
  if (!username.empty()) {
    connect_options_.set_user_name(username);
    connect_options_.set_password(password);
  }

  auto timeNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  auto id = clientId + "_" + std::to_string(timeNanoseconds);

  const std::string uri = fmt::format("{}://{}:{}", "tcp", brokerIP, port);
  try {
    client_ =
        std::shared_ptr<mqtt::async_client>(new mqtt::async_client(uri, id));
  } catch (const mqtt::exception &e) {
    RCLCPP_ERROR(get_logger(), "Client could not be initialized: %s", e.what());
    exit(EXIT_FAILURE);
  }

  // setup MQTT callbacks
  client_->set_callback(*this);
}

void MqttBridge::connect() {

  std::string as_client =
      clientId.empty() ? ""
                       : std::string(" as '") + clientId + std::string("'");
  RCLCPP_INFO(get_logger(), "Connecting to broker at '%s'%s ...",
              client_->get_server_uri().c_str(), as_client.c_str());

  try {
    client_->connect(connect_options_, nullptr, *this);
  } catch (const mqtt::exception &e) {
    RCLCPP_ERROR(get_logger(), "Connection to broker failed: %s", e.what());
    exit(EXIT_FAILURE);
  }
}

void MqttBridge::newMqtt2RosBridge(
    atos_interfaces::srv::NewMqtt2RosBridge::Request::SharedPtr request,
    atos_interfaces::srv::NewMqtt2RosBridge::Response::SharedPtr response) {

  // add mapping definition to mqtt2ros_
  Mqtt2RosInterface &mqtt2ros = mqtt2ros_[request->mqtt_topic];
  mqtt2ros.ros.is_stale = true;
  mqtt2ros.ros.topic = request->ros_topic;
  mqtt2ros.mqtt.qos = request->mqtt_qos;
  mqtt2ros.ros.publisher = this->create_publisher<std_msgs::msg::Empty>(
      mqtt2ros.ros.topic, request->ros_queue_size);
  mqtt2ros.ros.queue_size = request->ros_queue_size;

  RCLCPP_INFO(get_logger(), "Bridging MQTT topic '%s' to ROS topic '%s'",
              request->mqtt_topic.c_str(), mqtt2ros.ros.topic.c_str());

  // subscribe to the MQTT topic
  std::string mqtt_topic_to_subscribe = request->mqtt_topic;
  client_->subscribe(mqtt_topic_to_subscribe, mqtt2ros.mqtt.qos);
  RCLCPP_INFO(get_logger(), "Subscribed MQTT topic '%s'",
              mqtt_topic_to_subscribe.c_str());
  response->success = true;
}

void MqttBridge::onV2xMsg(const V2X::message_type::SharedPtr v2x_msg) {
  this->onMessage<V2X::message_type::SharedPtr>(v2x_msg, "v2x", v2xToJson);
}

void MqttBridge::onObcStateChangeMsg(
    const StateChange::message_type::SharedPtr obc_msg) {
  this->onMessage<StateChange::message_type::SharedPtr>(obc_msg, "state",
                                                        obcStateChangeToJson);
}

void MqttBridge::message_arrived(mqtt::const_message_ptr mqtt_msg) {
  std::string mqtt_topic = mqtt_msg->get_topic();
  RCLCPP_DEBUG(get_logger(), "Received MQTT message on topic '%s'",
               mqtt_topic.c_str());
  Mqtt2RosInterface &mqtt2ros = mqtt2ros_[mqtt_topic];

  // Publish empty message to ROS topic
  mqtt2ros.ros.publisher->publish(std_msgs::msg::Empty());
  RCLCPP_DEBUG(get_logger(), "Published empty message to ROS topic '%s'",
               mqtt2ros.ros.topic.c_str());
}

template <typename T>
void MqttBridge::onMessage(T msg, std::string mqtt_topic,
                           std::function<json(T)> convertFunc) {
  json payload = convertFunc(msg);
  try {
    RCLCPP_DEBUG(this->get_logger(), "Publishing MQTT msg to broker %s",
                 payload.dump().c_str());
    client_->publish(mqtt_topic, payload.dump().c_str(), payload.dump().size());
  } catch (std::runtime_error &) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish MQTT message");
  }
}

json MqttBridge::v2xToJson(const V2X::message_type::SharedPtr v2x_msg) {
  json j;
  j["message_type"] = v2x_msg->message_type;
  j["event_id"] = v2x_msg->event_id;
  j["cause_code"] = v2x_msg->cause_code;
  j["detection_time"] = v2x_msg->detection_time;
  j["altitude"] = v2x_msg->altitude;
  j["latitude"] = v2x_msg->latitude;
  j["longitude"] = v2x_msg->longitude;
  return j;
}

json MqttBridge::obcStateChangeToJson(
    const StateChange::message_type::SharedPtr obc_msg) {
  json j;
  j["current_state"] = obc_msg->current_state;
  j["prev_state"] = obc_msg->prev_state;
  return j;
}

void MqttBridge::on_success(const mqtt::token &token) {

  (void)token; // Avoid compiler warning for unused parameter.
  is_connected_ = true;
}

void MqttBridge::on_failure(const mqtt::token &token) {

  RCLCPP_ERROR(
      get_logger(),
      "Connection to broker failed (return code %d), will automatically "
      "retry...",
      token.get_return_code());
  is_connected_ = false;
}