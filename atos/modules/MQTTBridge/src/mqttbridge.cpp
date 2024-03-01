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
  declare_parameter("broker_ip", "");
  declare_parameter("port", 1883);
  declare_parameter("username", "");
  declare_parameter("password", "");
  declare_parameter("topic_prefix", "atos");
  declare_parameter("quality_of_service", 1);

  get_parameter("broker_ip", brokerIP);
  get_parameter("port", port);
  get_parameter("username", username);
  get_parameter("password", password);
  get_parameter("topic_prefix", topic_prefix);
  get_parameter("quality_of_service", QoS);

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

      RCLCPP_INFO(get_logger(), "Bridging MQTT topic '%s' to %sROS topic '%s'",
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
    this->setupConnection();
  }
}

void MqttBridge::setupConnection() {
  RCLCPP_INFO(this->get_logger(),
              "Setting up connection with clientID: %s, and broker IP: %s",
              clientId.c_str(), brokerIP.c_str());

  mqttClientWrapper =
      std::make_shared<MQTTClientWrapper>(brokerIP, port, username, password);
  mqttClientWrapper->connect();

  if (mqttClientWrapper == nullptr) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize MQTT connection to broker, exiting...");
    rclcpp::shutdown();
  } else {
    RCLCPP_DEBUG(this->get_logger(),
                 "Successfully initialized MQTT connection to broker");
  }
}

void MqttBridge::onV2xMsg(const V2X::message_type::SharedPtr v2x_msg) {
  this->onMessage<V2X::message_type::SharedPtr>(v2x_msg, "v2x", v2xToJson);
}

void MqttBridge::onObcStateChangeMsg(
    const StateChange::message_type::SharedPtr obc_msg) {
  this->onMessage<StateChange::message_type::SharedPtr>(obc_msg, "state",
                                                        obcStateChangeToJson);
}

template <typename T>
void MqttBridge::onMessage(T msg, std::string mqtt_topic,
                           std::function<json(T)> convertFunc) {
  json payload = convertFunc(msg);
  try {
    RCLCPP_DEBUG(this->get_logger(), "Publishing MQTT msg to broker %s",
                 payload.dump().c_str());
    mqttClientWrapper->publishMessage(topic_prefix + mqtt_topic, payload.dump(),
                                      QoS);
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