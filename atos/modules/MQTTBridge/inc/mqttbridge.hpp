/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "Imqtt2ros.hpp"
#include "Iros2mqtt.hpp"
#include "atos_interfaces/srv/new_mqtt2_ros_bridge.hpp"
#include "atos_interfaces/srv/new_ros2_mqtt_bridge.hpp"
#include "module.hpp"
#include "roschannels/customcommandaction.hpp"
#include "roschannels/statechange.hpp"
#include <chrono>
#include <fmt/format.h>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/*!
 * \brief The MQTTBridge node forwards ATOS V2X ROS msgs to a MQTTClient
 * publisher
 */

class MqttBridge : public Module,
                   public virtual mqtt::callback,
                   public virtual mqtt::iaction_listener {
public:
  MqttBridge();
  void initialize();

protected:
  /**
   * @brief Loads ROS parameters from parameter server.
   */
  void loadParameters();

  void connect();

  void newMqtt2RosBridge(
      const std::shared_ptr<atos_interfaces::srv::NewMqtt2RosBridge::Request>
          request,
      std::shared_ptr<atos_interfaces::srv::NewMqtt2RosBridge::Response>
          response);

  void newRos2MqttBridge(
      const std::shared_ptr<atos_interfaces::srv::NewRos2MqttBridge::Request>
          request,
      std::shared_ptr<atos_interfaces::srv::NewRos2MqttBridge::Response>
          response);

private:
  /**
   * @brief MQTT2ROS connection variables sorted by MQTT topic
   */
  std::map<std::string, Mqtt2RosInterface> mqtt2ros_;

  /**
   * @brief ROS2MQTT connection variables sorted by ROS topic
   */
  std::map<std::string, Ros2MqttInterface> ros2mqtt_;

  /** @brief ROS Service server for providing dynamic MQTT to ROS mappings.
   */
  rclcpp::Service<atos_interfaces::srv::NewMqtt2RosBridge>::SharedPtr
      new_mqtt2ros_bridge_service_;

  /** @brief ROS Service server for providing dynamic ROS to MQTT mappings.
   */
  rclcpp::Service<atos_interfaces::srv::NewRos2MqttBridge>::SharedPtr
      new_ros2mqtt_bridge_service_;

  /**
   * @brief MQTT client variable
   */
  std::shared_ptr<mqtt::async_client> client_;

  /**
   * @brief MQTT client connection options
   */
  mqtt::connect_options connect_options_;

  /**
   * @brief Callback for when the client receives a MQTT message from the
   * broker.
   *
   * Overrides mqtt::callback::message_arrived(mqtt::const_message_ptr).
   * Publishes any empty ROS message to the corresponding ROS topic.
   *
   * @param   mqtt_msg     MQTT message
   */
  void message_arrived(mqtt::const_message_ptr mqtt_msg) override;

  /**
   * @brief Checks all active ROS topics in order to set up generic subscribers.
   */
  void setupSubscriptions();

  /**
   * @brief Publishes a ROS message received via MQTT to ROS.
   *
   * @param   mqtt_msg       MQTT message
   * @param   arrival_stamp  arrival timestamp used for latency computation
   */
  void mqtt2ros(mqtt::const_message_ptr mqtt_msg,
                const rclcpp::Time &arrival_stamp);

  /**
   * @brief Publishes a MQTT message received via ROS to the MQTT broker.
   *
   * @param   serialized_msg  generic serialized ROS message
   * @param   ros_topic       ROS topic where the message was published
   */
  void
  ros2mqtt(const std::shared_ptr<rclcpp::SerializedMessage> &serialized_msg,
           const std::string &ros_topic);

  /**
   * @brief Callback for when a MQTT action succeeds.
   *
   * Overrides mqtt::iaction_listener::on_success(const mqtt::token&).
   * Does nothing.
   *
   * @param   token        token tracking the action
   */
  void on_success(const mqtt::token &token) override;

  /**
   * @brief Callback for when a MQTT action fails.
   *
   * Overrides mqtt::iaction_listener::on_failure(const mqtt::token&).
   * Logs error.
   *
   * @param   token        token tracking the action
   */
  void on_failure(const mqtt::token &token) override;

  /**
   * @brief Status variable keeping track of connection status to broker
   */
  bool is_connected_ = false;

  static inline std::string const moduleName = "mqtt_bridge";
  constexpr static std::chrono::milliseconds SEND_INTERVAL =
      std::chrono::milliseconds(5000);
  std::string brokerIP;
  int port;
  std::string clientId;
  std::string username;
  std::string password;
  std::string topic_prefix;

  ROSChannels::CustomCommandAction::Sub
      customCommandActionMsgSub; //!< Subscriber to v2x messages requests
  ROSChannels::StateChange::Sub
      obcStateChangeSub; //!< Subscriber to object state change requests

  void setupClient();
  void setupMqtt2RosBridge();
  void setupRos2MqttBridge();
  void onCustomCommandActionMsg(
      const ROSChannels::CustomCommandAction::message_type::SharedPtr);
  void
  onObcStateChangeMsg(const ROSChannels::StateChange::message_type::SharedPtr);

  template <typename T>
  void onMessage(T msg, std::string mqtt_topic,
                 std::function<json(T)> convertFunc);

  static json v2xToJson(const std::string v2x_msg);
  static json obcStateChangeToJson(
      const ROSChannels::StateChange::message_type::SharedPtr obc_msg);
};