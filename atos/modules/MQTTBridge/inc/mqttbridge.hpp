/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "Imqtt2ros.hpp"
#include "atos_interfaces/srv/new_mqtt2_ros_bridge.hpp"
#include "module.hpp"
#include "roschannels/statechange.hpp"
#include "roschannels/v2xchannel.hpp"
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

private:
  /**
   * @brief MQTT2ROS connection variables sorted by MQTT topic
   */
  std::map<std::string, Mqtt2RosInterface> mqtt2ros_;

  /** @brief ROS Service server for providing dynamic MQTT to ROS mappings.
   */
  rclcpp::Service<atos_interfaces::srv::NewMqtt2RosBridge>::SharedPtr
      new_mqtt2ros_bridge_service_;

  /**
   * @brief MQTT client variable
   */
  std::shared_ptr<mqtt::async_client> client_;

  /**
   * @brief MQTT client connection options
   */
  mqtt::connect_options connect_options_;

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
  int QoS;

  ROSChannels::V2X::Sub v2xMsgSub; //!< Subscriber to v2x messages requests
  ROSChannels::StateChange::Sub
      obcStateChangeSub; //!< Subscriber to object state change requests

  void setupConnection();
  void onV2xMsg(const ROSChannels::V2X::message_type::SharedPtr);
  void
  onObcStateChangeMsg(const ROSChannels::StateChange::message_type::SharedPtr);

  template <typename T>
  void onMessage(T msg, std::string mqtt_topic,
                 std::function<json(T)> convertFunc);

  static json
  v2xToJson(const ROSChannels::V2X::message_type::SharedPtr v2x_msg);
  static json obcStateChangeToJson(
      const ROSChannels::StateChange::message_type::SharedPtr obc_msg);
};