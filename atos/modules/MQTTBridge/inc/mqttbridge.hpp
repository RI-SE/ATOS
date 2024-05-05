/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "Imqtt2ros.hpp"
#include "atos_interfaces/srv/new_mqtt2_ros_bridge.hpp"
#include "module.hpp"
#include "mqttclientwrapper.hpp"
#include "roschannels/statechange.hpp"
#include "roschannels/v2xchannel.hpp"
#include <chrono>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/*!
 * \brief The MQTTBridge node forwards ATOS V2X ROS msgs to a MQTTClient
 * publisher
 */

class MqttBridge : public Module {
public:
  MqttBridge();
  void initialize();

protected:
  /**
   * @brief Loads ROS parameters from parameter server.
   */
  void loadParameters();

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

  std::shared_ptr<MQTTClientWrapper> mqttClientWrapper;
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