/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "mqttsubscribercallback.hpp"
#include <mqtt/async_client.h>
#include <string>
#include <vector>

class MQTTClientWrapper {

public:
  MQTTClientWrapper(const std::string &host, const int &port,
                    const std::string &username, const std::string &password);
  ~MQTTClientWrapper();

  std::string getMqttMessage() const;
  void connect();
  void setupSubscriptions(std::vector<std::string> &mqttTopics);
  void publishMessage(const std::string &topic, const std::string &message,
                      const int QoS);
  bool isConnected() { return mqttClient->is_connected(); }

private:
  std::unique_ptr<mqtt::async_client> mqttClient;
  std::unique_ptr<MQTTSubscriberCallback> mqttSubscriberCallback;

  mqtt::connect_options connectionOptions;

  std::string mqttHost;
  int mqttPort;
  std::string mqttClientID = "atos";
  std::string mqttUser;
  std::string mqttPass;

  // constants
  const int MIN_RECONNECT_INTERVAL_SECONDS = 1;
  const int MAX_RECONNECT_INTERVAL_SECONDS = 30;
  const bool REMEMBER_RECONNECT_STATE = false;
  const int KEEP_ALIVE_INTERVAL_SECONDS = 0;
};