/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "mqttclient.hpp"

MQTTClientWrapper::MQTTClientWrapper(const std::string &host, const int &port,
                                     const std::string &username,
                                     const std::string &password)
    : mqttHost(host), mqttPort(port), mqttUser(username), mqttPass(password) {

  // Each client needs a unique ID, adding a timestamp to the end of the ID
  // should be enough
  auto timeNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  mqttClientID += "_" + std::to_string(timeNanoseconds);

  mqttClient = std::make_unique<mqtt::async_client>(
      mqttHost + ":" + std::to_string(mqttPort), mqttClientID);
  connectionOptions =
      mqtt::connect_options_builder()
          .user_name(mqttUser)
          .password(mqttPass)
          .automatic_reconnect(
              std::chrono::seconds(MIN_RECONNECT_INTERVAL_SECONDS),
              std::chrono::seconds(MAX_RECONNECT_INTERVAL_SECONDS))
          .clean_session(REMEMBER_RECONNECT_STATE)
          .keep_alive_interval(
              std::chrono::seconds(KEEP_ALIVE_INTERVAL_SECONDS))
          .finalize();
}

MQTTClientWrapper::~MQTTClientWrapper() {
  if (mqttClient->is_connected()) {
    mqtt::disconnect_options disconnectOpts;
    disconnectOpts.set_timeout(std::chrono::seconds(30));
    mqttClient->disconnect(disconnectOpts)->wait();
  }
}

/**
 * @brief Connect to the MQTT broker with the connection options.
 *
 */
void MQTTClientWrapper::connect() { mqttClient->connect(connectionOptions); }

/**
 * @brief Setup the MQTT subscription with the topics provided and connect to
 * the broker.
 *
 */
void MQTTClientWrapper::setupSubscriptions(
    std::vector<std::string> &mqttTopics) {
  mqttSubscriberCallback = std::make_unique<MQTTSubscriberCallback>(
      *mqttClient, connectionOptions, mqttTopics);
  mqttClient->set_callback(*mqttSubscriberCallback);
  mqttClient->connect(connectionOptions, nullptr, *mqttSubscriberCallback);
}

/**
 * @brief Get the latest MQTT message from the callback.
 *
 * @return std::string The message received.
 */
std::string MQTTClientWrapper::getMqttMessage() const {
  return mqttSubscriberCallback->mqttMessage;
}

/**
 * @brief Publish a message to a topic.
 *
 * @param topic The topic to publish to.
 * @param message The message to publish.
 */
void MQTTClientWrapper::publishMessage(const std::string &topic,
                                       const std::string &message,
                                       const int &qos) {
  if (mqttClient->is_connected()) {
    mqtt::message_ptr pubmsg = mqtt::make_message(topic, message);
    pubmsg->set_qos(qos);
    mqttClient->publish(pubmsg);
  } else {
    throw std::runtime_error("MQTT client is not connected");
  }
}