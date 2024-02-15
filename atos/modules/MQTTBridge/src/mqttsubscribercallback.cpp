#include "mqttsubscribercallback.hpp"


MQTTSubscriberCallback::MQTTSubscriberCallback(mqtt::async_client& client, mqtt::connect_options& connectionOptions, std::vector<std::string>& topics) :
	client(client),
	connectionOptions(connectionOptions),
	topics(topics) {}


MQTTSubscriberCallback::~MQTTSubscriberCallback() {}


/**
 * @brief Callback for when the client has connected, subscribe to the topics.
 * 
 * @param cause Cause.
 */
void MQTTSubscriberCallback::connected(const std::string& cause) {
	for (const auto& topic : topics) {
		client.subscribe(topic, 1, nullptr, *this);
	}
}


/**
 * @brief Callback for when a message has arrived, set the message member variable so other
 * classes can access it.
 * 
 * @param msg The message received.
 */
void MQTTSubscriberCallback::message_arrived(mqtt::const_message_ptr msg) {
	mqttMessage = msg->to_string();
}


/**
 * @brief Callback for when the client has failed to connect.
 * 
 * @param tok Token
 */
void MQTTSubscriberCallback::on_failure(const mqtt::token& tok) {
	throw std::runtime_error("Failed to connect to MQTT broker");
}