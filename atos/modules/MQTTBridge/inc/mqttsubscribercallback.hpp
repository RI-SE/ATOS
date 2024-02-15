#pragma once

#include <mqtt/async_client.h>
#include <mqtt/callback.h>


class MQTTSubscriberCallback : public virtual mqtt::callback, public virtual mqtt::iaction_listener {

	public:
		MQTTSubscriberCallback(mqtt::async_client& client, mqtt::connect_options& connectionOptions, std::vector<std::string>& topics);
		~MQTTSubscriberCallback();

		std::string mqttMessage = "";

	private:
		mqtt::async_client& client;
		mqtt::connect_options& connectionOptions;
		std::vector<std::string> topics;

		void message_arrived(mqtt::const_message_ptr msg) override;
		void on_failure(const mqtt::token& tok) override;
		void on_success(const mqtt::token& tok) override {}
		void connected(const std::string& cause) override;
};