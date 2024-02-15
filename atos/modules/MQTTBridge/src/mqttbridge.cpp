/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "mqttbridge.hpp"
#include "mqttclient.hpp"
#include <random>


using namespace ROSChannels;

using std::placeholders::_1;

MqttBridge::MqttBridge() : Module(MqttBridge::moduleName),
						   v2xMsgSub(*this, std::bind(&MqttBridge::onV2xMsg, this, _1))
{
	declare_parameter("broker_ip","");
	declare_parameter("port", 1883);
	declare_parameter("client_id","");
	declare_parameter("username","");
	declare_parameter("password","");
	declare_parameter("topic","");
	declare_parameter("quality_of_service", 1);

	get_parameter("broker_ip", brokerIP);
	get_parameter("port", port);	
	get_parameter("client_id", clientId);
	get_parameter("username", username);
	get_parameter("password", password);
	get_parameter("topic", topic);
	get_parameter("quality_of_service", QoS);

	this->initialize();
}

/*!
 * \brief initializeModule Initializes this module by starting the mqtt client with ros parameter settings.
 */
void MqttBridge::initialize()
{
	if (this->brokerIP.empty())
	{
		RCLCPP_INFO(this->get_logger(), "No Broker IP provided in configuration. Shutting down...");
		rclcpp::shutdown();
	}
	else
	{
		this->setupConnection();
	}
}

void MqttBridge::setupConnection()
{
	RCLCPP_INFO(this->get_logger(), "Setting up connection with clientID: %s, and broker IP: %s", clientId.c_str(), brokerIP.c_str());

	mqttClient = std::make_shared<MQTTClient>(brokerIP, port, username, password);
	mqttClient->connect();

	if (mqttClient == nullptr)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to initialize MQTT connection to broker, exiting...");
		rclcpp::shutdown();
	}
	else
	{
		RCLCPP_DEBUG(this->get_logger(), "Successfully initialized MQTT connection to broker");
	}
}

void MqttBridge::onV2xMsg(const V2X::message_type::SharedPtr v2x_msg)
{
	if (mqttClient->isConnected())
	{
		json payload = v2xToJson(v2x_msg);
		bool retained = false;

		try
		{
			RCLCPP_DEBUG(this->get_logger(), "Publishing MQTT v2x msg to broker %s", payload.dump().c_str());
			mqttClient->publishMessage(topic, payload.dump());
		}
		catch (std::runtime_error&)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to publish MQTT message");
		}
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Received v2x msg while the client is disconnected, dropping msg...");
	}
}

json MqttBridge::v2xToJson(const V2X::message_type::SharedPtr v2x_msg)
{
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
