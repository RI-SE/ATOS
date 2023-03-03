/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "mqttbridge.hpp"
#include "mqtt.hpp"
#include <random>

using namespace ROSChannels;

using std::placeholders::_1;

MqttBridge::MqttBridge() : Module(MqttBridge::moduleName),
						   v2xMsgSub(*this, std::bind(&MqttBridge::onV2xMsg, this, _1))
{
	declare_parameter("broker_ip");
	declare_parameter("pub_client_id");
	declare_parameter("username");
	declare_parameter("password");
	declare_parameter("topic");
	declare_parameter("quality_of_service");

	get_parameter("broker_ip", brokerIP);
	get_parameter("pub_client_id", pubClientId);
	get_parameter("username", username);
	get_parameter("password", password);
	get_parameter("topic", topic);
	get_parameter("quality_of_service", QoS);

	timer = this->create_wall_timer(SEND_INTERVAL, std::bind(&MqttBridge::yieldMqttClient, this));

	this->initialize();
}

/*!
 * \brief initializeModule Initializes this module by starting the mqtt client with ros parameter settings.
 *		  return 0 if successfully initalized. None 0 otherwise. 
 */
void MqttBridge::initialize()
{
	RCLCPP_INFO(this->get_logger(), "%s task running with PID: %d", moduleName.c_str(), getpid());
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

/*!
 * \brief Yield MQTT client at intervals shorter then the MQTT_KEEPALIVE_INTERVAL_S to keep the connection open
 *        if no messages are published for a while.
 */
void MqttBridge::yieldMqttClient()
{
	MQTTClient_yield();
}

void MqttBridge::setupConnection()
{
	// Add a random number to avoid conflicting client IDs
	std::random_device rd;
	std::uniform_int_distribution<int> dist(10000, 99999);
	static const std::string client_id = pubClientId + std::to_string(dist(rd));

	RCLCPP_DEBUG(this->get_logger(), "Setting up connection with clientID %s", client_id.c_str());

	MQTTClient mqttClient = MQTT::setupConnection(brokerIP.c_str(),
												  client_id.c_str(),
												  username.c_str(),
												  password.c_str(),
												  MQTT::clientType::publisher,
												  "");

	if (mqttClient == nullptr)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to initialize MQTT connection to broker, exiting...");
		rclcpp::shutdown();
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Successfully initialized MQTT connection to broker with IP %s", brokerIP.c_str());
		this->mqttClient = mqttClient;
	}
}

void MqttBridge::onV2xMsg(const V2X::message_type::SharedPtr v2x_msg)
{
	if (MQTTClient_isConnected(this->mqttClient))
	{
		json payload = v2xToJson(v2x_msg);
		const MQTT::Message mqttMsg = MQTT::Message(payload.dump(), topic.c_str());
		bool retained = false;

		try
		{
			RCLCPP_DEBUG(this->get_logger(), "Publishing MQTT v2x msg to broker %s", payload.dump().c_str());
			MQTT::publishMessage(mqttMsg, this->mqttClient, retained);
		}
		catch (std::runtime_error)
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
