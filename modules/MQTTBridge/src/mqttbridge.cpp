#include "mqttbridge.hpp"
#include "mqtt.hpp"

using namespace ROSChannels;

std::shared_ptr<MqttBridge> MqttBridge::me = NULL;
using std::placeholders::_1;

/*!
 * \brief Creates an instance and initialize mqttbridge if none exists, otherwise returns the existing instance.
 * \return the sole MqttBridge instance.
 */
std::shared_ptr<MqttBridge> MqttBridge::instance()
{
	if (me == nullptr)
	{
		me = std::shared_ptr<MqttBridge>(new MqttBridge());
	}
	return me;
}

MqttBridge::MqttBridge() : Module(MqttBridge::moduleName),
						   v2xMsgSub(*this, std::bind(&MqttBridge::onV2xMsg, this, _1)),
						   mqttAbortSub(*this, std::bind(&MqttBridge::onAbortMessage, this, _1))
{
	declare_parameter("brokerIP");
	declare_parameter("pubClientId");
	declare_parameter("username");
	declare_parameter("password");
	declare_parameter("topic");
	declare_parameter("QoS");

	get_parameter("brokerIP", brokerIP);
	get_parameter("pubClientId", pubClientId);
	get_parameter("username", username);
	get_parameter("password", password);
	get_parameter("topic", topic);
	get_parameter("QoS", QoS);

	timer = this->create_wall_timer(SEND_INTERVAL, std::bind(&MqttBridge::yieldMqttClient, this));
}

/*!
 * \brief initializeModule Initializes this module by starting the mqtt client with conf setting.
 */
void MqttBridge::initializeModule()
{
	RCLCPP_INFO(me->get_logger(), "%s task running with PID: %d", moduleName.c_str(), getpid());
	me->setupConnection();
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
	RCLCPP_INFO(me->get_logger(), "Setting up MQTT connection with param %s", brokerIP.c_str());

	MQTTClient mqttClient = MQTT::setupConnection(brokerIP.c_str(),
												  QoS.c_str(),
												  username.c_str(),
												  password.c_str(),
												  MQTT::clientType::publisher,
												  "");

	if (mqttClient == nullptr)
	{
		RCLCPP_ERROR(me->get_logger(), "Failed to initialize MQTT connection to broker, exiting...");
		exit(1);
	}
	else
	{
		RCLCPP_INFO(me->get_logger(), "Successfully initialized MQTT connection to broker");
		me->mqttClient = mqttClient;
	}
}

void MqttBridge::onAbortMessage(const Abort::message_type::SharedPtr)
{
	// Any exceptions here should crash the program
}

void MqttBridge::onV2xMsg(const V2X::message_type::SharedPtr v2x_msg)
{
	if (MQTTClient_isConnected(me->mqttClient))
	{
		json payload = v2xToJson(v2x_msg);
		const MQTT::Message mqttMsg = MQTT::Message(payload.dump(), topic.c_str());
		bool retained = false;

		try
		{
			RCLCPP_DEBUG(me->get_logger(), "Publishing MQTT v2x msg to broker %s", payload.dump().c_str());
			MQTT::publishMessage(mqttMsg, me->mqttClient, retained);
		}
		catch (std::runtime_error)
		{
			RCLCPP_ERROR(me->get_logger(), "Failed to publish MQTT message");
		}
	}
	else
	{
		RCLCPP_ERROR(me->get_logger(), "Received v2x msg while the client is not connected, dropping msg...");
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
