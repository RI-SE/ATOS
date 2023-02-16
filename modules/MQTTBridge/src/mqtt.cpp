/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "mqtt.hpp"
#include <rclcpp/rclcpp.hpp>

#include <cstring>
#include <sstream>

std::string subMessageTopic;
std::string subMessagePayload;
using namespace rclcpp;


/*!
 * \brief MQTT::publishMessage Publishes a specified message to the selected
 *			MQTT topic.
 * \param message Message to be published.
 * \param topicName Name of the MQTT topic to be published to.
 * \param client MQTT client handle, assumed to be connected.
 * \param retained Boolean indicating whether the message should be retained.
 * \return 0 if successful, -1 otherwise.
 */
int MQTT::publishMessage(const Message& message,
		const MQTTClient &client,
		bool retained) {

	constexpr int deliveryTimeout_ms = MQTT_DELIVERY_TIMEOUT_MS;
	MQTTClient_deliveryToken token;
	MQTTClient_message mqttMessage = MQTTClient_message_initializer;
	char* messageCopy = new char[message.payload.length()];
	memset(messageCopy, 0, message.payload.length());
	int rc;

	mqttMessage.retained = retained;
	mqttMessage.qos = 0;
	mqttMessage.payloadlen = static_cast<int>(message.payload.length());
	memcpy(messageCopy, message.payload.c_str(), message.payload.length());
	mqttMessage.payload = messageCopy;

	rc = MQTTClient_publishMessage(client, message.topic.c_str(), &mqttMessage, &token);
	if (rc != MQTTCLIENT_SUCCESS) {
		std::string errDetails;
		std::stringstream ss(errDetails);
		ss << "Attempted to publish:" << std::endl;
		ss << "\tTopic: " << message.topic.c_str() << std::endl;
		ss << "\tMessage: " << std::string(static_cast<char*>(mqttMessage.payload), message.payload.length()) << std::endl;
		delete [] messageCopy;
		throw MQTT::Error("Failed to publish message to MQTT broker", ss.str(), rc);
	}
	else {
		rc = MQTTClient_waitForCompletion(client, token, deliveryTimeout_ms);
		if (rc != MQTTCLIENT_SUCCESS) {
			delete[] messageCopy;
			throw MQTT::Error("Timed out while waiting for MQTT publish completion", rc);
		}
	}

	delete[] messageCopy;
	return 0;
}

/*!
 * \brief MQTT::publishMessages
 * \param messagesAndTopics
 * \param client
 * \param retained
 * \return
 */
int MQTT::publishMessages(
		const std::vector<MQTT::Message>& messagesAndTopics,
		const MQTTClient &client,
		bool retained) {

	for (const auto &messageAndTopic : messagesAndTopics) {
		if (MQTT::publishMessage(messageAndTopic, client, retained) < 0) {
			return -1;
		}
	}
	return 0;
}


/*!
 * \brief messageGetStrBetween Get the string between two strings
 * \param full Complete string
 * \param begin First search string
 * \param end Second search string
 * \param middle String between first and secong search string
 * \return 0 if successful, -1 otherwise.
 */
int MQTT::messageGetStrBetween(const std::string full,
								const std::string begin,
								const std::string end,
								std::string &middle)
{
	size_t pos;
	pos=full.find(begin);
	if(pos > 0 && pos < full.length()){
		std::string str = full.substr(pos + begin.length());
		pos=str.find(end);
		str.erase(pos);
		if(str.length() > 0){
			middle = str;
			return str.length();
		} else {
			return -1;
		}
	} else return -1;
}


volatile MQTTClient_deliveryToken deliveredtoken;

/*!
 * \brief MQTT::hasSubscriptionMsgArrived Check if callback function has stored received data from broker
 * \param *topicPtr Pointer to topic string
 * \param *payloadPtr Pointer to payload string
 * \return void 
 */
int MQTT::hasSubscriptionMsgArrived(std::string *topicPtr, std::string *payloadPtr)
{
	int ret = 0;
	*topicPtr = subMessageTopic;
	*payloadPtr = subMessagePayload;
	if(subMessageTopic.length() > 0 && subMessageTopic.length() > 0 )
		ret = subMessageTopic.length() + subMessageTopic.length();
	return ret;
}


/*!
 * \brief MQTT::clearSubscriptionMsg Set subscription variables to empty strings
 * \return void 
 */
void MQTT::clearSubscriptionMsg()
{
	subMessageTopic = "";
	subMessagePayload = "";
}


/*!
 * \brief MQTT::msgDelivered Callback function for message delivered to mqtt broker
 * \param context
 * \param dt
 * \return void 
 */
void MQTT::msgDelivered(void *context, MQTTClient_deliveryToken dt)
{
    deliveredtoken = dt;
}

/*!
 * \brief MQTT::msgArrived Callback function for message arrived from mqtt broker
 * \param context
 * \param topicName
 * \param topicLen
 * \param message
 * \return 1
 */
int MQTT::msgArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
	if(strlen(topicName) > 0)
	{
	 	subMessageTopic = topicName;

	}
    if(strlen((char*)message->payload))
	{    
    	subMessagePayload = std::string((char*)message->payload);
    }
    
  	MQTTClient_freeMessage(&message);
	MQTTClient_free(topicName);
        
    return 1;
}

/*!
 * \brief MQTT::connLost Callback function for connection lost from mqtt broker
 * \param context
 * \param cause
 * \return void
 */
void MQTT::connLost(void *context, char *cause)
{
	RCLCPP_DEBUG(get_logger("MQTT"), "Connection lost, cause %s \n", cause);
}


/*!
 * \brief setupMQTTConnection Sets up an MQTT client and connects to the selected
 *			broker IP.
 * \param brokerAddress IP address of the MQTT broker.
 * \param clientID Client name to be used when connecting (must be unique!).
 * \param username Username 
 * \param password Password
 * \param clientType Type of client, subscriber or publisher
 * \param subTopic The topic the subscriber subscribes to
 * \return Value according to ::MQTTClient, or nullptr if failed.
 */

#define QOS         1

MQTTClient MQTT::setupConnection(
		const std::string &brokerAddress,
		const std::string &clientID,
		const std::string &username,
		const std::string &password,
		const clientType clientType,
		const std::string& subTopic) {
	MQTTClient client;
	MQTTClient_connectOptions conn_opts;
	int rc;

	conn_opts = MQTTClient_connectOptions_initializer;
	rc = MQTTClient_create(&client, brokerAddress.c_str(),
						   clientID.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr);

	if (rc != MQTTCLIENT_SUCCESS) {
		RCLCPP_ERROR(get_logger("MQTT"), "Failed to create MQTT client");
		return nullptr;
	}

	
	if(clientType == subscriber){
		if ((rc = MQTTClient_setCallbacks(client, NULL, connLost, msgArrived, msgDelivered)) != MQTTCLIENT_SUCCESS)
    	{
			RCLCPP_ERROR(get_logger("MQTT"), "Failed to set callbacks, return code %d\n", rc);
        	rc = EXIT_FAILURE;
        	return nullptr;
    	}
	}

	conn_opts.keepAliveInterval = MQTT::keepAliveInterval_s;
	conn_opts.cleansession = 1;
	conn_opts.connectTimeout = 3;
	conn_opts.username = username.c_str();
	conn_opts.password = password.c_str();

	rc = MQTTClient_connect(client, &conn_opts);
	if (rc != MQTTCLIENT_SUCCESS) {
		RCLCPP_ERROR(get_logger("MQTT"), "Failed to connect to MQTT broker");
		MQTTClient_destroy(&client);
		return nullptr;
	}

	if(clientType == publisher) RCLCPP_INFO(get_logger("MQTT"), "Connected to MQTT broker as publisher." );
	else if (clientType == subscriber) RCLCPP_INFO(get_logger("MQTT"), "Connected to MQTT broker as subscriber." );

	if(clientType == subscriber)
	{
	   	
	   	if ((rc = MQTTClient_subscribe(client, subTopic.c_str(), QOS)) != MQTTCLIENT_SUCCESS)
    	{
			RCLCPP_ERROR(get_logger("MQTT"), "Failed to subscribe to: %s", subTopic.c_str() );
			MQTTClient_destroy(&client);
			return nullptr;
    	}
		RCLCPP_INFO(get_logger("MQTT"), "Subscribing to %s", subTopic.c_str() );

	}

	return client;
}


/*!
 * \brief MQTT::Topic::prettify Replaces all whitespace characters with dashes,
 *			and upper case letters with lower case.
 * \param topicName Topic name to be prettified.
 * \return Reference to the prettified topic name.
 */
std::string MQTT::prettifyTopic(const std::string& topic) {
	std::string retval = topic;
	for (char &c : retval) {
		if (std::isblank(c)) {
			c = '-';
		}
		else if (std::isupper(c)) {
			c = static_cast<char>(std::tolower(c));
		}
	}
	return retval;
}
