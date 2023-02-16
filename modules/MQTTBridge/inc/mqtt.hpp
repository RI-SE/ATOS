/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <stdexcept>
#include <vector>
#include "MQTTClient.h"

#define MQTT_DELIVERY_TIMEOUT_MS 	1
#define MQTT_KEEPALIVE_INTERVAL_S 	20

namespace MQTT {
static int keepAliveInterval_s = MQTT_KEEPALIVE_INTERVAL_S;
static int yieldInterval_s = MQTT::keepAliveInterval_s/5;

enum clientType: std::uint8_t
{
	publisher, subscriber
};

std::string prettifyTopic(const std::string& topic);

class Message {
public:
	std::string topic;
	std::string payload;

	Message() {}
	Message(const std::string& msg, const std::string& topic)
		: payload(msg), topic(topic) {}
};

static std::string subMessageTopic;
static std::string subMessagePayload;

int hasSubscriptionMsgArrived(std::string *topicPtr, std::string *payloadPtr);
void clearSubscriptionMsg();

int publishMessage(const Message& message,
		const MQTTClient &client, bool retained = false);
int publishMessages(const std::vector<MQTT::Message>& messagesAndTopics,
		const MQTTClient &client, bool retained = false);
MQTTClient setupConnection(const std::string& brokerAddress,
		const std::string& clientID, const std::string& username,
		const std::string& password, clientType clientType, const std::string& subTopic);
int messageGetStrBetween(const std::string full, const std::string begin,
							const std::string end, std::string &middle);
void connLost(void *context, char *cause);
void msgDelivered(void *context, MQTTClient_deliveryToken dt);
int msgArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message);

class Error : virtual public std::runtime_error {
protected:
	int errorCode = 1; // Unknown error code
	std::string errorDetails;

public:
	explicit Error(const std::string& msg,
				   const std::string& details,
				   int errNum) : MQTT::Error(msg, errNum) {
		errorDetails = details;
	}
	explicit Error(const std::string& msg,
				   int errNum) : MQTT::Error(msg) {
		errorCode = errNum;
	}
	explicit Error(const std::string& msg) : std::runtime_error(msg) {
	}

	virtual ~Error() noexcept {}
	virtual int code() const noexcept {
		return errorCode;
	}
	virtual const char* what() const noexcept {
		std::string wh;
		wh += std::runtime_error::what();
		wh += ": ";
		wh += MQTTClient_strerror(errorCode);
		return wh.c_str();
	}
	virtual std::string strerror() const noexcept {
		return MQTTClient_strerror(errorCode);
	}
	virtual std::string details() const noexcept {
		return errorDetails.empty() ? "<no details provided>" : errorDetails;
	}


};


}