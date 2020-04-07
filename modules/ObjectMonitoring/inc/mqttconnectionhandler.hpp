#ifndef MQTTCONNECTIONHANDLER_H
#define MQTTCONNECTIONHANDLER_H

#include <string>
#include <exception>
#include <unordered_map>

#include <MQTTClient.h>
#include "logging.h"
#include "iso22133.h"
#include "mqtttopichandlers.hpp"

using namespace std;

class MQTTError : virtual public std::exception {
protected:
	int errorCode;
	string errorMessage;
public:
	explicit MQTTError(const string &msg, int errorCode) : errorCode(errorCode), errorMessage(msg) {}

	virtual ~MQTTError() noexcept {}

	virtual const char* what() const noexcept {
		return errorMessage.c_str();
	}

	virtual int getErrorNumber() const noexcept {
		return errorCode;
	}
};


class MQTTConnectionHandler {
public:
	MQTTConnectionHandler(const string clientID);
	void establishConnection(void);

private:

	void setMessageArrivedCallback(MQTTTopicHandlers::MQTTTopicHandler messageCallback) {
		this->messageCallback = messageCallback;
		MQTTClient_setCallbacks(this->client, this, nullptr, &MQTTConnectionHandler::arrivalCallback, nullptr);
		return;
	}
	MQTTTopicHandlers::MQTTTopicHandler messageCallback;

	static int arrivalCallback(void* context, char* topicName, int,
							   MQTTClient_message* message) {
		MQTTConnectionHandler* thisObject = static_cast<MQTTConnectionHandler *>(context);
		string topic(topicName);
		return thisObject->messageCallback(message->payload, topic);
	}

	MQTTClient client;
	string clientID = "";
	string serverURI = "";
	MQTTClient_connectOptions connectionOptions;
	enum : int {
		FIRE_AND_FORGET = 0,
		AT_LEAST_ONCE = 1,
		ONLY_ONCE = 2
	} qualityOfService = FIRE_AND_FORGET;
};

#endif // MQTTCONNECTIONHANDLER_H
