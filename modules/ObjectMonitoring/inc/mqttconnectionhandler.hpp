#ifndef MQTTCONNECTIONHANDLER_H
#define MQTTCONNECTIONHANDLER_H

#include <string>
#include <exception>

#include <MQTTClient.h>
#include "logging.h"
#include "iso22133.h"

using namespace std;

class MQTTError : virtual public std::exception {
protected:
	int errorCode;
	string errorMessage;
public:
	explicit MQTTError(string msg, int errorCode) : errorCode(errorCode), msg(errorMessage) {}

	virtual const char* what() const throw () {
		return errorMessage.c_str();
	}

	virtual int getErrorNumber() const throw () {
		return errorCode;
	}
} ;


class MQTTConnectionHandler {
public:
	MQTTConnectionHandler(const string clientID);
	void establishConnection(void);

private:

	MQTTClient client;
	string clientID = "";
	string serverURI = "";
	MQTTClient_connectOptions connectionOptions;

	template <typename MQTTDataType> class MQTTConnection {
	public:
		MQTTConnection(MQTTClient &client, string &topicExpression, int messageCallback(MQTTDataType* message,
							string &topicExpression)) : topicExpression(topicExpression), messageCallback(messageCallback) {
			MQTTClient_setCallbacks(client, this, nullptr, &MQTTConnection::arrivalCallback, nullptr);
		}
	private:
		string topicExpression = "";
		int (*messageCallback)(void* message, string &topicName);

		static int arrivalCallback(void* context, char* topicName, int,
								   MQTTClient_message* message) {
			MQTTConnection* thisObject = static_cast<MQTTConnection *>(context);
			string topic(topicName);
			if (message->payloadlen == sizeof (MQTTDataType)) {
				return thisObject->messageCallback(static_cast<MQTTDataType*>(message->payload), topic);
			}
			LogMessage(LOG_LEVEL_ERROR, "Received MQTT payload of size %d while expecting size %u on topic %s",
					   message->payloadlen, sizeof (MQTTDataType), topic.c_str());
			return -1;
		}
	};

};

#endif // MQTTCONNECTIONHANDLER_H
