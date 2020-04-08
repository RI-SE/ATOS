#include "mqttconnectionhandler.hpp"


#define DEFAULT_SERVER_URI "tcp://127.0.0.1:1883"

MQTTConnectionHandler::MQTTConnectionHandler(const string clientID)
	: clientID(clientID), connectionOptions(MQTTClient_connectOptions_initializer) {
	int returnCode;
	// TODO: read a config file to get non-default values
	this->serverURI = DEFAULT_SERVER_URI;
	if ((returnCode = MQTTClient_create(&this->client, this->serverURI.c_str(), this->clientID.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr))
			!= MQTTCLIENT_SUCCESS) {
		string errMsg = "Failed to create MQTT client: error code " + to_string(returnCode);
		LogMessage(LOG_LEVEL_WARNING, errMsg.c_str());
		throw new MQTTError(errMsg, returnCode);
	}
}

void MQTTConnectionHandler::establishConnection() {
	int returnCode;
	if (MQTTClient_isConnected(this->client)) {
		return;
	}
	this->setMessageCallback(&MQTTTopicHandlers::handleMessage);

	if ((returnCode = MQTTClient_setCallbacks(this->client, this, nullptr, &MQTTConnectionHandler::arrivalCallback, nullptr))
			!= MQTTCLIENT_SUCCESS) {
		string errMsg = "Failed to set callbacks for MQTT client: error code " + to_string(returnCode);
		LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
		throw new MQTTError(errMsg, returnCode);
	}

	LogMessage(LOG_LEVEL_INFO, "Connecting to MQTT broker at %s", this->serverURI.c_str());
	if ((returnCode = MQTTClient_connect(this->client, &this->connectionOptions))
			!= MQTTCLIENT_SUCCESS) {
		string errMsg = "Failed to connect to MQTT broker: error code " + to_string(returnCode);
		LogMessage(LOG_LEVEL_WARNING, errMsg.c_str());
		throw new MQTTError(errMsg, returnCode);
	}
	LogMessage(LOG_LEVEL_INFO, "Successfully connected to MQTT broker");

	for (const string &subscription : MQTTTopicHandlers::getSubscriptions()) {
		LogMessage(LOG_LEVEL_INFO, "Subscribing to topic %s", subscription.c_str());
		if ((returnCode = MQTTClient_subscribe(this->client, subscription.c_str(), this->qualityOfService))
				!= MQTTCLIENT_SUCCESS) {
			string errMsg = "Failed to subscribe to topic " + subscription + ": error code " + to_string(returnCode);
			LogMessage(LOG_LEVEL_WARNING, errMsg.c_str());
		}
	}
}
