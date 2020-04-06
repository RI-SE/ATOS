#include "mqttconnection.h"


#define DEFAULT_SERVER_URI "tcp://localhost:1883"

MQTTConnectionHandler::MQTTConnectionHandler(const string clientID)
	: clientID(clientID), connectionOptions(MQTTClient_connectOptions_initializer5) {
	// TODO: read a config file to get non-default values
	this->serverURI = DEFAULT_SERVER_URI;
	MQTTClient_create(&this->client, this->serverURI.c_str(), this->clientID.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr);
}

void MQTTConnectionHandler::establishConnection() {
	int returnCode;
	LogMessage(LOG_LEVEL_INFO, "Connecting to MQTT broker at %s", this->serverURI.c_str());
	if ((returnCode = MQTTClient_connect(this->client, &this->connectionOptions))
			!= MQTTCLIENT_SUCCESS) {
		string errMsg = "Failed to connect to MQTT broker: error code " + to_string(returnCode);
		LogMessage(LOG_LEVEL_WARNING, errMsg.c_str());
		throw new MQTTError(errMsg, returnCode);
	}
	LogMessage(LOG_LEVEL_INFO, "Successfully connected to MQTT broker");
}
