/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "roschannels/v2xchannel.hpp"
#include "roschannels/statechange.hpp"
#include "mqttclient.hpp"
#include <nlohmann/json.hpp>
#include <chrono>

using json = nlohmann::json;

/*!
 * \brief The MQTTBridge node forwards ATOS V2X ROS msgs to a MQTTClient publisher 
 */

class MqttBridge : public Module
{
public:
	MqttBridge();
    void initialize();

private:
    std::shared_ptr<MQTTClient> mqttClient;
    static inline std::string const moduleName = "mqtt_bridge";
    constexpr static std::chrono::milliseconds SEND_INTERVAL = std::chrono::milliseconds(5000);
    std::string brokerIP;
    int port;
    std::string clientId;
    std::string username;
    std::string password;
    std::string topic;
    int QoS;

    ROSChannels::V2X::Sub v2xMsgSub;      //!< Subscriber to v2x messages requests
    ROSChannels::StateChange::Sub obcStateChangeSub; //!< Subscriber to object state change requests

    void setupConnection();
    void onV2xMsg(const ROSChannels::V2X::message_type::SharedPtr);
    void onObcStateChangeMsg(const ROSChannels::StateChange::message_type::SharedPtr);

    template <typename T>
    void onMessage(T msg, std::string mqtt_topic, std::function<json(T)> convertFunc);

    static json v2xToJson(const ROSChannels::V2X::message_type::SharedPtr v2x_msg);
    static json obcStateChangeToJson(const ROSChannels::StateChange::message_type::SharedPtr obc_msg);

};