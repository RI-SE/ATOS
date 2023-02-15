#pragma once

#include "module.hpp"
#include "MQTTClient.h"
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
    MQTTClient mqttClient;

private:
    static inline std::string const moduleName = "mqtt_bridge";
    constexpr static std::chrono::milliseconds SEND_INTERVAL = std::chrono::milliseconds(5000);
    std::string brokerIP;
    std::string pubClientId;
    std::string username;
    std::string password;
    std::string topic;
    std::string QoS;

    rclcpp::TimerBase::SharedPtr timer;
    ROSChannels::V2X::Sub v2xMsgSub;      //!< Subscriber to v2x messages requests

    void yieldMqttClient();
    void setupConnection();
    void onV2xMsg(const ROSChannels::V2X::message_type::SharedPtr);
    json v2xToJson(const ROSChannels::V2X::message_type::SharedPtr v2x_msg);
};