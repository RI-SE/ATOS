#pragma once

#include "module.hpp"
#include "MQTTClient.h"
#include <nlohmann/json.hpp>
#include <chrono>

using json = nlohmann::json;

/*!
 * \brief The MQTTBridge class is a singleton class that
 * 	handles the forwards Maestro V2X ROS msgs to a MQTT publisher client
 */

class MqttBridge : public Module
{
public:
    static inline std::string const moduleName = "mqtt_bridge";
    static void initializeModule();
    MqttBridge(MqttBridge const &) = delete;
    MqttBridge &operator=(MqttBridge const &) = delete;
    static std::shared_ptr<MqttBridge> instance();
    MQTTClient mqttClient;

    constexpr static std::chrono::milliseconds SEND_INTERVAL = std::chrono::milliseconds(5000);
private:
    std::string brokerIP;
    std::string pubClientId;
    std::string username;
    std::string password;
    std::string topic;
    std::string QoS;

    rclcpp::TimerBase::SharedPtr timer;
    MqttBridge();
    ROSChannels::Abort::Sub mqttAbortSub; //!< Subscriber to scenario abort requests
    ROSChannels::V2X::Sub v2xMsgSub;      //!< Subscriber to v2x messages requests

    void yieldMqttClient();
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    void setupConnection();
    void onV2xMsg(const ROSChannels::V2X::message_type::SharedPtr);
    json v2xToJson(const ROSChannels::V2X::message_type::SharedPtr v2x_msg);

    static std::shared_ptr<MqttBridge> me;
};