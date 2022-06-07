#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <string>
#include <functional>

#include "maestro_interfaces/msg/accm.hpp"
#include "maestro_interfaces/msg/trcm.hpp"
#include "maestro_interfaces/msg/exac.hpp"
#include "maestro_interfaces/msg/object_enabled.hpp"
#include "maestro_interfaces/msg/monitor.hpp"
#include "maestro_interfaces/msg/manoeuvre_command.hpp"
#include "maestro_interfaces/msg/control_signal_percentage.hpp"
#include "maestro_interfaces/msg/trigger_event.hpp"
#include "maestro_interfaces/msg/object_id_array.hpp"

namespace ROSChannels {

template<typename T>
class BasePub {
public:
    BasePub(rclcpp::Node& node,
        const std::string& topicName,
        const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepAll()))
        : pub(node.create_publisher<T>(topicName, qos)) {}
    BasePub() = delete;
    typename rclcpp::Publisher<T>::SharedPtr pub;
    inline virtual void publish(const T& msg) { assert(pub); pub->publish(msg); };
};

template<typename T>
class BaseSub {
public:
    BaseSub(rclcpp::Node& node,
        const std::string& topicName,
        std::function<void(const typename T::SharedPtr)> callback,
        const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepAll()))
        : sub(node.create_subscription<T>(topicName, qos, callback)) {}
    BaseSub() = delete;
    typename rclcpp::Subscription<T>::SharedPtr sub;
};

namespace Init {
    const std::string topicName = "init";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Start {
    const std::string topicName = "start";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Arm {
    const std::string topicName = "arm";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Disarm {
    const std::string topicName = "disarm";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Connect {
    const std::string topicName = "connect";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Disconnect {
    const std::string topicName = "disconnect";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Stop {
    const std::string topicName = "stop";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Abort {
    const std::string topicName = "abort";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace AllClear {
    const std::string topicName = "all_clear";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace ActionConfiguration {
    const std::string topicName = "action_configuration";
    using message_type = maestro_interfaces::msg::Accm;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace TriggerConfiguration {
    const std::string topicName = "trigger_configuration";
    using message_type = maestro_interfaces::msg::Trcm;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace ExecuteAction {
    const std::string topicName = "execute_action";
    using message_type = maestro_interfaces::msg::Exac;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace GetStatus {
    const std::string topicName = "get_status";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Failure {
    const std::string topicName = "failure";
    using message_type = std_msgs::msg::UInt8;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace GetStatusResponse {
    const std::string topicName = "get_status_response";
    using message_type = std_msgs::msg::String;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace BackToStart {
    const std::string topicName = "back_to_start";
    using message_type = maestro_interfaces::msg::ManoeuvreCommand;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace BackToStartResponse {
    const std::string topicName = "back_to_start_response";
    using message_type = std_msgs::msg::Int8;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace DataDictionary {
    const std::string topicName = "data_dict";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace RemoteControlEnable {
    const std::string topicName = "remote_control_enable";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace RemoteControlDisable {
    const std::string topicName = "remote_control_disable";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace RemoteControlManoeuvre {
    const std::string topicName = "remote_control_manoeuvre";
    using message_type = maestro_interfaces::msg::ManoeuvreCommand;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace EnableObject {
    const std::string topicName = "enable_object";
    using message_type = maestro_interfaces::msg::ObjectEnabled;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Exit {
    const std::string topicName = "exit";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace Replay {
    const std::string topicName = "replay";
    using message_type = std_msgs::msg::Empty;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
    };
}

namespace ControlSignal {
    const std::string topicName = "control_signal";
    using message_type = maestro_interfaces::msg::ControlSignalPercentage;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepLast(1));

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Monitor {
    const std::string topicName = "object_monitor";
    using message_type = maestro_interfaces::msg::Monitor;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepLast(1));

    class Pub : public BasePub<message_type> {
    public:
        const uint32_t objectId;
        Pub(rclcpp::Node& node, const uint32_t id, const rclcpp::QoS& qos = defaultQoS) : 
            objectId(id),
            BasePub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, qos) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        const uint32_t objectId;
        Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : 
            objectId(id),
            BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback, qos) {}
    };
}

namespace ObjectsConnected {
    const std::string topicName = "objects_connected";
    using message_type = maestro_interfaces::msg::ObjectIdArray;
    const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace TriggerEvent {
    const std::string topicName = "trigger_event";
    using message_type = maestro_interfaces::msg::TriggerEvent;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Trajectory {
    const std::string topicName = "trajectory";
    using message_type = nav_msgs::msg::Path;

    class Pub : public BasePub<message_type> {
    public:
        const uint32_t objectId;
        Pub(rclcpp::Node& node, const uint32_t id) :
            objectId(id),
            BasePub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        const uint32_t objectId;
        Sub(rclcpp::Node& node, const uint32_t id, std::function<void(const message_type::SharedPtr)> callback) : 
            objectId(id),
            BaseSub<message_type>(node, "object_" + std::to_string(id) + "/" + topicName, callback) {}
    };
}
} // namespace ROSChannels