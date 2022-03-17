#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <functional>

#include "maestro_interfaces/msg/accm.hpp"
#include "maestro_interfaces/msg/exac.hpp"
#include "maestro_interfaces/msg/object_enabled.hpp"
#include "maestro_interfaces/msg/monitor.hpp"
#include "maestro_interfaces/msg/manoeuvre_command.hpp"

namespace ROSChannels {

template<typename T>
class BasePub {
public:
    BasePub(rclcpp::Node& node,
        const std::string& topicName,
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll()))
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
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepAll()))
        : sub(node.create_subscription<T>(topicName, qos, callback)) {}
    BaseSub() = delete;
    typename rclcpp::Subscription<T>::SharedPtr sub;
};

namespace Init {
    const std::string topicName = "init";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Start {
    const std::string topicName = "start";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Arm {
    const std::string topicName = "arm";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Disarm {
    const std::string topicName = "disarm";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Connect {
    const std::string topicName = "connect";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Disconnect {
    const std::string topicName = "disconnect";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Stop {
    const std::string topicName = "stop";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Abort {
    const std::string topicName = "abort";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace AllClear {
    const std::string topicName = "all_clear";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace ActionConfiguration {
    const std::string topicName = "action_configuration";
    using message_type = maestro_interfaces::msg::Accm;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace ExecuteAction {
    const std::string topicName = "execute_action";
    using message_type = maestro_interfaces::msg::Exac;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace GetStatus {
    const std::string topicName = "get_status";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Failure {
    const std::string topicName = "failure";
    using message_type = std_msgs::msg::UInt8;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace GetStatusResponse {
    const std::string topicName = "get_status_response";
    using message_type = std_msgs::msg::String;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace BackToStart {
    const std::string topicName = "back_to_start";
    using message_type = maestro_interfaces::msg::ManoeuvreCommand;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace BackToStartResponse {
    const std::string topicName = "back_to_start_response";
    using message_type = std_msgs::msg::String;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace DataDictionary {
    const std::string topicName = "data_dict";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace RemoteControlEnable {
    const std::string topicName = "remote_control_enable";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace RemoteControlDisable {
    const std::string topicName = "remote_control_disable";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace RemoteControlManoeuvre {
    const std::string topicName = "remote_control_manoeuvre";
    using message_type = maestro_interfaces::msg::ManoeuvreCommand;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace EnableObject {
    const std::string topicName = "enable_object";
    using message_type = maestro_interfaces::msg::ObjectEnabled;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Exit {
    const std::string topicName = "exit";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

namespace Replay {
    const std::string topicName = "replay";
    using message_type = std_msgs::msg::Empty;

    class Pub : public BasePub<message_type> {
    public:
        Pub(rclcpp::Node& node) : BasePub<message_type>(node, topicName) {}
    };

    class Sub : public BaseSub<message_type> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback) : BaseSub<message_type>(node, topicName, callback) {}
    };
}

} // namespace ROSChannels