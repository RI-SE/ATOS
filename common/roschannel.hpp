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

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Start {
    const std::string topicName = "start";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Arm {
    const std::string topicName = "arm";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Disarm {
    const std::string topicName = "disarm";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Connect {
    const std::string topicName = "connect";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Disconnect {
    const std::string topicName = "disconnect";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Stop {
    const std::string topicName = "stop";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Abort {
    const std::string topicName = "abort";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace AllClear {
    const std::string topicName = "all_clear";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace ActionConfiguration {
    const std::string topicName = "action_configuration";

    class Pub : public BasePub<maestro_interfaces::msg::Accm> {
    public:
        Pub(rclcpp::Node& node) : BasePub<maestro_interfaces::msg::Accm>(node, topicName) {}
    };

    class Sub : public BaseSub<maestro_interfaces::msg::Accm> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const maestro_interfaces::msg::Accm::SharedPtr)> callback) : BaseSub<maestro_interfaces::msg::Accm>(node, topicName, callback) {}
    };
}

namespace ExecuteAction {
    const std::string topicName = "execute_action";

    class Pub : public BasePub<maestro_interfaces::msg::Exac> {
    public:
        Pub(rclcpp::Node& node) : BasePub<maestro_interfaces::msg::Exac>(node, topicName) {}
    };

    class Sub : public BaseSub<maestro_interfaces::msg::Exac> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const maestro_interfaces::msg::Exac::SharedPtr)> callback) : BaseSub<maestro_interfaces::msg::Exac>(node, topicName, callback) {}
    };
}

namespace GetStatus {
    const std::string topicName = "get_status";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Failure {
    const std::string topicName = "failure";

    class Pub : public BasePub<std_msgs::msg::UInt8> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::UInt8>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::UInt8> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::UInt8::SharedPtr)> callback) : BaseSub<std_msgs::msg::UInt8>(node, topicName, callback) {}
    };
}

namespace GetStatusResponse {
    const std::string topicName = "get_status_response";

    class Pub : public BasePub<std_msgs::msg::String> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::String>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::String> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::String::SharedPtr)> callback) : BaseSub<std_msgs::msg::String>(node, topicName, callback) {}
    };
}

namespace BackToStart {
    const std::string topicName = "back_to_start";

    class Pub : public BasePub<maestro_interfaces::msg::ManoeuvreCommand> {
    public:
        Pub(rclcpp::Node& node) : BasePub<maestro_interfaces::msg::ManoeuvreCommand>(node, topicName) {}
    };

    class Sub : public BaseSub<maestro_interfaces::msg::ManoeuvreCommand> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const maestro_interfaces::msg::ManoeuvreCommand::SharedPtr)> callback) : BaseSub<maestro_interfaces::msg::ManoeuvreCommand>(node, topicName, callback) {}
    };
}

namespace BackToStartResponse {
    const std::string topicName = "back_to_start_response";

    class Pub : public BasePub<std_msgs::msg::String> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::String>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::String> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::String::SharedPtr)> callback) : BaseSub<std_msgs::msg::String>(node, topicName, callback) {}
    };
}

namespace DataDictionary {
    const std::string topicName = "data_dict";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace RemoteControlEnable {
    const std::string topicName = "remote_control_enable";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace RemoteControlDisable {
    const std::string topicName = "remote_control_disable";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace RemoteControlManoeuvre {
    const std::string topicName = "remote_control_manoeuvre";

    class Pub : public BasePub<maestro_interfaces::msg::ManoeuvreCommand> {
    public:
        Pub(rclcpp::Node& node) : BasePub<maestro_interfaces::msg::ManoeuvreCommand>(node, topicName) {}
    };

    class Sub : public BaseSub<maestro_interfaces::msg::ManoeuvreCommand> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const maestro_interfaces::msg::ManoeuvreCommand::SharedPtr)> callback) : BaseSub<maestro_interfaces::msg::ManoeuvreCommand>(node, topicName, callback) {}
    };
}

namespace EnableObject {
    const std::string topicName = "enable_object";

    class Pub : public BasePub<maestro_interfaces::msg::ObjectEnabled> {
    public:
        Pub(rclcpp::Node& node) : BasePub<maestro_interfaces::msg::ObjectEnabled>(node, topicName) {}
    };

    class Sub : public BaseSub<maestro_interfaces::msg::ObjectEnabled> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const maestro_interfaces::msg::ObjectEnabled::SharedPtr)> callback) : BaseSub<maestro_interfaces::msg::ObjectEnabled>(node, topicName, callback) {}
    };
}

namespace Exit {
    const std::string topicName = "exit";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

namespace Replay {
    const std::string topicName = "replay";

    class Pub : public BasePub<std_msgs::msg::Empty> {
    public:
        Pub(rclcpp::Node& node) : BasePub<std_msgs::msg::Empty>(node, topicName) {}
    };

    class Sub : public BaseSub<std_msgs::msg::Empty> {
    public:
        Sub(rclcpp::Node& node, std::function<void(const std_msgs::msg::Empty::SharedPtr)> callback) : BaseSub<std_msgs::msg::Empty>(node, topicName, callback) {}
    };
}

} // namespace ROSChannels