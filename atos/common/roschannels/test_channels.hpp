#include "roschannel.hpp"
#include "std_msgs/msg/empty.hpp"


namespace ROSChannels {
    namespace SampleModuleTestForInitResponse {
        const std::string topicName = "sample_module_test_for_init_response";
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
}