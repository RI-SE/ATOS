#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

/**
 * @brief Struct containing variables related to a MQTT2ROS connection.
 */
struct Ros2MqttInterface {
  struct {
    std::string topic; ///< MQTT topic
    int qos = 0;       ///< MQTT QoS value
  } mqtt;              ///< MQTT-related variables
  struct {
    std::string topic;    ///< ROS topic
    std::string msg_type; ///< message type of subscriber
    rclcpp::GenericSubscription::SharedPtr subscriber; ///< ROS subscriber
    int queue_size = 1; ///< ROS subscriber queue size
    bool is_stale =
        false; ///< whether a new generic publisher/subscriber is required
  } ros;       ///< ROS-related variables
};