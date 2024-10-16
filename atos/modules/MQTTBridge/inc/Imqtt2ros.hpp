#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

/**
 * @brief Struct containing variables related to a MQTT2ROS connection.
 * @details This struct is intended to be used in a map structure where the key
 * is the MQTT topic name.
 */
struct Mqtt2RosInterface {
  struct {
    int qos = 0; ///< MQTT QoS value
  } mqtt;        ///< MQTT-related variables
  struct {
    std::string topic;    ///< ROS topic
    std::string msg_type; ///< message type of publisher
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
        publisher;      ///< ROS publisher
    int queue_size = 1; ///< ROS publisher queue size
    bool is_stale =
        false; ///< whether a new generic publisher/subscriber is required
  } ros;       ///< ROS-related variables
};