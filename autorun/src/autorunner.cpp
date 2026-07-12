#include <cstdio>
#include <unistd.h>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

class Autorunner : public rclcpp::Node
{
  public:
    Autorunner()
    : Node("autorunner")
	{
		init_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/init", 10);
		connect_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/connect", 10);
		arm_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/arm", 10);
		start_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/start", 10);
		abort_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/abort", 10);
		all_clear_pub = this->create_publisher<std_msgs::msg::Empty>("/atos/all_clear", 10);
		run_experiment(20);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr init_pub;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr connect_pub;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_pub;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_pub;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr abort_pub;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr all_clear_pub;
    void run_experiment(int n)
    {
		RCLCPP_INFO(this->get_logger(), "Running %d experiments", n);
		std_msgs::msg::Empty message;
		RCLCPP_INFO(this->get_logger(), "Sending init to ATOS");
		init_pub->publish(message);
		sleep(1);
		init_pub->publish(message);
		sleep(2);
		RCLCPP_INFO(this->get_logger(), "Sending connect to ATOS");
		connect_pub->publish(message);
		sleep(2);

		while (n > 0)
		{
			RCLCPP_INFO(this->get_logger(), "Experiment %d", n);
			RCLCPP_INFO(this->get_logger(), "Sending arm to ATOS");
			arm_pub->publish(message);
			sleep(2);
			RCLCPP_INFO(this->get_logger(), "Sending start to ATOS");
			start_pub->publish(message);
			sleep(35);
			RCLCPP_INFO(this->get_logger(), "Sending abort to ATOS");
			abort_pub->publish(message);
			sleep(2);
			RCLCPP_INFO(this->get_logger(), "Sending all_clear to ATOS");
			all_clear_pub->publish(message);
			sleep(2);
			n--;
		}
    }
};


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Autorunner>());
	rclcpp::shutdown();
    return 0;
}
