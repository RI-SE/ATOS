#include "CotReceiver.hpp"

CotReceiver::CotReceiver() :
  Module("cot_receiver") {
	RCLCPP_INFO(get_logger(), "Hello from CotReceiver!");
}
