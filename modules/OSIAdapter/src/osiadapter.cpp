#include <iostream>
#include <chrono>

#include "util.h"
#include "osi_handler.hpp"
#include "osiadapter.hpp"
#include "tcphandler.hpp"
#include "server.hpp"


using namespace ROSChannels;
using namespace std::chrono_literals;

OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName)
  {
    initialize();
    publisher = this->create_publisher<std_msgs::msg::String>("position", 10);
    timer = this->create_wall_timer(500ms, std::bind(&OSIAdapter::sendPositionOSI, this));  
  };


int OSIAdapter::initialize() {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());

  // make socket to driver model
  const TCPServer::Address localAddress = "127.0.0.1";
  const TCPServer::Port port = 55555;
  tcp = TCPServer(localAddress, port, false);
  auto connection = tcp.await(); // connect by running "nc 127.0.0.1 55555 in terminal"

  return 0;
}


void OSIAdapter::sendPositionOSI() {
  RCLCPP_INFO(get_logger(), "Sending position - testing!");
}

void OSIAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}