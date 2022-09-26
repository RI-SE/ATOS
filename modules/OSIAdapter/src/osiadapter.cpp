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
    connection = tcp.await(); // connect by running "nc 127.0.0.1 55555" in terminal

    publisher = this->create_publisher<std_msgs::msg::String>("position", 10);
    timer = this->create_wall_timer(500ms, std::bind(&OSIAdapter::sendPositionOSI, this));  
  };


int
OSIAdapter::initialize() {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());

  // make socket to connect to
  const TCPServer::Address localAddress = "127.0.0.1";
  const TCPServer::Port port = 55555;
  tcp = TCPServer(localAddress, port, false);

  return 0;
}


void
OSIAdapter::sendPositionOSI() {
  RCLCPP_INFO(get_logger(), "Sending position - testing!");

  const OsiHandler::LocalObjectGroundTruth_t osiData = OSIAdapter::makeTestOsiData();
  std::vector<char> positionOSI = OSIAdapter::getPositionOSI(osiData);
  connection.send(positionOSI);
}

std::vector<char>
OSIAdapter::getPositionOSI(const OsiHandler::LocalObjectGroundTruth_t osiData) {

  OsiHandler osi;
  std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
  auto projStr = "Test";
  auto rawData = osi.encodeSvGtMessage(osiData, timestamp, projStr, false);

  std::vector<char> vec(rawData.length());
  std::copy(rawData.begin(), rawData.end(), vec.begin());
  return vec;

}

const OsiHandler::LocalObjectGroundTruth_t
OSIAdapter::makeTestOsiData() {
  OsiHandler::LocalObjectGroundTruth_t osiData;

  osiData.id = 1;

  osiData.pos_m.x = 1;
  osiData.pos_m.y = 1;
  osiData.pos_m.z = 1;

  osiData.vel_m_s.lat = 1;
  osiData.vel_m_s.lon = 1;
  osiData.vel_m_s.up = 1;

  osiData.acc_m_s2.lat = 1;
  osiData.acc_m_s2.lon = 1;
  osiData.acc_m_s2.up = 1;

  osiData.orientation_rad.pitch = 1;
  osiData.orientation_rad.roll = 1;
  osiData.orientation_rad.yaw = 1;

  return osiData;
}

void
OSIAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}