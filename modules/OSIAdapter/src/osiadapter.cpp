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
    publisher = this->create_publisher<std_msgs::msg::String>("driver_model", QUALITY_OF_SERVICE);
    timer = this->create_wall_timer(SEND_INTERVAL, std::bind(&OSIAdapter::sendOSIData, this));
  };

  OSIAdapter::~OSIAdapter() {
    tcp.close();
  }


/**
 * @brief Intializes socket and waits for someone to connect.
 * 
 * @param address Address to connect. Default: 127.0.0.1
 * @param port Port to use. Default: 55555
 * @param debug Debug or not. Default: false
 */
void
OSIAdapter::initialize(const TCPServer::Address address, const TCPServer::Port port, bool debug) {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());

  RCLCPP_INFO(get_logger(), "Awaiting TCP connection...");
  tcp = TCPServer(address, port, debug);
  connection = tcp.await(address, port);
}


/**
 * @brief Send OSI-data to the connection.
 * 
 */
void
OSIAdapter::sendOSIData() {
  RCLCPP_INFO(get_logger(), "Sending OSI-data");

  const OsiHandler::LocalObjectGroundTruth_t osiData = OSIAdapter::makeTestOsiData();
  std::vector<char> positionOSI = OSIAdapter::makeOSIMessage(osiData);
  
  try {
    connection.send(positionOSI);
  }
  catch (SocketErrors::SocketSendError& e) {
    RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
  }
  catch (SocketErrors::DisconnectedError& e) {
    RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
  }
}

/**
 * @brief Encodes SvGt message and returns vector to use for sending message in socket. This is currently
 * used for making a test message. In the future timestamp and projStr should come from MONR-message?
 * 
 * @param osiData OSI-data
 * @return std::vector<char> Vector from SvGt encoding
 */
std::vector<char>
OSIAdapter::makeOSIMessage(const OsiHandler::LocalObjectGroundTruth_t osiData) {

  OsiHandler osi;
  std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
  auto projStr = "Test";
  auto rawData = osi.encodeSvGtMessage(osiData, timestamp, projStr, false);

  std::vector<char> vec(rawData.length());
  std::copy(rawData.begin(), rawData.end(), vec.begin());
  
  return vec;
}


/**
 * @brief This function creates test OSI-data for testing the module. Should be removed later,
 * since this data should come from MONR-messages.
 * 
 * @return const OsiHandler::LocalObjectGroundTruth_t OSI-data 
 */
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