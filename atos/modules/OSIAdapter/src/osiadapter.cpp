/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <chrono>
#include <boost/asio.hpp>
#include <algorithm>

#include "osi_handler.hpp"
#include "osiadapter.hpp"


using namespace ROSChannels;
using namespace std::chrono;
using std::placeholders::_1;


/**
 * @brief Module for sending OSI-data from MONR-messages over a server.
 * 
 */
OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName),
  connectedObjectIdsSub(*this,std::bind(&OSIAdapter::onConnectedObjectIdsMessage, this, _1))
  {
    getParameters();
    initializeServer();
    
    std::chrono::duration sendInterval = std::chrono::milliseconds(1000 / frequency);
    timer = this->create_wall_timer(std::chrono::milliseconds(sendInterval), std::bind(&OSIAdapter::sendOSIData, this));
  };


/**
 * @brief Destroy the OSIAdapter::OSIAdapter object.
 * 
 */
OSIAdapter::~OSIAdapter() {
    server->destroyServer();
  }


/**
 * @brief Get parameters from parameter file.
 * 
 */
void OSIAdapter::getParameters() {
  declare_parameter("address","0.0.0.0");
  declare_parameter("port",10);
  declare_parameter("protocol","tcp");
  declare_parameter("frequency",10);

  get_parameter("address", address);
  get_parameter("port", port);
  get_parameter("protocol", protocol);
  get_parameter("frequency", frequency);

}


/**
 * @brief Initializes a server, using either TCP or UDP.
 * 
 * @param address IP-address
 * @param port Port
 */
void OSIAdapter::initializeServer() {
  server = std::make_unique<ServerFactory>(address, port, protocol);
  server->createServer();
  server->setupServer();
}


/**
 * @brief Send OSI-data to the receiver at other end of socket.
 * writing to a socket that has been closed by the other end will result in error code 32 (broken pipe),
 * therefore, the connection is reset and OSIAdapter waits until a client connects again.
 * 
 */
void OSIAdapter::sendOSIData() {
  // Extrapolate monr data and create a sensorView containing the objects
  std::for_each(lastMonitors.begin(),lastMonitors.end(),[&](auto pair){ OSIAdapter::extrapolateMONR(pair.second,lastMonitorTimes.at(pair.first));});
  std::vector<OsiHandler::GlobalObjectGroundTruth_t> sensorView(lastMonitors.size());
  std::transform(lastMonitors.begin(),lastMonitors.end(), sensorView.begin(), [&](auto pair) {return OSIAdapter::makeOSIData(pair.second);});
  
  std::vector<char> data = OSIAdapter::makeOSIMessage(sensorView);
  
  try {
    server->sendData(data);
  }
  catch (const SocketErrors::DisconnectedError& e) {
    RCLCPP_INFO(get_logger(), "Client disconnected: %s. Restarting server", e.what());
    server->resetServer();
  }
  catch (const SocketErrors::SocketSendError& e) {
    RCLCPP_INFO(get_logger(), "Client disconnected: %s. Restarting server" , e.what());
    server->resetServer();
  }
}


/**
 * @brief Encodes SvGt message and returns vector to use for sending message in socket.
 * 
 * @param osiData OSI-data
 * @return std::vector<char> Vector from SvGt encoding
 */
std::vector<char> OSIAdapter::makeOSIMessage(const std::vector<OsiHandler::GlobalObjectGroundTruth_t>& osiData) {

  OsiHandler osi;
  std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
  auto projStr = "";
  auto rawData = osi.encodeSvGtMessage(osiData, timestamp, projStr, false);

  std::vector<char> vec(rawData.length());
  std::copy(rawData.begin(), rawData.end(), vec.begin());
  
  return vec;
}


/**
 * @brief Create OSI-data from MONR.
 * 
 * @return const OsiHandler::GlobalObjectGroundTruth_t OSI-data 
 */
const OsiHandler::GlobalObjectGroundTruth_t OSIAdapter::makeOSIData(ROSChannels::Monitor::message_type& monr) {
  
  OsiHandler::GlobalObjectGroundTruth_t osiData;
  osiData.id = monr.atos_header.object_id;

  osiData.pos_m.x = monr.pose.pose.position.x;
  osiData.pos_m.y = monr.pose.pose.position.y;
  osiData.pos_m.z = monr.pose.pose.position.z;

  osiData.acc_m_s2.x = monr.acceleration.accel.linear.x;
  osiData.acc_m_s2.y = monr.acceleration.accel.linear.y;
  osiData.acc_m_s2.z = monr.acceleration.accel.linear.z;

  osiData.vel_m_s.x = monr.velocity.twist.linear.x;
  osiData.vel_m_s.y = monr.velocity.twist.linear.y;
  osiData.vel_m_s.z = monr.velocity.twist.linear.z;

  osiData.orientation_rad.yaw = 2 * acos(monr.pose.pose.orientation.w);

  return osiData;

}


/**
 * @brief Linear extrapolation for a position.
 * 
 * @param position The current position
 * @param velocity The current velocity
 * @param dt Time forward
 * @return double extrapolated position, in dt milliseconds
 */
double OSIAdapter::linPosPrediction(const double position, const double velocity, const TimeUnit dt) {
  return position + velocity * duration<double>(dt).count();
}


/**
 * @brief Linear extrapolation for a MONR-message.
 * 
 * @param monr MONR-message to extrapolate
 * @param dt Time forward
 */
void OSIAdapter::extrapolateMONR(Monitor::message_type& monr,  const TimeUnit dt) {
  monr.pose.pose.position.x = linPosPrediction(monr.pose.pose.position.x, monr.velocity.twist.linear.x, dt);
  monr.pose.pose.position.y = linPosPrediction(monr.pose.pose.position.y, monr.velocity.twist.linear.y, dt);
  monr.pose.pose.position.z = linPosPrediction(monr.pose.pose.position.z, monr.velocity.twist.linear.z, dt);
}


void OSIAdapter::onConnectedObjectIdsMessage(const ConnectedObjectIds::message_type::SharedPtr msg) {
  for (uint32_t id : msg->ids) {
    if (monrSubscribers.find(id) == monrSubscribers.end()){
      monrSubscribers[id] = std::make_shared<Monitor::Sub>(*this, id, std::bind(&OSIAdapter::onMonitorMessage, this, _1, id));
    }
  }
}


void OSIAdapter::onMonitorMessage(const Monitor::message_type::SharedPtr msg, uint32_t id) {
  if (lastMonitors.find(id) == lastMonitors.end()){ // Do not extrapolate first message
    lastMonitorTimes[id] = TimeUnit(0);
  }
  else{ // Otherwise take diff between last two messages
    auto newTime = seconds(msg->atos_header.header.stamp.sec) + nanoseconds(msg->atos_header.header.stamp.nanosec);
    auto oldTime = seconds(lastMonitors[id].atos_header.header.stamp.sec) + nanoseconds(lastMonitors[id].atos_header.header.stamp.nanosec);
    lastMonitorTimes[id] = duration_cast<TimeUnit>(newTime-oldTime);
  }
  lastMonitors[id] = *msg;
}
