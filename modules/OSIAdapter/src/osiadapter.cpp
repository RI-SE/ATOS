#include <chrono>
#include <boost/asio.hpp>

#include "osi_handler.hpp"
#include "osiadapter.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace boost::asio;
using std::placeholders::_1;

OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName),
  connectedObjectIdsSub(*this,std::bind(&OSIAdapter::onConnectedObjectIdsMessage, this, _1))
  {
    initialize();
    timer = this->create_wall_timer(SEND_INTERVAL, std::bind(&OSIAdapter::sendOSIData, this));
  };

  OSIAdapter::~OSIAdapter() {
    destroyTCPServer();
  }

void OSIAdapter::destroyTCPServer() {
  RCLCPP_DEBUG(this->get_logger(), "Destroying TCP Server");
  if (acceptor) {
    acceptor->close();
    acceptor.reset();
  }
  if (socket) {
    socket->close();
    socket.reset();
  }
}

void OSIAdapter::setupTCPServer(ip::tcp::endpoint endpoint){
  acceptor = std::make_shared<ip::tcp::acceptor>(*io_service, endpoint);
  socket = std::make_shared<ip::tcp::socket>(*io_service);
  boost::system::error_code ignored_error;
  RCLCPP_DEBUG(get_logger(), "Waiting for connection on %s:%d", endpoint.address().to_string().c_str(), endpoint.port());
  acceptor->accept(*socket,ignored_error);
  while (ignored_error) {
    RCLCPP_DEBUG(get_logger(), "Failed to accept the connection from %s:%d, retrying..", endpoint.address().to_string().c_str(), endpoint.port());
    std::this_thread::sleep_for(500ms);
    acceptor->accept(*socket,ignored_error);
  }
  RCLCPP_INFO(get_logger(), "Connection established with %s:%d", socket->remote_endpoint().address().to_string().c_str(), socket->remote_endpoint().port());
}

void OSIAdapter::resetTCPServer(ip::tcp::endpoint endpoint) {
  destroyTCPServer();
  setupTCPServer(endpoint);
}

/**
 * @brief Intializes socket and waits for someone to connect.
 * 
 * @param address Address to connect. Default: 127.0.0.1
 * @param port Port to use. Default: 55555
 * @param debug Debug or not. Default: false
 */
void
OSIAdapter::initialize(const std::string& address, const uint16_t port, bool debug) {
  RCLCPP_INFO(get_logger(), "%s task running with PID %d", get_name(), getpid());
  endpoint = ip::tcp::endpoint(ip::make_address_v4(address), port);
  io_service = std::make_shared<boost::asio::io_service>();
  setupTCPServer(endpoint);
}


/**
 * @brief Send OSI-data to the receiver at other end of socket.
 * writing to a socket that has been closed by the other end will result in error code 32 (broken pipe),
 * therefore, the connection is reset and OSIAdapter waits until a client connects again.
 * 
 */
void
OSIAdapter::sendOSIData() {
  boost::system::error_code ignored_error;
  // TODO: Replace makeTestOSIData with extrapolation + creation of synchronized OSI message.
  const OsiHandler::GlobalObjectGroundTruth_t osiData = OSIAdapter::makeTestOSIData();
  std::vector<char> positionOSI = OSIAdapter::makeOSIMessage(osiData);
  
  write(*socket, buffer(positionOSI), ignored_error);
  if (ignored_error.value() != 0){ // Error while sending to client
    if (ignored_error.value() == error::broken_pipe){
      RCLCPP_INFO(get_logger(), "Client has disconnected.");
    }
    else{
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ignored_error.message().c_str());
    }
    // Either way, reset the socket and go back to listening
    resetTCPServer(endpoint);
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
OSIAdapter::makeOSIMessage(const OsiHandler::GlobalObjectGroundTruth_t osiData) {

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
const OsiHandler::GlobalObjectGroundTruth_t
OSIAdapter::makeTestOSIData() {
  
  OsiHandler::GlobalObjectGroundTruth_t osiData;

  osiData.id = 1;

  osiData.pos_m.x = 1.0;
  osiData.pos_m.y = 2.0;
  osiData.pos_m.z = 3.0;

  osiData.orientation_rad.pitch = 4.0;
  osiData.orientation_rad.roll = 5.0;
  osiData.orientation_rad.yaw = 6.0;



  return osiData;
}


double
OSIAdapter::calculatePosition(double position, double velocity, double deltaT) {
  return position + velocity * deltaT;
}


void
OSIAdapter::extrapolateMONR(const uint32_t id,  const double deltaT) {
  auto monrMessage = lastMonitors[id];

  auto positionX = monrMessage.pose.pose.position.x;
  auto positionY = monrMessage.pose.pose.position.y;
  auto positionZ = monrMessage.pose.pose.position.z;

  auto velocityX = monrMessage.velocity.twist.linear.x;
  auto velocityY = monrMessage.velocity.twist.linear.y;
  auto velocityZ = monrMessage.velocity.twist.linear.z;

  auto newPositionX = calculatePosition(positionX, velocityX, deltaT);
  auto newPositionY = calculatePosition(positionY, velocityY, deltaT);
  auto newPositionZ = calculatePosition(positionZ, velocityZ, deltaT);

  monrMessage.pose.pose.position.x = newPositionX;
  monrMessage.pose.pose.position.y = newPositionY;
  monrMessage.pose.pose.position.z = newPositionZ;

  RCLCPP_INFO(get_logger(), "Old position x: %lf", positionX);
  RCLCPP_INFO(get_logger(), "New position x: %lf", monrMessage.pose.pose.position.x);
}


void OSIAdapter::onConnectedObjectIdsMessage(const ConnectedObjectIds::message_type::SharedPtr msg) {
  for (uint32_t id : msg->ids) {
    if (monrSubscribers.find(id) == monrSubscribers.end()){
      monrSubscribers[id] = std::make_shared<Monitor::Sub>(*this, id, std::bind(&OSIAdapter::onMonitorMessage, this, _1, id));
    }
  }
}

void OSIAdapter::onMonitorMessage(const Monitor::message_type::SharedPtr msg, uint32_t id) {
  this->lastMonitors[id] = *msg;
  extrapolateMONR(id, 0.5);
}


void OSIAdapter::onAbortMessage(const Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}