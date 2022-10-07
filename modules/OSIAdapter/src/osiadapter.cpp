#include <chrono>
#include <boost/asio.hpp>
#include <algorithm>
#include <filesystem>
#include <cmath>

#include "osi_handler.hpp"
#include "osiadapter.hpp"
#include "trajectory.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace boost::asio;
using std::placeholders::_1;

OSIAdapter::OSIAdapter() :
  Module(OSIAdapter::moduleName),
  initSub(*this, std::bind(&OSIAdapter::onInitMessage, this, _1)),
  connectedObjectIdsSub(*this, std::bind(&OSIAdapter::onConnectedObjectIdsMessage, this, _1))
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
 */
void
OSIAdapter::initialize(const std::string& address, const uint16_t port) {
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
  
  // Extrapolate monr data and create a sensorView containing the objects
  std::for_each(lastMonitors.begin(),lastMonitors.end(),[&](auto pair){ OSIAdapter::extrapolateMONR(pair.second,lastMonitorTimes.at(pair.first));});
  std::vector<OsiHandler::GlobalObjectGroundTruth_t> sensorView(lastMonitors.size());
  std::transform(lastMonitors.begin(),lastMonitors.end(), sensorView.begin(), [&](auto pair) {return OSIAdapter::makeOSIData(pair.second);});
  
  // Serialize the sensorView
  std::vector<char> positionOSI = OSIAdapter::makeOSIMessage(sensorView);
  
  // Write the sensorView to the socket
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
OSIAdapter::makeOSIMessage(const std::vector<OsiHandler::GlobalObjectGroundTruth_t>& osiData) {

  OsiHandler osi;
  std::chrono::system_clock::time_point timestamp = std::chrono::system_clock::now();
  auto projStr = "";
  auto rawData = osi.encodeSvGtMessage(osiData, timestamp, projStr, false);

  std::vector<char> vec(rawData.length());
  std::copy(rawData.begin(), rawData.end(), vec.begin());
  
  return vec;
}


/**
 * @brief This method creates test OSI-data for testing the module. Should be removed later,
 * since this data should come from MONR-messages.
 * 
 * @return const OsiHandler::GlobalObjectGroundTruth_t OSI-data 
 */
const OsiHandler::GlobalObjectGroundTruth_t
OSIAdapter::makeOSIData(ROSChannels::Monitor::message_type& monr) {
  
  OsiHandler::GlobalObjectGroundTruth_t osiData;
  osiData.id = monr.maestro_header.object_id;

  osiData.pos_m.x = monr.pose.pose.position.x;
  osiData.pos_m.y = monr.pose.pose.position.y;
  osiData.pos_m.z = monr.pose.pose.position.z;

  osiData.acc_m_s2.x = monr.acceleration.accel.linear.x;
  osiData.acc_m_s2.y = monr.acceleration.accel.linear.y;
  osiData.acc_m_s2.z = monr.acceleration.accel.linear.z;

  osiData.vel_m_s.x = monr.velocity.twist.linear.x;
  osiData.vel_m_s.y = monr.velocity.twist.linear.y;
  osiData.vel_m_s.z = monr.velocity.twist.linear.z;

  return osiData;
}


double
OSIAdapter::linPosPrediction(const double position, const double velocity, const TimeUnit dt) {
  return position + velocity * duration<double>(dt).count();
}


void
OSIAdapter::extrapolateMONR(ROSChannels::Monitor::message_type& monr,  const TimeUnit dt) {

  monr.pose.pose.position.x = linPosPrediction(monr.pose.pose.position.x, monr.velocity.twist.linear.x, dt);
  monr.pose.pose.position.y = linPosPrediction(monr.pose.pose.position.y, monr.velocity.twist.linear.y, dt);
  monr.pose.pose.position.z = linPosPrediction(monr.pose.pose.position.z, monr.velocity.twist.linear.z, dt);
}

/**
 * @brief Finds the distances from the car position (x,y) to all points in the trajectory
 * 
 * @param id Object ID
 * @param xCar x-position of car
 * @param yCar y-position of car
 * @return std::vector<double> Vector of distances from (xCar, yCar) to all points in the trajectory
 */
std::vector<double>
OSIAdapter::getDistances(const uint16_t id, const double xCar, const double yCar) {

  std::vector<std::pair<double,double>> trajectoryPoints = trajPoints[id];
  std::vector<double> distances;

  for (auto points : trajectoryPoints) {
    auto xPoints = points.first;
    auto yPoints = points.second;

    auto xDiff = xCar - xPoints;
    auto yDiff = yCar - yPoints;

    auto distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));
    distances.emplace_back(distance);
  }

  return distances;
}

/**
 * @brief Finds the index of the trajectory that is the most close a point
 * 
 * @param id Object ID
 * @param xCar x-position of car
 * @param yCar y-position of car
 * @return int Index of the trajectory point most close to (xCar, yCar)
 */
int
OSIAdapter::findNearestTrajectory(const uint16_t id, const double xCar, const double yCar) {

  auto distances = getDistances(id, xCar, yCar);
  int minIndex = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
  return minIndex;
}

void
OSIAdapter::saveTrajPoints() {

  for (auto& traj : trajectories) {
    std::vector<std::pair<double,double>> trajPointsForId;
    for (auto& points : traj->points) {
      auto x = points.getXCoord();
      auto y = points.getYCoord();
      trajPointsForId.emplace_back(std::make_pair(x,y));
    }
    trajPoints.emplace_back(trajPointsForId);
  }
}


void
OSIAdapter::loadObjectFiles() {
  char path[MAX_FILE_PATH];
	std::vector<std::invalid_argument> errors;

	UtilGetObjectDirectoryPath(path, sizeof (path));
	std::filesystem::path objectDir(path);
	if (!std::filesystem::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	for (const auto& entry : std::filesystem::directory_iterator(objectDir)) {
		if (std::filesystem::is_regular_file(entry.status())) {
			ObjectConfig conf;
            conf.parseConfigurationFile(entry.path());
            objectConfigurations.push_back(std::make_unique<ObjectConfig>(conf));
		}
	} 
    RCLCPP_INFO(get_logger(), "Loaded %d object configurations", objectConfigurations.size());
}


void
OSIAdapter::saveTrajectories() {

  for (auto& config : objectConfigurations) {
    auto traj = config->getTrajectory();
    trajectories.emplace_back(std::make_unique<Trajectory>(traj));
  }
}


void
OSIAdapter::onInitMessage(const ROSChannels::Init::message_type::SharedPtr msg) {
  
  RCLCPP_INFO(get_logger(), "Loading object configuration...");
  loadObjectFiles();
  saveTrajectories(); 
  saveTrajPoints();
  findNearestTrajectory(0, 0, 0);
}



void OSIAdapter::onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg) {
  for (uint32_t id : msg->ids) {
    if (monrSubscribers.find(id) == monrSubscribers.end()){
      monrSubscribers[id] = std::make_shared<ROSChannels::Monitor::Sub>(*this, id, std::bind(&OSIAdapter::onMonitorMessage, this, _1, id));
    }
  }
}

void OSIAdapter::onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr msg, uint32_t id) {
  if (lastMonitors.find(id) == lastMonitors.end()){
    // Do not extrapolate first message
    lastMonitorTimes[id] = TimeUnit(0);
  }
  else{
    // Otherwise take diff between last two messages
    auto newTime = seconds(msg->maestro_header.header.stamp.sec) + nanoseconds(msg->maestro_header.header.stamp.nanosec);
    auto oldTime = seconds(lastMonitors[id].maestro_header.header.stamp.sec) + nanoseconds(lastMonitors[id].maestro_header.header.stamp.nanosec);
    lastMonitorTimes[id] = duration_cast<TimeUnit>(newTime-oldTime);
  }
  lastMonitors[id] = *msg;
}


void OSIAdapter::onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) {
  RCLCPP_INFO(get_logger(), "Received abort message");
}