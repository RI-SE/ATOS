#pragma once

#include <boost/asio.hpp>
#include <chrono>

#include "module.hpp"
#include "roschannel.hpp"
#include "osi_handler.hpp"
#include "unordered_map"
#include "objectconfig.hpp"



class OSIAdapter : public Module
{
  public:
    void initialize(const std::string& address = DEFAULT_ADDRESS,
                  const uint16_t port = DEFAULT_PORT);
    OSIAdapter();
    ~OSIAdapter();


  private:
    using TimeUnit = std::chrono::milliseconds;
    static inline std::string const DEFAULT_ADDRESS = "127.0.0.1";
    constexpr static uint16_t DEFAULT_PORT = 55555;
    constexpr static uint8_t QUALITY_OF_SERVICE = 10;
    constexpr static std::chrono::duration SEND_INTERVAL = std::chrono::milliseconds(500);

    static inline std::string const moduleName = "osi_adapter";

    void sendOSIData();
    std::vector<char> makeOSIMessage(const std::vector<OsiHandler::GlobalObjectGroundTruth_t>& osiData);
    const OsiHandler::GlobalObjectGroundTruth_t makeOSIData(ROSChannels::Monitor::message_type& monr);
    
    rclcpp::TimerBase::SharedPtr timer;

    boost::asio::ip::tcp::endpoint endpoint;
    std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
    std::shared_ptr<boost::asio::io_service> io_service;
    std::shared_ptr<boost::asio::ip::tcp::socket> socket;

    void setupTCPServer(boost::asio::ip::tcp::endpoint endpoint);
    void destroyTCPServer();
    void resetTCPServer(boost::asio::ip::tcp::endpoint endpoint);

    std::unordered_map<uint32_t,ROSChannels::Monitor::message_type> lastMonitors;
    std::unordered_map<uint32_t,TimeUnit> lastMonitorTimes;
    std::unordered_map<uint32_t,std::shared_ptr<ROSChannels::Monitor::Sub>> monrSubscribers;

    inline double linPosPrediction(const double position, const double velocity, const TimeUnit dt);
    void extrapolateMONR(ROSChannels::Monitor::message_type& monr, const TimeUnit dt);

    ROSChannels::Init::Sub initSub;
    ROSChannels::ConnectedObjectIds::Sub connectedObjectIdsSub;	//!< Publisher to report connected objects

    void onInitMessage(const ROSChannels::Init::message_type::SharedPtr msg) override;
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr msg, uint32_t id) override;
    void onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg);


    // Trajectory
    void loadObjectFiles();
    void saveTrajectories();
    void saveTrajPoints();
    std::vector<double> getDistances(const uint16_t id, const double xCar, const double yCar);
    int findNearestTrajectory(const uint16_t id, const double xCar, const double yCar);

    // Variables
    std::vector<std::unique_ptr<ObjectConfig>> objectConfigurations;
    std::vector<std::unique_ptr<const Trajectory>> trajectories;
    std::vector<std::vector<std::pair<double, double>>> trajPoints;
};
