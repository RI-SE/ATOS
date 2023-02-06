#pragma once

#include <boost/asio.hpp>

#include "module.hpp"
#include "roschannel.hpp"
#include "osi_handler.hpp"
#include "unordered_map"
#include "serverfactory.hpp"
#include <chrono>

class OSIAdapter : public Module
{
  public:
    void initializeServer(const std::string& address = DEFAULT_ADDRESS,
                  const uint16_t port = DEFAULT_PORT);
    OSIAdapter();
    ~OSIAdapter();


  private:
    using TimeUnit = std::chrono::milliseconds;
    static inline std::string const DEFAULT_ADDRESS = "0.0.0.0";
    constexpr static uint16_t DEFAULT_PORT = 55555;
    constexpr static uint8_t QUALITY_OF_SERVICE = 10;
    constexpr static std::chrono::duration SEND_INTERVAL = std::chrono::milliseconds(50);

    static inline std::string const moduleName = "osi_adapter";

    void sendOSIData();
    std::vector<char> makeOSIMessage(const std::vector<OsiHandler::GlobalObjectGroundTruth_t>& osiData);
    const OsiHandler::GlobalObjectGroundTruth_t makeOSIData(ROSChannels::Monitor::message_type& monr);
    
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    
    std::unique_ptr<Server> server;

    ROSChannels::ConnectedObjectIds::Sub connectedObjectIdsSub;	//!< Publisher to report connected objects
    rclcpp::TimerBase::SharedPtr timer;

    std::unordered_map<uint32_t,ROSChannels::Monitor::message_type> lastMonitors;
    std::unordered_map<uint32_t,TimeUnit> lastMonitorTimes;
    std::unordered_map<uint32_t,std::shared_ptr<ROSChannels::Monitor::Sub>> monrSubscribers;

    void onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg);
    void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr msg, uint32_t id) override;

    inline double linPosPrediction(const double position, const double velocity, const TimeUnit dt);
    void extrapolateMONR(ROSChannels::Monitor::message_type& monr, const TimeUnit dt);
};
