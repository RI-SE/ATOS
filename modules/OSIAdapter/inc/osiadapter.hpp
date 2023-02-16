/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "roschannel.hpp"
#include "osi_handler.hpp"
#include "unordered_map"
#include "serverfactory.hpp"
#include <chrono>

class OSIAdapter : public Module
{
  public:
    void initializeServer();
    OSIAdapter();
    ~OSIAdapter();


  private:
    using TimeUnit = std::chrono::milliseconds;
    std::string address;
    uint16_t port;
    std::string protocol;
    uint16_t frequency;
    static inline std::string const moduleName = "osi_adapter";

    void getParameters();
    void sendOSIData();
    std::vector<char> makeOSIMessage(const std::vector<OsiHandler::GlobalObjectGroundTruth_t>& osiData);
    const OsiHandler::GlobalObjectGroundTruth_t makeOSIData(ROSChannels::Monitor::message_type& monr);
    
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    
    std::unique_ptr<ServerFactory> server;
    rclcpp::TimerBase::SharedPtr timer;
    ROSChannels::ConnectedObjectIds::Sub connectedObjectIdsSub;	//!< Publisher to report connected objects

    std::unordered_map<uint32_t,ROSChannels::Monitor::message_type> lastMonitors;
    std::unordered_map<uint32_t,TimeUnit> lastMonitorTimes;
    std::unordered_map<uint32_t,std::shared_ptr<ROSChannels::Monitor::Sub>> monrSubscribers;

    void onConnectedObjectIdsMessage(const ROSChannels::ConnectedObjectIds::message_type::SharedPtr msg);
    void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr msg, uint32_t id) override;

    inline double linPosPrediction(const double position, const double velocity, const TimeUnit dt);
    void extrapolateMONR(ROSChannels::Monitor::message_type& monr, const TimeUnit dt);
};
