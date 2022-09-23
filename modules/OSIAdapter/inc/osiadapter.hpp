#pragma once

#include <future>

#include "module.hpp"
#include "osi_handler.hpp"
#include "server.hpp"

class OSIAdapter : public Module
{
  public:
    int initialize();
    OSIAdapter();

  private:
    void sendPositionOSI();
    static inline std::string const moduleName = "osi_adapter";
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};