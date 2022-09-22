#pragma once

#include <future>

#include "module.hpp"

class OSIAdapter : public Module
{

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  public:
    int initialize();
    OSIAdapter();

  private:
    void sendPosition();
    static inline std::string const moduleName = "osi_adapter";
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;

};