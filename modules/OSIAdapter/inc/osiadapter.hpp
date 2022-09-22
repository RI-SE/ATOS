#pragma once

#include <future>

#include "module.hpp"

class OSIAdapter : public Module
{
  public:
    int initialize();
    OSIAdapter();

  private:
    void sendPosition();
    static inline std::string const moduleName = "osi_adapter";
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;

};