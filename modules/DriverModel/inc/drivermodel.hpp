#pragma once

#include <future>

#include "module.hpp"

class DriverModel : public Module
{
  public:
    int initialize();
    DriverModel();

  private:
    void sendPosition();
    static inline std::string const moduleName = "driver_model";
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;

};