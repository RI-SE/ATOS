#pragma once

#include "module.hpp"

class DriverModel : public Module
{
  public:
    void initialize();
    DriverModel();

  private:
    static inline std::string const moduleName = "driver_model";
    void sendPosition();
};