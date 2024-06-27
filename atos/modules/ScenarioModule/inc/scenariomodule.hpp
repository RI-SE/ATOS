/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include "module.hpp"
#include "server.hpp"
#include <thread>

/*!
 * \brief The ScenarioModule is a ros2 node that demonstrates how to use the
 * Module class
 */
class ScenarioModule : public Module {
public:
  static inline std::string const moduleName = "scenario_module";
  ScenarioModule();

private:
  ROSChannels::Init::Sub initSub;
  void onInitMessage(ROSChannels::Init::message_type::SharedPtr) override;
};
