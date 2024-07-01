/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include "atos_interfaces/srv/get_object_ids.hpp"
#include "module.hpp"
#include <string>
#include <vector>

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
  rclcpp::Service<atos_interfaces::srv::GetObjectIds>::SharedPtr getObjectIdsService;

  void onInitMessage(ROSChannels::Init::message_type::SharedPtr) override;
  void onRequestObjectIDs(std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request> req,
                          const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response> &res);
};
