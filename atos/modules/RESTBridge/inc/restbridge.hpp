/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include "module.hpp"
#include "roschannels/customcommandaction.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/*!
 * \brief The RESTBridge is a ros2 node that demonstrates how to use the Module
 * class
 */
class RESTBridge : public Module {
public:
  static inline std::string const moduleName = "rest_bridge";
  RESTBridge();
  ~RESTBridge();

protected:
  void onCustomCommandAction(
      const atos_interfaces::msg::CustomCommandAction::SharedPtr msg);

private:
  ROSChannels::CustomCommandAction::Sub
      customCommandActionMsgSub; //!< Subscriber to icdc messages requests

  json parseICDCCommand(std::string &msg);
  void sendRESTMessages(const std::string &endpoint, const json &data);

  CURL *curl_handle;
};
