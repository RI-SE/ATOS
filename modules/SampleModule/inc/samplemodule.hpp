/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <thread>
#include <std_srvs/srv/set_bool.hpp>
#include "roschannels/test_channels.hpp"
#include "module.hpp"
#include "server.hpp"

/*!
 * \brief The SampleModule is a ros2 node that demonstrates how to use the Module class 
 */
class SampleModule : public Module{
public:
	static inline std::string const moduleName = "sample_module";
	SampleModule();
	std::vector<std::uint32_t> getObjectIds();
	bool getAborting() const { return aborting_; }

private:
	ROSChannels::Init::Sub initSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::AllClear::Sub allClearSub;
	ROSChannels::SampleModuleTestForInitResponse::Pub smOnInitResponsePub;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server;

	void onInitMessage(ROSChannels::Init::message_type::SharedPtr) override;
	void onAbortMessage(ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(ROSChannels::AllClear::message_type::SharedPtr) override;
	void OnCallbackSetBool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
								std::shared_ptr<std_srvs::srv::SetBool::Response> response);

	std::vector<std::uint32_t> objectIds;
	bool aborting_ = false;
};
