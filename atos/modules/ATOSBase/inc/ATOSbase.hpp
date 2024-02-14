/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef ATOSBASE_HPP
#define ATOSBASE_HPP

#include <memory>
#include <std_srvs/srv/set_bool.hpp>
#include "atos_interfaces/srv/get_object_ids.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "module.hpp"

class ATOSBase : public Module {
   public:
	ATOSBase();
	~ATOSBase();

   private:
	static inline std::string const moduleName = "atos_base";
	rclcpp::Service<atos_interfaces::srv::GetObjectIds>::SharedPtr getObjectIdsService;

	void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr) override;
	// Module only provides plumbing, no need to handle abort
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override{};

	void onRequestObjectIDs(const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request>,
							std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response>);
	bool isInitialized = false;
	ROSChannels::Exit::Sub exitSub;
};

#endif