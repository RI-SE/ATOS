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
#include "atos_interfaces/srv/get_object_ip.hpp"
#include "module.hpp"

class ATOSBase : public Module {
   public:
	ATOSBase();
	~ATOSBase();

   private:
	static inline std::string const moduleName = "atos_base";
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr initDataDictionaryService;
	rclcpp::Service<atos_interfaces::srv::GetObjectIds>::SharedPtr getObjectIdsService;
	rclcpp::Service<atos_interfaces::srv::GetObjectIp>::SharedPtr getObjectIpService;
	rclcpp::Service<atos_interfaces::srv::GetTestOrigin>::SharedPtr getTestOriginService;

	void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr) override;
	// Module only provides plumbing, no need to handle abort
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override{};

	void onInitDataDictionary(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
							  std::shared_ptr<std_srvs::srv::SetBool::Response>);
	void onRequestObjectIDs(const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request>,
							std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response>);
	void onRequestObjectIP(const std::shared_ptr<atos_interfaces::srv::GetObjectIp::Request>,
							std::shared_ptr<atos_interfaces::srv::GetObjectIp::Response>);
	void onRequestTestOrigin(const std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Request>,
							 std::shared_ptr<atos_interfaces::srv::GetTestOrigin::Response>);
	bool isInitialized = false;
	ROSChannels::Exit::Sub exitSub;
	std::map<uint32_t,uint32_t> getObjectsInfo();
	};

#endif