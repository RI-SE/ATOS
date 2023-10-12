/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#pragma once

#include <thread>
#include "module.hpp"
#include "server.hpp"


// Module specific publisher/subscriber. 
// When creating channels for non-test modules, put them in a namespace under common/roschannels instead.
namespace ROSChannels {
    namespace SampleModuleTestForInitResponce {
        const std::string topicName = "sample_module_test_for_init_responce";
        using message_type = std_msgs::msg::Empty;
		const rclcpp::QoS defaultQoS = rclcpp::QoS(rclcpp::KeepAll());
        class Pub : public BasePub<message_type> {
        public:
            Pub(rclcpp::Node& node, const rclcpp::QoS& qos = defaultQoS) : BasePub<message_type>(node, topicName, qos) {}
        };

		class Sub : public BaseSub<message_type> {
		public:
			Sub(rclcpp::Node& node, std::function<void(const message_type::SharedPtr)> callback, const rclcpp::QoS& qos = defaultQoS) : BaseSub<message_type>(node, topicName, callback, qos) {}
		};
    }
}

/*!
 * \brief The SampleModule is a ros2 node that demonstrates how to use the Module class 
 */
class SampleModule : public Module{
public:
	static inline std::string const moduleName = "sample_module";
	SampleModule();
	std::vector<std::uint32_t> getObjectIds();
	bool getAborting() { return aborting_; }

private:
	ROSChannels::Init::Sub initSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::AllClear::Sub allClearSub;
	ROSChannels::SampleModuleTestForInitResponce::Pub smOnInitResponsePub;

	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr) override;

	std::vector<std::uint32_t> objectIds;
	bool aborting_ = false;
};
