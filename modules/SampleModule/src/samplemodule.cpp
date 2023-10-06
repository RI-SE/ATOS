#include "samplemodule.hpp"
#include "roschannels/commandchannels.hpp"
#include "rclcpp/wait_for_message.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

SampleModule::SampleModule() :
	Module(moduleName),
	initSub(*this, std::bind(&SampleModule::onInitMessage, this, _1)),
	abortSub(*this, std::bind(&SampleModule::onAbortMessage, this, _1)),
	allClearSub(*this, std::bind(&SampleModule::onAllClearMessage, this, _1)),
	smOnInitResponsePub(*this)
{
}

//! Message queue callbacks
void SampleModule::onInitMessage(const Init::message_type::SharedPtr) {
	// Callback executed when this module receives a test is initialized

	// Get the object id of the objects in the scenario
	ConnectedObjectIds::message_type connectedObjectIds;
	// Wait for a single message containing a vector of ids, at most 1000ms
	rclcpp::wait_for_message(connectedObjectIds, shared_from_this(), std::string(get_namespace()) + "connected_object_ids", 1000ms);
	// Populate a list of object IDs
	for (auto id : connectedObjectIds.ids) {
		objectIds.push_back(id);
	}
	smOnInitResponsePub.publish(std_msgs::msg::Empty());
}

void SampleModule::onAbortMessage(const Abort::message_type::SharedPtr) {
	aborting_ = true;
}

void SampleModule::onAllClearMessage(const AllClear::message_type::SharedPtr) {}

/*! \brief  returns the object ids of the objects in the scenario
 * \return a vector of object ids
*/
std::vector<std::uint32_t> SampleModule::getObjectIds() {
	return objectIds;
}