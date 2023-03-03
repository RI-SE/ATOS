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
	tcpServer("0.0.0.0",TCPPort)
{
	tcpThread = std::make_unique<std::thread>(&SampleModule::tcpSocketProcedure, this);
	RCLCPP_INFO(get_logger(), "%s task running with PID: %d",moduleName.c_str(), getpid());
}

SampleModule::~SampleModule() {
	// When the SampleModule object is destroyed, make sure that socket thread is joined
	quit = true;
	if (tcpThread != nullptr) {
		tcpThread->join();
	}
}

//! Message queue callbacks
void SampleModule::onInitMessage(const Init::message_type::SharedPtr) {
	// Callback executed when this module receives a test is initialized

	// Get the object id of the objects in the scenario
	ConnectedObjectIds::message_type connectedObjectIds;
	// Wait for a single message containing a vector of ids, at most 1000ms
	rclcpp::wait_for_message(connectedObjectIds, shared_from_this(), std::string(get_namespace()) + "/connected_object_ids", 1000ms);
	// Do something with IDs
}

void SampleModule::onAbortMessage(const Abort::message_type::SharedPtr) {}

void SampleModule::onAllClearMessage(const AllClear::message_type::SharedPtr) {}

/*! \brief This function is executed in a separate thread
 * It is used to handle incoming TCP connections
 * and receive data from them
*/
void SampleModule::tcpSocketProcedure() {
	while (!quit) {
		auto socket = tcpServer.await();
		auto bytes = socket.recv();
		for (auto b : bytes) {
			RCLCPP_INFO(get_logger(), "Received byte: %d", b);
		}
	}
}
