/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "objectconnection.hpp"

#include <netinet/in.h>
#include <future>
#include <vector>
#include "trajectory.hpp"
#include "objectconfig.hpp"
#include "osi_handler.hpp"
#include "roschannels/controlsignalchannel.hpp"
#include "roschannels/navsatfixchannel.hpp"
#include "roschannels/pathchannel.hpp"
#include "roschannels/monitorchannel.hpp"

#include "loggable.hpp"

using atos_interfaces::msg::ControlSignalPercentage;

// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#define OSI_DEFAULT_OBJECT_TCP_PORT 53250

class TestObject : public rclcpp::Node {
	using clock = std::chrono::steady_clock;
public:
	TestObject(uint32_t id);
	TestObject(const TestObject&) = delete;
	TestObject(TestObject&&);

	TestObject& operator=(const TestObject&) = delete;
	TestObject& operator=(TestObject&&) = default;

	void parseConfigurationFile(const fs::path& file);

	uint32_t getTransmitterID() const { return conf.getTransmitterID(); }
	std::string getTrajectoryFileName() const { return conf.getTrajectoryFileName(); }
	ATOS::Trajectory getTrajectory() const { return conf.getTrajectory(); }
	GeographicPositionType getOrigin() const { return conf.getOrigin(); }
	ObjectStateType getState(const bool awaitUpdate);
	ObjectStateType getState(const bool awaitUpdate, const std::chrono::milliseconds timeout);
	ObjectStateType getState() const { return isConnected() ? state : OBJECT_STATE_UNKNOWN; }
	ObjectMonitorType getLastMonitorData() const { return lastMonitor; }
	ObjectConfig getObjectConfig() const { return conf; }
	void setTrajectory(const ATOS::Trajectory& newTrajectory) { conf.setTrajectory(newTrajectory); }
	void setTransmitterID(const uint32_t newID) { conf.setTransmitterID(newID); }
	void setLastReceivedPath(ROSChannels::Path::message_type::SharedPtr);
	void setObjectIP(const in_addr_t newIP);
	void setCommandAddress(const sockaddr_in& newAddr);
	void setMonitorAddress(const sockaddr_in& newAddr);
	void setOsiAddress(const sockaddr_in& newAddr);
	void setObjectConfig(ObjectConfig& newObjectConfig);
	void setTriggerStart(const bool startOnTrigger = true);
	void setOrigin(const GeographicPositionType&);
	void interruptSocket() { comms.interruptSocket();}
	
	bool isAnchor() const { return conf.isAnchor(); }
	bool isOsiCompatible() const { return conf.isOSI(); }
	bool isStartingOnTrigger() const { return startOnTrigger; }
	std::string toString() const;
	std::string getProjString() const { return conf.getProjString(); }
	ObjectDataType getAsObjectData() const;

	bool isConnected() const { return comms.isConnected(); }
	void establishConnection(std::shared_future<void> stopRequest);
	void disconnect() {
		RCLCPP_INFO(get_logger(), "Disconnecting object %u",
				   this->getTransmitterID());
		this->comms.disconnect();
	}

	void sendSettings();
	void sendHeartbeat(const ControlCenterStatusType ccStatus);
	void sendArm();
	void sendDisarm();
	void sendRemoteControl(bool on);
	void sendStart(std::chrono::system_clock::time_point timestamp);
	void sendAllClear();
	void sendOsiData(const OsiHandler::LocalObjectGroundTruth_t& osidata,
					 const std::string& projStr,
					 const std::chrono::system_clock::time_point& timestamp);

	void sendControlSignal(const ControlSignalPercentage::SharedPtr csp);
	void publishMonr(const ROSChannels::Monitor::message_type);
	void publishNavSatFix(const ROSChannels::NavSatFix::message_type);

	std::chrono::milliseconds getTimeSinceLastMonitor() const {
		if (lastMonitorTime.time_since_epoch().count() == 0) {
			return std::chrono::milliseconds(0);
		}
		return std::chrono::duration_cast<std::chrono::milliseconds>(
					clock::now() - lastMonitorTime);
	}

	std::chrono::milliseconds getMaxAllowedMonitorPeriod() const {
		return this->maxAllowedMonitorPeriod;
	}
	MonitorMessage readMonitorMessage() {
		MonitorMessage retval;
		this->comms.mntr >> retval;
		lastMonitorTime = clock::now();
		updateMonitor(retval);
		return retval;
	}
	ObjectPropertiesType parseObjectPropertyMessage() {
		ObjectPropertiesType retval;
		this->comms.cmd >> retval; // TODO make use of this
		RCLCPP_DEBUG(get_logger(), "Ignoring object properties message");
		return retval;
	}

	ISOMessageID pendingMessageType(bool awaitNext = false) {
		return this->comms.pendingMessageType(awaitNext);
	}
private:
	ObjectConnection comms;		//!< Channel for communication with object over the ISO 22133 protocol
	Channel osiChannel;			//!< Channel for communication with object over the OSI protocol
	ObjectStateType state = OBJECT_STATE_UNKNOWN;
	std::shared_ptr<ROSChannels::Monitor::Pub> monrPub;
	std::shared_ptr<ROSChannels::NavSatFix::Pub> navSatFixPub;
	std::shared_ptr<ROSChannels::Path::Sub> pathSub;
	std::shared_ptr<ROSChannels::Path::message_type> lastReceivedPath;

	void onPathMessage(const ROSChannels::Path::message_type::SharedPtr msg, int id);

	ObjectConfig conf;

	bool startOnTrigger = false;

	void updateMonitor(const MonitorMessage&);
	MonitorMessage awaitNextMonitor();
	std::future<MonitorMessage> nextMonitor;
	ObjectMonitorType lastMonitor; // TODO change this into a more usable format
	clock::time_point lastMonitorTime;

	static constexpr auto connRetryPeriod = std::chrono::milliseconds(1000);
	std::chrono::milliseconds maxAllowedMonitorPeriod = std::chrono::milliseconds(static_cast<unsigned int>(1000.0 * 100.0 / MONR_EXPECTED_FREQUENCY_HZ ));
};

// Template specialisation of std::less for TestObject
namespace std {
	template<> struct less<TestObject> {
		bool operator() (const TestObject& lhs, const TestObject& rhs) const {
			return lhs.getTransmitterID() < rhs.getTransmitterID();
		}
	};
}

