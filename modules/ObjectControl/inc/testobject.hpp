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
#include "roschannels/objstatechangechannel.hpp"

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
public:
	TestObject(uint32_t id);
	TestObject(const TestObject&) = delete;
	TestObject(TestObject&&);

	TestObject& operator=(const TestObject&) = delete;
	TestObject& operator=(TestObject&&) = default;

	virtual void parseConfigurationFile(const fs::path& file);

	virtual uint32_t getTransmitterID() const { return conf.getTransmitterID(); }
	virtual std::string getTrajectoryFileName() const { return conf.getTrajectoryFileName(); }
	virtual ATOS::Trajectory getTrajectory() const { return conf.getTrajectory(); }
	virtual GeographicPositionType getOrigin() const { return conf.getOrigin(); }
	virtual ObjectStateType getState(const bool awaitUpdate);
	virtual ObjectStateType getState(const bool awaitUpdate, const std::chrono::milliseconds timeout);
	virtual ObjectStateType getState() const { return isConnected() ? state : OBJECT_STATE_UNKNOWN; }
	virtual ObjectMonitorType getLastMonitorData() const { return lastMonitor; }
	virtual ObjectConfig getObjectConfig() const { return conf; }
	virtual void setTrajectory(const ATOS::Trajectory& newTrajectory) { conf.setTrajectory(newTrajectory); }
	virtual void setTransmitterID(const uint32_t newID) { conf.setTransmitterID(newID); }
	virtual void setLastReceivedPath(ROSChannels::Path::message_type::SharedPtr);
	virtual void setObjectIP(const in_addr_t newIP);
	virtual void setCommandAddress(const sockaddr_in& newAddr);
	virtual void setMonitorAddress(const sockaddr_in& newAddr);
	virtual void setOsiAddress(const sockaddr_in& newAddr);
	virtual void setObjectConfig(ObjectConfig& newObjectConfig);
	virtual void setTriggerStart(const bool startOnTrigger = true);
	virtual void setOrigin(const GeographicPositionType&);
	virtual void interruptSocket() { comms.interruptSocket();}
	
	virtual bool isAnchor() const { return conf.isAnchor(); }
	virtual bool isOsiCompatible() const { return conf.isOSI(); }
	virtual bool isStartingOnTrigger() const { return startOnTrigger; }
	virtual std::string toString() const;
	virtual std::string getProjString() const { return conf.getProjString(); }
	virtual ObjectDataType getAsObjectData() const;

	virtual bool isConnected() const { return comms.isConnected(); }
	virtual void establishConnection(std::shared_future<void> stopRequest);
	virtual void disconnect() {
		RCLCPP_INFO(get_logger(), "Disconnecting object %u",
				   this->getTransmitterID());
		this->comms.disconnect();
	}

	virtual void sendSettings();
	virtual void sendHeartbeat(const ControlCenterStatusType ccStatus);
	virtual void sendArm();
	virtual void sendDisarm();
	virtual void sendRemoteControl(bool on);
	virtual void sendStart(std::chrono::system_clock::time_point timestamp);
	virtual void sendAllClear();
	virtual void sendOsiData(const OsiHandler::LocalObjectGroundTruth_t& osidata,
					 const std::string& projStr,
					 const std::chrono::system_clock::time_point& timestamp);

	virtual void sendControlSignal(const ControlSignalPercentage::SharedPtr csp);
	virtual void publishMonr(const ROSChannels::Monitor::message_type);
	virtual void publishNavSatFix(const ROSChannels::NavSatFix::message_type);

	virtual std::chrono::milliseconds getTimeSinceLastMonitor() const {
		if (lastMonitorTime.time_since_epoch().count() == 0) {
			return std::chrono::milliseconds(0);
		}
		return std::chrono::duration_cast<std::chrono::milliseconds>(
					clock::now() - lastMonitorTime);
	}

	virtual std::chrono::milliseconds getMaxAllowedMonitorPeriod() const {
		return this->maxAllowedMonitorPeriod;
	}
	virtual MonitorMessage readMonitorMessage() {
		MonitorMessage retval;
		this->comms.mntr >> retval;
		lastMonitorTime = clock::now();
		updateMonitor(retval);
		return retval;
	}
	virtual ObjectPropertiesType parseObjectPropertyMessage() {
		ObjectPropertiesType retval;
		this->comms.cmd >> retval; // TODO make use of this
		RCLCPP_DEBUG(get_logger(), "Ignoring object properties message");
		return retval;
	}
	virtual void handleISOMessage(bool awaitNext = false);

protected:
	using clock = std::chrono::steady_clock;
	ObjectConnection comms;		//!< Channel for communication with object over the ISO 22133 protocol
	Channel osiChannel;			//!< Channel for communication with object over the OSI protocol
	ObjectStateType state = OBJECT_STATE_UNKNOWN;
	std::shared_ptr<ROSChannels::Monitor::Pub> monrPub;
	std::shared_ptr<ROSChannels::NavSatFix::Pub> navSatFixPub;
	std::shared_ptr<ROSChannels::Path::Sub> pathSub;
	std::shared_ptr<ROSChannels::ObjectStateChange::Pub> stateChangePub;
	std::shared_ptr<ROSChannels::Path::message_type> lastReceivedPath;

	virtual void onPathMessage(const ROSChannels::Path::message_type::SharedPtr msg, int id);
	virtual void publishMonitor(MonitorMessage& monr);
	virtual void publishStateChange(ObjectStateType &prevObjState);

	ObjectConfig conf;

	bool startOnTrigger = false;

	virtual void updateMonitor(const MonitorMessage&);
	virtual MonitorMessage awaitNextMonitor();
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

