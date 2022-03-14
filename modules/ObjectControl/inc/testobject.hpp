#pragma once

#include <netinet/in.h>
#include <future>
#include <vector>
#include "trajectory.hpp"
#include "objectconfig.hpp"
#include "osi_handler.hpp"
#include "maestro_msgs/msg/control_signal_percentage.hpp"

using maestro_msgs::msg::ControlSignalPercentage;

// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

struct MonitorMessage : std::pair<uint32_t,ObjectMonitorType> {};

#define OSI_DEFAULT_OBJECT_TCP_PORT 53250

/*!
 * \brief The Channel class represents any socket based connection
 *			and allows transmission / reception of ISO messages
 */
class Channel {
public:
	Channel(const size_t bufferLength, const int type)
		: channelType(type),
		  transmitBuffer(bufferLength, 0),
		  receiveBuffer(bufferLength, 0)
	{}
	Channel(int type) : Channel(1024, type) {}
	struct sockaddr_in addr = {};
	int socket = -1;
	int channelType = 0; //!< SOCK_STREAM or SOCK_DGRAM
	std::vector<char> transmitBuffer;
	std::vector<char> receiveBuffer;

	ISOMessageID pendingMessageType(bool awaitNext = false);
	std::string remoteIP() const;
	bool isValid() const { return socket != -1; }
	void connect(std::shared_future<void> stopRequest,
				 const std::chrono::milliseconds retryPeriod);
	void disconnect();

	friend Channel& operator<<(Channel&,const HeabMessageDataType&);
	friend Channel& operator<<(Channel&,const ObjectSettingsType&);
	friend Channel& operator<<(Channel&,const Trajectory&);
	friend Channel& operator<<(Channel&,const ObjectCommandType&);
	friend Channel& operator<<(Channel&,const StartMessageType&);
	friend Channel& operator<<(Channel&,const std::vector<char>&);
	friend Channel& operator<<(Channel&,const RemoteControlManoeuvreMessageType&);

	friend Channel& operator>>(Channel&,MonitorMessage&);
	friend Channel& operator>>(Channel&,ObjectPropertiesType&);

};

/*!
 * \brief The ObjectConnection class holds network connection data for
 *			a single object, i.e. the two channels for command and
 *			safety data.
 */
class ObjectConnection {
public:
	Channel cmd;
	Channel mntr;

	ObjectConnection() : cmd(SOCK_STREAM), mntr(SOCK_DGRAM) {}

	bool isValid() const;
	bool isConnected() const;
	void connect(std::shared_future<void> stopRequest,
				 const std::chrono::milliseconds retryPeriod);
	void disconnect();
	ISOMessageID pendingMessageType(bool awaitNext = false);
};

class TestObject {
	using clock = std::chrono::steady_clock;
public:
	TestObject();
	TestObject(const TestObject&) = delete;
	TestObject(TestObject&&);

	TestObject& operator=(const TestObject&) = delete;
	TestObject& operator=(TestObject&&) = default;

	void parseConfigurationFile(const fs::path& file);

	uint32_t getTransmitterID() const { return conf.getTransmitterID(); }
	std::string getTrajectoryFileName() const { return conf.getTrajectoryFileName(); }
	Trajectory getTrajectory() const { return conf.getTrajectory(); }
	GeographicPositionType getOrigin() const { return conf.getOrigin(); }
	ObjectStateType getState(const bool awaitUpdate);
	ObjectStateType getState(const bool awaitUpdate, const std::chrono::milliseconds timeout);
	ObjectStateType getState() const { return isConnected() ? state : OBJECT_STATE_UNKNOWN; }
	ObjectMonitorType getLastMonitorData() const { return lastMonitor; }
	ObjectConfig getObjectConfig() const { return conf; }
	void setTrajectory(const Trajectory& newTrajectory) { conf.setTrajectory(newTrajectory); }
	void setCommandAddress(const sockaddr_in& newAddr);
	void setMonitorAddress(const sockaddr_in& newAddr);
	void setOsiAddress(const sockaddr_in& newAddr);
	void setObjectConfig(ObjectConfig& newObjectConfig);
	void setTriggerStart(const bool startOnTrigger = true);
	
	bool isAnchor() const { return conf.isAnchor(); }
	bool isOsiCompatible() const { return conf.isOSI(); }
	bool isStartingOnTrigger() const { return startOnTrigger; }
	std::string toString() const;
	std::string getProjString() const { return conf.getProjString(); }
	ObjectDataType getAsObjectData() const;

	bool isConnected() const { return comms.isConnected(); }
	void establishConnection(std::shared_future<void> stopRequest);
	void disconnect() {
		LogMessage(LOG_LEVEL_INFO, "Disconnecting object %u",
				   this->getTransmitterID());
		this->comms.disconnect();
	}

	void sendSettings();
	void sendHeartbeat(const ControlCenterStatusType ccStatus);
	void sendArm();
	void sendDisarm();
	void sendStart();
	void sendAllClear();
	void sendOsiData(const OsiHandler::LocalObjectGroundTruth_t& osidata,
					 const std::string& projStr,
					 const std::chrono::system_clock::time_point& timestamp);

	void sendControlSignal(RemoteControlManoeuvreMessageType& rcmm);

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
		LogMessage(LOG_LEVEL_DEBUG, "Ignoring object properties message");
		return retval;
	}

	ISOMessageID pendingMessageType(bool awaitNext = false) {
		return this->comms.pendingMessageType(awaitNext);
	}
private:
	ObjectConnection comms;		//!< Channel for communication with object over the ISO 22133 protocol
	Channel osiChannel;			//!< Channel for communication with object over the OSI protocol
	ObjectStateType state = OBJECT_STATE_UNKNOWN;

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

