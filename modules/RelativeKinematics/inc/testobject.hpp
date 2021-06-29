#pragma once

#include <netinet/in.h>
#include <future>
#include <vector>
#include "trajectory.hpp"
#include "objectconfig.hpp"


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
	Channel(const size_t bufferLength)
		: transmitBuffer(bufferLength, 0),
		  receiveBuffer(bufferLength, 0) {}
	Channel() : Channel(1024) {}
	struct sockaddr_in addr = {};
	int socket = -1;
	std::vector<char> transmitBuffer;
	std::vector<char> receiveBuffer;

	ISOMessageID pendingMessageType(bool awaitNext = false);
	std::string remoteIP() const;

	friend Channel& operator<<(Channel&,const HeabMessageDataType&);
	friend Channel& operator<<(Channel&,const ObjectSettingsType&);
	friend Channel& operator<<(Channel&,const Trajectory&);
	friend Channel& operator<<(Channel&,const ObjectCommandType&);
	friend Channel& operator<<(Channel&,const StartMessageType&);
	friend Channel& operator<<(Channel&,std::vector<char>&);

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
	void parseTrajectoryFile(const fs::path& file);

	uint32_t getTransmitterID() const { return transmitterID; }
	fs::path getTrajectoryFile() const { return trajectoryFile; }
	Trajectory getTrajectory() const { return trajectory; }
	GeographicPositionType getOrigin() const { return origin; }
	ObjectStateType getState(const bool awaitUpdate);
	ObjectStateType getState(const bool awaitUpdate, const std::chrono::milliseconds timeout);
	ObjectStateType getState() const { return isConnected() ? state : OBJECT_STATE_UNKNOWN; }
	ObjectMonitorType getLastMonitorData() const { return lastMonitor; }
	ObjectConfig getObjectConfig(){ return objectConfig; }
	void setTrajectory(const Trajectory& newTrajectory) { trajectory = newTrajectory; }
	void setCommandAddress(const sockaddr_in& newAddr);
	void setMonitorAddress(const sockaddr_in& newAddr);
	void setOsiAddress(const sockaddr_in& newAddr);
	void setObjectConfig(ObjectConfig& newObjectConfig); 
	
	bool isAnchor() const { return isAnchorObject; }
	bool isOsiCompatible() const { return isOsiObject; }
	std::string toString() const;
	ObjectDataType getAsObjectData() const;

	bool isConnected() const { return comms.isConnected(); }
	void establishConnection(std::shared_future<void> stopRequest);
	void disconnect() {
		LogMessage(LOG_LEVEL_INFO, "Disconnecting object %u",
				   this->transmitterID);
		this->comms.disconnect();
	}

	void sendSettings();
	void sendHeartbeat(const ControlCenterStatusType ccStatus);
	void sendArm();
	void sendDisarm();
	void sendStart();
	void sendOsiData(std::vector<char> osidata);

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
	Channel osi;
	ObjectConnection comms;
	ObjectStateType state = OBJECT_STATE_UNKNOWN;

	fs::path objectFile;
	fs::path trajectoryFile;
	uint32_t transmitterID = 0;
	bool isAnchorObject = false;
	bool isOsiObject = false;
	ObjectConfig objectConfig;
	Trajectory trajectory;
	GeographicPositionType origin;
	bool isEnabled = true;

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

template<typename Duration>
void to_timeval(Duration&& d, struct timeval & tv) {
	std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(d);

	tv.tv_sec  = sec.count();
	tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(d - sec).count();
}

template<typename Duration>
void from_timeval(struct timeval & tv, Duration& d) {
	// TODO
	//const auto sec = std::chrono::seconds(tv.tv_sec);
	//const auto usec = std::chrono::microseconds(tv.tv_usec);
	//d = sec + usec;
}