#include <netinet/in.h>
#include <future>
#include <vector>
#include "trajectory.hpp"

// GCC version 8.1 brings non-experimental support for std::filesystem
#if __GNUC__ > 8 || (__GNUC__ == 8 && __GNUC_MINOR__ >= 1)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

class Channel {
public:
	Channel() : transmitBuffer(1024, 0), receiveBuffer(1024, 0) {}
	struct sockaddr_in addr = {};
	int socket = -1;
	std::vector<char> transmitBuffer;
	std::vector<char> receiveBuffer;

	friend Channel& operator<<(Channel&,const HeabMessageDataType&);
	friend Channel& operator<<(Channel&,const ObjectSettingsType&);
	friend Channel& operator<<(Channel&,const Trajectory&);
};

class ObjectConnection {
public:
	Channel cmd;
	Channel mntr;

	bool valid() const;
	bool connected() const;
	void disconnect();
};

class TestObject {
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
	ObjectStateType getState(bool awaitUpdate);
	ObjectStateType getState() const { return state; }
	void setTrajectory(const Trajectory& newTrajectory) { trajectory = newTrajectory; }
	void setCommandAddress(const sockaddr_in& newAddr);
	void setMonitorAddress(const sockaddr_in& newAddr);

	bool isVehicleUnderTest() const { return isVUT; }
	std::string toString() const;

	bool isConnected() const { return comms.connected(); }
	void establishConnection(std::shared_future<void> stopRequest);
	void disconnect() { this->comms.disconnect(); }

	void sendSettings();
	void sendHeartbeat(const ControlCenterStatusType ccStatus);

	void updateMonitor(const ObjectDataType&);
private:
	ObjectConnection comms;
	ObjectStateType state = OBJECT_STATE_UNKNOWN;

	fs::path objectFile;
	fs::path trajectoryFile;
	uint32_t transmitterID = 0;
	bool isVUT = false;
	Trajectory trajectory;
	GeographicPositionType origin;
	bool isEnabled = true;

	ObjectDataType awaitNextMonitor();
	std::future<ObjectDataType> nextMonitor;
	ObjectDataType lastMonitor; // TODO change this into a more usable format

	static constexpr std::chrono::milliseconds connRetryPeriod = std::chrono::milliseconds(1000);
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
