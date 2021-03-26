#include <netinet/in.h>
#include <future>
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
	struct sockaddr_in addr = {};
	int socket = -1;
	char transmitBuffer[1024];

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
	void parseConfigurationFile(const fs::path& file);
	void parseTrajectoryFile(const fs::path& file);

	uint32_t getTransmitterID() const { return transmitterID; }
	fs::path getTrajectoryFile() const { return trajectoryFile; }
	Trajectory getTrajectory() const { return trajectory; }
	GeographicPositionType getOrigin() const { return origin; }
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
private:
	ObjectConnection comms;
	ObjectStateType state = OBJECT_STATE_UNKNOWN;

	fs::path objectFile;
	fs::path trajectoryFile;
	uint32_t transmitterID;
	bool isVUT = false;
	Trajectory trajectory;
	GeographicPositionType origin = {};

	static constexpr int connRetryPeriod_ms = 1000;
};

// Template specialisation of std::less for TestObject
namespace std {
	template<> struct less<TestObject> {
		bool operator() (const TestObject& lhs, const TestObject& rhs) const {
			return lhs.getTransmitterID() < rhs.getTransmitterID();
		}
	};
}
