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

class TestObject {
public:
	TestObject();
	void parseConfigurationFile(const fs::path& file);
	void parseTrajectoryFile(const fs::path& file);

	uint32_t getTransmitterID() const { return transmitterID; }
	fs::path getTrajectoryFile() const { return trajectoryFile; }
	Trajectory getTrajectory() const { return trajectory; }
	ObjectStateType getState() const { return state; }

	void setTrajectory(const Trajectory& newTrajectory) { trajectory = newTrajectory; }
	void setCommandAddress(const sockaddr_in& newAddr);
	void setMonitorAddress(const sockaddr_in& newAddr);

	bool isVehicleUnderTest() const { return isVUT; }
	std::string toString() const;

	void initiateConnection(std::shared_future<void> stopRequest);
private:
	class ObjectConnection {
	public:
		struct sockaddr_in cmdAddr;
		struct sockaddr_in mntrAddr;
		int commandSocket;
		int monitorSocket;

		bool valid() const;
		bool connected() const;
		void disconnect();
	};
	ObjectConnection channel;
	ObjectStateType state = OBJECT_STATE_UNKNOWN;

	fs::path objectFile;
	fs::path trajectoryFile;
	uint32_t transmitterID;
	bool isVUT = false;
	Trajectory trajectory;

	static constexpr int connRetryPeriod_ms = 1000;
	void establishConnection(std::promise<void> result, std::shared_future<void> stopRequest);
	std::promise<void> connResultPromise;
	std::future<void> connResult;
	std::thread connThread;
};

// Template specialisation of std::less for TestObject
namespace std {
	template<> struct less<TestObject> {
		bool operator() (const TestObject& lhs, const TestObject& rhs) const {
			return lhs.getTransmitterID() < rhs.getTransmitterID();
		}
	};
}
