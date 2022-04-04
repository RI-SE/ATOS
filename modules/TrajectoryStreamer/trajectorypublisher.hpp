#include "objectconfig.hpp"
#include "roschannel.hpp"
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

namespace maestro {

class TrajectoryPublisher {
public:
    TrajectoryPublisher(rclcpp::Node& node, const Trajectory&, const uint32_t objectId);
    void handleStart();
    void beginPublish();
    nav_msgs::msg::Path extractChunk();
private:
    std::chrono::milliseconds chunkLength = std::chrono::milliseconds(2000);
    std::chrono::milliseconds publishPeriod = std::chrono::milliseconds(50);
    std::unique_ptr<std::chrono::steady_clock::time_point> startTime;
    std::shared_ptr<Trajectory> traj;
    ROSChannels::Trajectory::Pub pub;
    uint32_t objectId;
    std::thread pubThread;
};

}  // namespace maestro