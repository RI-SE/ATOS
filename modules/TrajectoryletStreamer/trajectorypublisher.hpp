#include "objectconfig.hpp"
#include "roschannel.hpp"
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <utility>

namespace ATOS {

class TrajectoryPublisher {
private:
    typedef std::pair<std::vector<Trajectory::TrajectoryPoint>::const_iterator,
        std::vector<Trajectory::TrajectoryPoint>::const_iterator> Chunk;
public:
    TrajectoryPublisher(rclcpp::Node& node, const Trajectory&, const uint32_t objectId);
    void handleStart();
private:
    ROSChannels::Path::Pub pub;
    std::shared_ptr<rclcpp::TimerBase> timer;

    std::chrono::milliseconds chunkLength = std::chrono::milliseconds(2000);
    std::chrono::milliseconds publishPeriod = std::chrono::milliseconds(50);
    std::unique_ptr<std::chrono::steady_clock::time_point> startTime;

    std::unique_ptr<const Trajectory> traj;
    Chunk lastPublishedChunk;

    void publishChunk();
    nav_msgs::msg::Path chunkToPath(Chunk chunk, std::chrono::steady_clock::time_point);
    Chunk extractChunk(std::chrono::steady_clock::duration beginTime, std::chrono::steady_clock::duration endTime);

};

}  // namespace ATOS