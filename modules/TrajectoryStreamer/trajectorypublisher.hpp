#include "objectconfig.hpp"
#include "roschannel.hpp"
#include <memory>
#include <thread>

namespace maestro {

class TrajectoryPublisher {
public:
    TrajectoryPublisher(rclcpp::Node& node, const Trajectory&, const uint32_t objectId);
    void beginPublish();
private:
    std::shared_ptr<Trajectory> traj;
    ROSChannels::Trajectory::Pub pub;
    uint32_t objectId;
    std::thread pubThread;
};

}  // namespace maestro