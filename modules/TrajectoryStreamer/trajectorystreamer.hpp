#include "module.hpp"
#include "trajectory.hpp"
#include "objectconfig.hpp"
#include "roschannel.hpp"
#include "trajectorypublisher.hpp"
#include <vector>
#include <map>

namespace maestro {

class TrajectoryStreamer : public Module {
public:
    TrajectoryStreamer();
private:
	static inline std::string const moduleName = "trajectory_streamer";
    void onInitMessage(const std_msgs::msg::Empty::SharedPtr);
    void onObjectsConnectedMessage(const maestro_interfaces::msg::ObjectIdArray::SharedPtr);
    void onStartMessage(const std_msgs::msg::Empty::SharedPtr);
    void onAbortMessage(const std_msgs::msg::Empty::SharedPtr) override {}

    void loadObjectFiles();
    void clearScenario();

    std::vector<uint32_t> getObjectIds() const;

    ROSChannels::Init::Sub initSub;
    ROSChannels::ObjectsConnected::Sub connectedSub;
    ROSChannels::Start::Sub startSub;

    std::vector<TrajectoryPublisher> publishers;

    std::vector<std::unique_ptr<ObjectConfig>> objectConfigurations;
};

}  // namespace maestro