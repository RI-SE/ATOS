#include "module.hpp"
#include "trajectory.hpp"
#include "objectconfig.hpp"
#include "roschannel.hpp"
#include "trajectorypublisher.hpp"
#include <vector>
#include <map>

namespace maestro {

class TrajectoryletStreamer : public Module {
public:
    TrajectoryletStreamer();
private:
	static inline std::string const moduleName = "trajectorylet_streamer";
    void onInitMessage(const ROSChannels::Init::message_type::SharedPtr);
    void onObjectsConnectedMessage(const ROSChannels::ObjectsConnected::message_type::SharedPtr);
    void onStartMessage(const ROSChannels::Start::message_type::SharedPtr);
    void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
    void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) override;

    void loadObjectFiles();
    void clearScenario();

    std::vector<uint32_t> getObjectIds() const;

    ROSChannels::Init::Sub initSub;
    ROSChannels::ObjectsConnected::Sub connectedSub;
    ROSChannels::Start::Sub startSub;
    ROSChannels::Abort::Sub abortSub;
    ROSChannels::Stop::Sub stopSub;

    std::vector<TrajectoryPublisher> publishers;

    std::vector<std::unique_ptr<ObjectConfig>> objectConfigurations;
};

}  // namespace maestro