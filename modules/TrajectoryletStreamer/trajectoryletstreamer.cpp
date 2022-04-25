#include "trajectoryletstreamer.hpp"
#include "maestro_interfaces/srv/get_object_ids.hpp"
#include <algorithm>

using namespace maestro;
using namespace ROSChannels;
using std::placeholders::_1;

TrajectoryletStreamer::TrajectoryletStreamer()
    : Module(TrajectoryletStreamer::moduleName),
    initSub(*this, std::bind(&TrajectoryletStreamer::onInitMessage, this, _1)),
    connectedSub(*this, std::bind(&TrajectoryletStreamer::onObjectsConnectedMessage, this, _1)),
    startSub(*this, std::bind(&TrajectoryletStreamer::onStartMessage, this, _1)),
    stopSub(*this, std::bind(&TrajectoryletStreamer::onStopMessage, this, _1)),
    abortSub(*this, std::bind(&TrajectoryletStreamer::onAbortMessage, this, _1))
{

}

void TrajectoryletStreamer::onInitMessage(const std_msgs::msg::Empty::SharedPtr) {
    // Parse trajectory files into trajectories
    loadObjectFiles();
}

void TrajectoryletStreamer::onObjectsConnectedMessage(const maestro_interfaces::msg::ObjectIdArray::SharedPtr) {
    // TODO setup and first chunk transmission
    RCLCPP_INFO(get_logger(), "Starting trajectory publishers");
    for (const auto& conf : objectConfigurations) {
        publishers.emplace_back(*this, conf->getTrajectory(), conf->getTransmitterID());
    }
}

void TrajectoryletStreamer::onStartMessage(const std_msgs::msg::Empty::SharedPtr) {
    for (auto& pub : publishers) {
        pub.handleStart();
    }
}

void TrajectoryletStreamer::loadObjectFiles() {
	clearScenario();
	char path[MAX_FILE_PATH];
	std::vector<std::invalid_argument> errors;

	UtilGetObjectDirectoryPath(path, sizeof (path));
	fs::path objectDir(path);
	if (!fs::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	for (const auto& entry : fs::directory_iterator(objectDir)) {
		if (fs::is_regular_file(entry.status())) {
			ObjectConfig conf;
            conf.parseConfigurationFile(entry.path());
            objectConfigurations.push_back(std::make_unique<ObjectConfig>(conf));
		}
	}
    RCLCPP_INFO(get_logger(), "Loaded %d object configurations", objectConfigurations.size());
}


void TrajectoryletStreamer::clearScenario() {
    publishers.clear();
	objectConfigurations.clear();
}

std::vector<uint32_t> TrajectoryletStreamer::getObjectIds() const {
    std::vector<uint32_t> retval;
    std::transform(objectConfigurations.begin(), objectConfigurations.end(), std::back_inserter(retval),
        [](const auto& conf){ return conf->getTransmitterID(); });
    return retval;
}

void TrajectoryletStreamer::onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) {
    // TODO
}

void TrajectoryletStreamer::onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) {
    // TODO
}