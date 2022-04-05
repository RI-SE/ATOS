#include "trajectorystreamer.hpp"
#include "maestro_interfaces/srv/get_object_ids.hpp"
#include <algorithm>

using namespace maestro;

TrajectoryStreamer::TrajectoryStreamer()
    : Module(TrajectoryStreamer::moduleName),
    initSub(*this, std::bind(&TrajectoryStreamer::onInitMessage, this, std::placeholders::_1)),
    connectedSub(*this, std::bind(&TrajectoryStreamer::onObjectsConnectedMessage, this, std::placeholders::_1)),
    startSub(*this, std::bind(&TrajectoryStreamer::onStartMessage, this, std::placeholders::_1))
{

}

void TrajectoryStreamer::onInitMessage(const std_msgs::msg::Empty::SharedPtr) {
    // Parse trajectory files into trajectories
    loadObjectFiles();
}

void TrajectoryStreamer::onObjectsConnectedMessage(const maestro_interfaces::msg::ObjectIdArray::SharedPtr) {
    // TODO setup and first chunk transmission
    RCLCPP_INFO(get_logger(), "Starting trajectory publishers");
    for (const auto& conf : objectConfigurations) {
        publishers.emplace_back(*this, conf->getTrajectory(), conf->getTransmitterID());
    }
}

void TrajectoryStreamer::onStartMessage(const std_msgs::msg::Empty::SharedPtr) {
    for (auto& pub : publishers) {
        pub.handleStart();
    }
}

void TrajectoryStreamer::loadObjectFiles() {
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


void TrajectoryStreamer::clearScenario() {
    publishers.clear();
	objectConfigurations.clear();
}

std::vector<uint32_t> TrajectoryStreamer::getObjectIds() const {
    std::vector<uint32_t> retval;
    std::transform(objectConfigurations.begin(), objectConfigurations.end(), std::back_inserter(retval),
        [](const auto& conf){ return conf->getTransmitterID(); });
    return retval;
}