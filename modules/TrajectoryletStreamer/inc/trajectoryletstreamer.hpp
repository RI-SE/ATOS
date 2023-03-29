/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <chrono>
#include <map>
#include <memory>
#include <vector>
#include "atos_interfaces/srv/get_object_ids.hpp"
#include "atos_interfaces/srv/get_object_trajectory.hpp"
#include "module.hpp"
#include "objectconfig.hpp"
#include "roschannels/commandchannels.hpp"
#include "trajectory.hpp"
#include "trajectorypublisher.hpp"

namespace ATOS {

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

	ROSChannels::Init::Sub initSub;
	ROSChannels::ObjectsConnected::Sub connectedSub;
	ROSChannels::Start::Sub startSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::Stop::Sub stopSub;

	rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedPtr idClient;	 //!< Client to request object ids
	rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedPtr
		trajectoryClient;  //!< Client to request object trajectories

	std::vector<TrajectoryPublisher> publishers;

	std::map<uint32_t, std::unique_ptr<ATOS::Trajectory>> trajectories;

	std::chrono::milliseconds chunkLength;
};

}  // namespace ATOS
