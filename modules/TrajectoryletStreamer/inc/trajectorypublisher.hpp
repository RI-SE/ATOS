/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectconfig.hpp"
#include "roschannels/pathchannel.hpp"
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
	TrajectoryPublisher(
		rclcpp::Node& node,
		const Trajectory&,
		const uint32_t objectId,
		const std::chrono::milliseconds chunkLength
			= std::chrono::milliseconds(0));
	void handleStart();

	void setChunkLength(std::chrono::milliseconds chunkLength) {
		this->chunkLength = chunkLength;
	}
private:
	ROSChannels::Path::Pub pub;
	std::shared_ptr<rclcpp::TimerBase> timer;

	std::chrono::milliseconds chunkLength = std::chrono::milliseconds(0);
	std::chrono::milliseconds publishPeriod = std::chrono::milliseconds(50);
	std::unique_ptr<std::chrono::steady_clock::time_point> startTime;
	std::chrono::steady_clock::time_point lastPublishTime;

	std::unique_ptr<const Trajectory> traj;
	Chunk lastPublishedChunk;

	void publishChunk();
	nav_msgs::msg::Path chunkToPath(Chunk chunk, std::chrono::steady_clock::time_point);
	Chunk extractChunk(std::chrono::steady_clock::duration beginTime, std::chrono::steady_clock::duration endTime);

};

}  // namespace ATOS
