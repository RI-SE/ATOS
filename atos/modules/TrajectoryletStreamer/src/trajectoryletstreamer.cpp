/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "trajectoryletstreamer.hpp"
#include <algorithm>
#include "atos_interfaces/srv/get_object_ids.hpp"

using namespace ATOS;
using namespace ROSChannels;
using std::placeholders::_1;

TrajectoryletStreamer::TrajectoryletStreamer()
	: Module(TrajectoryletStreamer::moduleName),
	  initSub(*this, std::bind(&TrajectoryletStreamer::onInitMessage, this, _1)),
	  connectedSub(*this, std::bind(&TrajectoryletStreamer::onObjectsConnectedMessage, this, _1)),
		abortSub(*this, std::bind(&TrajectoryletStreamer::onAbortMessage, this, _1)),
	  stopSub(*this, std::bind(&TrajectoryletStreamer::onStopMessage, this, _1)) {
	declare_parameter("chunk_duration", 0.0);
	idClient = create_client<atos_interfaces::srv::GetObjectIds>(ServiceNames::getObjectIds);
	trajectoryClient
		= create_client<atos_interfaces::srv::GetObjectTrajectory>(ServiceNames::getObjectTrajectory);
}

void TrajectoryletStreamer::onInitMessage(const std_msgs::msg::Empty::SharedPtr) {
	// Parse trajectory files into trajectories
	loadObjectFiles();
}

void TrajectoryletStreamer::onObjectsConnectedMessage(const ObjectsConnected::message_type::SharedPtr) {
	// TODO setup and first chunk transmission
	RCLCPP_INFO(get_logger(), "Starting trajectory publishers");
	for (const auto& [id, traj] : trajectories) {
		publishers.emplace_back(std::make_shared<TrajectoryPublisher>(*this, *traj, id, chunkLength));
	}
}


void TrajectoryletStreamer::loadObjectFiles() {
	clearScenario();
	double res = 0.0;
	auto success = get_parameter("chunk_duration", res);
	if (!success) {
		RCLCPP_ERROR(get_logger(), "Could not get parameter chunk_duration");
		return;
	} else {
		chunkLength = std::chrono::milliseconds(static_cast<int>(res * 1000.0));
		RCLCPP_INFO(get_logger(), "Chunk duration: %ld ms", chunkLength.count());
	}
	RCLCPP_INFO(get_logger(), "Loading trajectories");
	auto idsCallback = [&](const rclcpp::Client<atos_interfaces::srv::GetObjectIds>::SharedFuture future) {
		auto idResponse = future.get();
		for (const auto id : idResponse->ids) {
			auto trajectoryCallback =
				[this](const rclcpp::Client<atos_interfaces::srv::GetObjectTrajectory>::SharedFuture future) {
					RCLCPP_INFO(get_logger(), "Got trajectory");
					auto trajResponse = future.get();
					if (!trajResponse->success) {
						RCLCPP_ERROR(get_logger(), "Get trajectory service call failed for object %u",
									 trajResponse->id);
						return;
					}
					ATOS::Trajectory traj(get_logger());
					traj.initializeFromCartesianTrajectory(trajResponse->trajectory);
					trajectories[trajResponse->id] = std::make_unique<ATOS::Trajectory>(traj);
					RCLCPP_INFO(get_logger(), "Loaded trajectory for object %u with %ld points",
								trajResponse->id, trajectories[trajResponse->id]->size());
				};
			auto trajRequest = std::make_shared<atos_interfaces::srv::GetObjectTrajectory::Request>();
			trajRequest->id = id;
			trajectoryClient->async_send_request(trajRequest, trajectoryCallback);
		}
	};
	auto request = std::make_shared<atos_interfaces::srv::GetObjectIds::Request>();
	idClient->async_send_request(request, idsCallback);
}

void TrajectoryletStreamer::clearScenario() {
	publishers.clear();
	trajectories.clear();
}

void TrajectoryletStreamer::onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) {
	// TODO
}

void TrajectoryletStreamer::onStopMessage(const ROSChannels::Stop::message_type::SharedPtr) {
	// TODO
}