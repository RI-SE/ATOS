/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "trajectorypublisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/path.hpp>

using namespace ATOS;
using namespace std::chrono_literals;

TrajectoryPublisher::TrajectoryPublisher(
	rclcpp::Node& node,
	const Trajectory& _traj,
	const uint32_t objectId,
	const std::chrono::milliseconds chunkLength)
	: traj(std::make_unique<const Trajectory>(_traj)),
	pathPub(node, objectId),
	trajPub(node, objectId),
	lastPublishedChunk(traj->points.end(), traj->points.end()),
	chunkLength(chunkLength)
{
	timer = node.create_wall_timer(publishPeriod, std::bind(&TrajectoryPublisher::publishChunk, this));
}

void TrajectoryPublisher::publishChunk()
{
	using std::chrono::steady_clock;
	Chunk chunk(traj->points.cbegin(), traj->points.cend());
	auto now = steady_clock::now();
	auto trajTimeOffset = now;
	auto timeIntoTraj = steady_clock::duration(0);
	if (startTime) {
		timeIntoTraj = now-*startTime;
		trajTimeOffset = *startTime;
	}
	if (chunkLength.count() > 0) {
		chunk = extractChunk(timeIntoTraj, timeIntoTraj + chunkLength);
	}
	if (chunk != lastPublishedChunk ||
		(chunkLength == 0ms && now - lastPublishTime > 1s)) {
		auto pathMsg = chunkToPath(chunk, trajTimeOffset);
		auto trajMsg = chunkToTrajectory(chunk);
		pathPub.publish(pathMsg);
		trajPub.publish(trajMsg);
		lastPublishedChunk = chunk;
		lastPublishTime = now;
	}
}


void TrajectoryPublisher::handleStart() {
	using std::chrono::steady_clock;
	startTime.reset(new steady_clock::time_point(steady_clock::now()));
}


TrajectoryPublisher::Chunk TrajectoryPublisher::extractChunk(
	std::chrono::steady_clock::duration beginTime,
	std::chrono::steady_clock::duration endTime)
{
	using std::chrono::steady_clock;
	auto ret = Chunk(traj->points.cbegin(), traj->points.cend());

	ret.first = std::upper_bound(traj->points.cbegin(), traj->points.cend(), beginTime,
		[](const steady_clock::duration& dur, const Trajectory::TrajectoryPoint& pt) {
			return pt.getTime() > dur;
		});
	if (ret.first == traj->points.cend()) {
		return ret;
	}
	ret.second = std::upper_bound(ret.first, traj->points.cend(), endTime,
		[](const steady_clock::duration& dur, const Trajectory::TrajectoryPoint& pt) {
			return pt.getTime() > dur;
		}
	);
	return ret;
}

nav_msgs::msg::Path TrajectoryPublisher::chunkToPath(
	Chunk chunk,
	std::chrono::steady_clock::time_point relTimeOffset)
{
	nav_msgs::msg::Path ret;
	ret.header.frame_id = "map";
	auto rosTimeOffset = rclcpp::Time(relTimeOffset.time_since_epoch().count());
	ret.header.stamp = rosTimeOffset;

	std::transform(chunk.first, chunk.second, std::back_inserter(ret.poses),
		[&](const Trajectory::TrajectoryPoint& pt){
			geometry_msgs::msg::PoseStamped pose;
			pose.header.stamp = rosTimeOffset + rclcpp::Duration(pt.getTime());
			pose.pose.position.x = pt.getXCoord();
			pose.pose.position.y = pt.getYCoord();
			pose.pose.position.z = pt.getZCoord();
			tf2::Quaternion q;
			q.setRPY(0, 0, pt.getHeading());
			tf2::convert(q, pose.pose.orientation);
			return pose;
		}
	);
	// Force same coordinate frame as header
	for (auto& pose : ret.poses) {
		pose.header.frame_id = ret.header.frame_id;
	}
	return ret;
}


atos_interfaces::msg::CartesianTrajectory TrajectoryPublisher::chunkToTrajectory(Chunk chunk) {
	atos_interfaces::msg::CartesianTrajectory trajMsg;
		
		// TODO: Can this be made from a ATOS::Trajectory instead?
		for (auto it = chunk.first; it != chunk.second; ++it) {
			auto point = *it;
			atos_interfaces::msg::CartesianTrajectoryPoint pointMsg;
			// Time
			auto millis = point.getTime().count();
			pointMsg.time_from_start.sec = millis/1000;
			pointMsg.time_from_start.nanosec = (millis%1000)*1000000;

			// Position
			pointMsg.pose.position.x = point.getPosition().x();
			pointMsg.pose.position.y = point.getPosition().y();
			pointMsg.pose.position.z = point.getPosition().z();

			// Rotation
			tf2::Quaternion q;
			q.setRPY(0, 0, point.getHeading());
			tf2::convert(q, pointMsg.pose.orientation);

			// Velocity TODO convert longitudinal / lateral into xyz coordinate system
			pointMsg.twist.linear.x = point.getLongitudinalVelocity();
			pointMsg.twist.linear.y = point.getLateralVelocity();
			pointMsg.twist.linear.z = 0; // TODO: Support for drones etc..

			// Acceleration TODO convert longitudinal / lateral into xyz coordinate system
			pointMsg.acceleration.linear.x = point.getLongitudinalAcceleration();
			pointMsg.acceleration.linear.y = point.getLateralAcceleration();
			pointMsg.acceleration.linear.z = 0; // TODO: Support for drones etc..

			trajMsg.points.push_back(pointMsg);
		}

		return trajMsg;
}
