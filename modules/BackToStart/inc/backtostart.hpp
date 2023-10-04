/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <unordered_map>
#include "module.hpp"
#include "trajectory.hpp"
#include "atos_interfaces/srv/get_object_return_trajectory.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "roschannels/commandchannels.hpp"
#include "roschannels/pathchannel.hpp"
#include "roschannels/gnsspathchannel.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

/*!
 * \brief The BackToStart class offers services to calculate a trajectory to return test objects to start position.
 */
class BackToStart : public Module {
public:
	BackToStart();

private:
	static inline std::string const moduleName = "back_to_start";
	
	// atos_interfaces::srv::GetTestOrigin::Response::SharedPtr originResponse; //!< Response of origin request
	geographic_msgs::msg::GeoPoint origin_pos; //!< Test origin

	rclcpp::Service<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr getObjectReturnTrajectoryService; //!< Service to request object return trajectory
    rclcpp::Client<atos_interfaces::srv::GetTestOrigin>::SharedPtr originClient; //!< Client to request test origin

	void onExitMessage(const ROSChannels::ResetTest::message_type::SharedPtr) override;
    void onReturnTrajectoryRequest(const std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Request>,
                            std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Response>);
    // atos_interfaces::msg::CartesianTrajectory cutOffTraj(const CartesianTrajectory& traj, const CartesianPosition& point);

	ROSChannels::Exit::Sub exitSub;
	std::unordered_map<uint32_t,ROSChannels::Path::Pub> pathPublishers;
	std::unordered_map<uint32_t,ROSChannels::GNSSPath::Pub> gnssPathPublishers;
};