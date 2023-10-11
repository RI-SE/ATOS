/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "trajectory.hpp"
#include "atos_interfaces/srv/get_object_return_trajectory.hpp"
#include "atos_interfaces/srv/get_test_origin.hpp"
#include "roschannels/commandchannels.hpp"

/*!
 * \brief The BackToStart class offers services to calculate a trajectory to return test objects to start position.
 */
class BackToStart : public Module {
public:
	BackToStart();

private:
	static inline std::string const moduleName = "back_to_start";

	rclcpp::Service<atos_interfaces::srv::GetObjectReturnTrajectory>::SharedPtr getObjectReturnTrajectoryService; //!< Service to request object return trajectory

    void onReturnTrajectoryRequest(const std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Request>,
                            std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Response>);
    // atos_interfaces::msg::CartesianTrajectory cutOffTraj(const CartesianTrajectory& traj, const CartesianPosition& point);

	ROSChannels::Exit::Sub exitSub;
};