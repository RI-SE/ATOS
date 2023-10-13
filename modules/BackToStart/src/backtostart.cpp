/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "backtostart.hpp"
using std::placeholders::_1, std::placeholders::_2;

BackToStart::BackToStart() : Module(BackToStart::moduleName)
{
	getObjectReturnTrajectoryService = create_service<atos_interfaces::srv::GetObjectReturnTrajectory>(ServiceNames::getObjectReturnTrajectory,
		std::bind(&BackToStart::onReturnTrajectoryRequest, this, _1, _2));
}


void BackToStart::onReturnTrajectoryRequest(const std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Request> request, 
                                            std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Response> response) {
    
    if (request->trajectory.points.size() == 0) {
        RCLCPP_ERROR(get_logger(), "Received empty trajectory");
        response->success = false;
        return;
    }
    // Load the trajectory in a Trajectory object
    ATOS::Trajectory currentTraj(get_logger());
    currentTraj.initializeFromCartesianTrajectory(request->trajectory);
    uint32_t id = request->id;

    // Create a new trajectory object for the return trajectory
    ATOS::Trajectory b2sTraj(get_logger());

    //Add first turn
    ATOS::Trajectory turn1 = ATOS::Trajectory::createWilliamsonTurn(5, 1, currentTraj.points.back());
    b2sTraj.points.insert(b2sTraj.points.end(), std::begin(turn1.points), turn1.points.end());

    //Add reversed original traj
    auto rev = currentTraj.reversed().delayed(b2sTraj.points.back().getTime());
    b2sTraj.points.insert(b2sTraj.points.end(), std::begin(rev.points), rev.points.end());

    //Add last turn
    ATOS::Trajectory turn2 = ATOS::Trajectory::createWilliamsonTurn(5, 1, b2sTraj.points.back());
    turn2 = turn2.delayed(b2sTraj.points.back().getTime());
    b2sTraj.points.insert(b2sTraj.points.end(), std::begin(turn2.points), turn2.points.end());

    // Fill the response message
    response->id = id;
    response->trajectory = b2sTraj.toCartesianTrajectory();
    response->success = true;
    RCLCPP_INFO(get_logger(), "Calculated return trajectory for object %u with %lu points", response->id, response->trajectory.points.size());
};
