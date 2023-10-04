/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "backtostart.hpp"
using std::placeholders::_1, std::placeholders::_2;

BackToStart::BackToStart() : Module(BackToStart::moduleName), 
	exitSub(*this, std::bind(&BackToStart::onExitMessage, this, _1))
{
	getObjectReturnTrajectoryService = create_service<atos_interfaces::srv::GetObjectReturnTrajectory>(ServiceNames::getObjectReturnTrajectory,
		std::bind(&BackToStart::onReturnTrajectoryRequest, this, _1, _2));
	originClient = create_client<atos_interfaces::srv::GetTestOrigin>(ServiceNames::getTestOrigin);
}
void BackToStart::onExitMessage(const ROSChannels::Exit::message_type::SharedPtr)
{
    RCLCPP_INFO(get_logger(), "Received exit message");
    rclcpp::shutdown();
}


void BackToStart::onReturnTrajectoryRequest(const std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Request> request, 
                                            std::shared_ptr<atos_interfaces::srv::GetObjectReturnTrajectory::Response> response) {
    ATOS::Trajectory currentTraj(get_logger());
    currentTraj.initializeFromCartesianTrajectory(request->trajectory);
    uint32_t id = request->id;
    // object_position = request->position;

    // auto trajShortened = cutOffTraj(request->trajectory, object_position);
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

    response->id = id;
    response->trajectory = b2sTraj.toCartesianTrajectory();
    response->success = true;
    RCLCPP_INFO(get_logger(), "Calculated return trajectory for object %u with %lu points", response->id, response->trajectory.points.size());

    this->pathPublishers.emplace(id, ROSChannels::Path::Pub(*this, id));
    this->pathPublishers.at(id).publish(b2sTraj.toPath());

    // Get test origin
    auto originCallback = [id, b2sTraj, this](const rclcpp::Client<atos_interfaces::srv::GetTestOrigin>::SharedFuture future) {
        auto originResponse = future.get();
        if (!originResponse->success) {
            RCLCPP_ERROR(get_logger(), "Get origin service call failed for object %u", id);
            return;
        }
        auto origin_pos = originResponse->origin.position;
        RCLCPP_INFO(get_logger(), "Loaded test origin for object. lat: %f, lon: %f, alt: %f", 
                    origin_pos.latitude, origin_pos.longitude, origin_pos.altitude);
        std::array<double,3> llh_0 = {origin_pos.latitude, origin_pos.longitude, origin_pos.altitude};
        this->gnssPathPublishers.emplace(id, ROSChannels::GNSSPath::Pub(*this, id));
        this->gnssPathPublishers.at(id).publish(b2sTraj.toGeoJSON(llh_0));
    };
    auto originRequest = std::make_shared<atos_interfaces::srv::GetTestOrigin::Request>();
    auto originPromise = originClient->async_send_request(originRequest, originCallback);
};


// Need to double check this function
/*
using namespace atos_interfaces::msg;
CartesianTrajectory BackToStart::cutOffTraj(const CartesianTrajectory& traj, const CartesianPosition& point) {
    CartesianTrajectory trajShortened;
    auto it = std::find_if(traj.begin(), traj.end(), [point](const CartesianPoint& p) {
        return p.getTime() > point.getTime();
    });
    trajShortened.insert(trajShortened.end(), it, traj.end());
    return trajShortened;
}
*/