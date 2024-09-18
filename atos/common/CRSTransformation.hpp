/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "proj.h"
#include "trajectory.hpp"
#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/point.hpp"

class CRSTransformation {

  public:

    CRSTransformation(const std::string &fromCRS, const std::string &toCRS);
    void apply(std::vector<ATOS::Trajectory::TrajectoryPoint> &traj);
    void apply(geometry_msgs::msg::Point &point, PJ_DIRECTION direction);
    static std::vector<double> projToLLH(const std::string &projString, const std::string &datum);
    static void llhOffsetMeters(double *llh, const double *xyzOffset);

  private:

    std::unique_ptr<PJ_CONTEXT, std::function<void(PJ_CONTEXT*)>> ctxt;
	  std::unique_ptr<PJ, std::function<void(PJ*)>> projection;
    rclcpp::Logger logger = rclcpp::get_logger("CRSTransformation");

};
