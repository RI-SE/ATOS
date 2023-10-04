#pragma once

#include "proj.h"
#include "trajectory.hpp"
#include "rclcpp/logger.hpp"

class CRSTransformation {

  public:

    CRSTransformation(const std::string &fromCRS, const std::string &toCRS);
    void apply(std::vector<ATOS::Trajectory::TrajectoryPoint> &traj);
    static const std::vector<double> projToLLH(const std::string &projString, const std::string &datum);

  private:

    std::unique_ptr<PJ_CONTEXT, std::function<void(PJ_CONTEXT*)>> ctxt;
	  std::unique_ptr<PJ, std::function<void(PJ*)>> projection;
    rclcpp::Logger logger = rclcpp::get_logger("CRSTransformation");

};
