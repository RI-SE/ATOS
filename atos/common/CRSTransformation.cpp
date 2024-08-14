/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "CRSTransformation.hpp"

/**
 * @brief Construct a new CRSTransformation::CRSTransformation object to create
 * a pipeline between two known coordinate reference systems
 *
 * @param fromCRS Coordinate reference system from
 * @param toCRS Coordinate reference system to
 */

CRSTransformation::CRSTransformation(const std::string &fromCRS,
                                     const std::string &toCRS)
    : ctxt(proj_context_create(),
           [](PJ_CONTEXT *ctxt) { proj_context_destroy(ctxt); }),
      projection(proj_create_crs_to_crs(ctxt.get(), fromCRS.c_str(),
                                        toCRS.c_str(), nullptr),
                 [](PJ *proj) { proj_destroy(proj); }) {
  if (projection == nullptr) {
    throw std::logic_error(
        "Failed to create CRS conversion from " + fromCRS + " to " + toCRS +
        ": " +
        proj_context_errno_string(ctxt.get(), proj_context_errno(ctxt.get())));
  }
}

/**
 * @brief Apply transformation from fromCRS to toCRS
 *
 * @param traj reference to trajectory to transform
 */
void CRSTransformation::apply(
    std::vector<ATOS::Trajectory::TrajectoryPoint> &trajPoints) {

  // Put TrajPoints into array of PJ_POINTS
  PJ_COORD in[trajPoints.size()];
  auto arraySize = sizeof(in) / sizeof(in[0]);
  for (int i = 0; i < arraySize; i++) {
    in[i].xyz.x = trajPoints[i].getXCoord();
    in[i].xyz.y = trajPoints[i].getYCoord();
    in[i].xyz.z = trajPoints[i].getZCoord();
    RCLCPP_INFO(logger, "Point %d: %f, %f, %f", i, in[i].xyz.x, in[i].xyz.y,
                in[i].xyz.z);
  }

  RCLCPP_DEBUG(logger, "Converting trajectory with %ld points", arraySize);
  if (0 != proj_trans_array(projection.get(), PJ_FWD, arraySize, in)) {
    throw std::runtime_error("Failed to convert trajectory");
  }

  // Put transformed PJ_POINTS back into TrajPoints
  for (int i = 0; i < arraySize; i++) {
    if (isnan(in[i].xyz.x) || isnan(in[i].xyz.y) || isnan(in[i].xyz.z) ) {
      // Apply transformation to the point again. quick fix. Need to understand
      // why some points are not transformed
      auto point = geometry_msgs::msg::Point();
      point.x = trajPoints[i].getXCoord();
      point.y = trajPoints[i].getYCoord();
      point.z = trajPoints[i].getZCoord();
      apply(point, PJ_FWD);
      in[i].xyz.x = point.x;
      in[i].xyz.y = point.y;
      in[i].xyz.z = point.z;
    }

    trajPoints[i].setXCoord(in[i].xyz.x);
    trajPoints[i].setYCoord(in[i].xyz.y);
    trajPoints[i].setZCoord(in[i].xyz.z);
    RCLCPP_INFO(logger, "Converted Point %d: %f, %f, %f", i, in[i].xyz.x,
                in[i].xyz.y, in[i].xyz.z);
  }
}

/**
 * @brief Apply a transform on a point
 *
 * @param point The point to transform
 * @param direction Which direction to transform, e.g. PJ_FWD or PJ_INV
 */
void CRSTransformation::apply(geometry_msgs::msg::Point &point,
                              PJ_DIRECTION direction) {
  PJ_COORD in = proj_coord(point.x, point.y, point.z, 0);
  PJ_COORD out = proj_trans(projection.get(), direction, in);
  point.x = out.xyz.x;
  point.y = out.xyz.y;
  point.z = out.xyz.z;
}

/**
 * @brief Get the origin lat, lon, height in the given datum from a proj string
 *
 * @parameter: projString proj string to transform
 * @parameter: datum datum to transform to
 * @return std::vector<double> Vector with lat, lon, height
 */
std::vector<double> CRSTransformation::projToLLH(const std::string &projString,
                                                 const std::string &datum) {
  auto *pjSrc =
      proj_create_crs_to_crs(NULL, projString.c_str(), datum.c_str(), NULL);
  if (pjSrc == NULL) {
    throw std::runtime_error("Failed to create projPJ object");
  }

  PJ_COORD src = proj_coord(0, 0, 0, 0);
  PJ_COORD dst = proj_trans(pjSrc, PJ_FWD, src);

  std::vector<double> result = {dst.xyz.x, dst.xyz.y, dst.xyz.z};
  proj_destroy(pjSrc);

  return result;
}

/**
 * @brief Returns a new latitude, longitude, height after offsetting x, y, z
 * meters
 *
 * @param llh The latitude, longitude, height [degrees, degrees, meters]
 * @param xyzOffset Meters offset from llh [meters, meters, meters]
 */
void CRSTransformation::llhOffsetMeters(double *llh, const double *xyzOffset) {
  constexpr double EARTH_EQUATOR_RADIUS_M =
      6378137.0; // earth semimajor axis (WGS84) (m)
  const auto [lat, lon, hgt] = std::make_tuple(llh[0], llh[1], llh[2]);
  const auto [dx, dy, dz] =
      std::make_tuple(xyzOffset[0], xyzOffset[1], xyzOffset[2]);

  llh[0] = lat + (dy / EARTH_EQUATOR_RADIUS_M) * (180 / M_PI);
  llh[1] = lon + (dx / EARTH_EQUATOR_RADIUS_M) * (180 / M_PI) /
                     std::cos(lat * M_PI / 180.0);
  llh[2] = hgt + dz;
}