#include "CRSTransformation.hpp"

/**
 * @brief Construct a new CRSTransformation::CRSTransformation object to create a pipeline
 * 				between two known coordinate reference systems
 * 
 * @param fromCRS Coordinate reference system from
 * @param toCRS Coordinate reference system to
 */
CRSTransformation::CRSTransformation(const std::string &fromCRS, const std::string &toCRS) :
 ctxt(proj_context_create(), [](PJ_CONTEXT* ctxt){ proj_context_destroy(ctxt); }),
	projection(proj_create_crs_to_crs(ctxt.get(), fromCRS.c_str(), toCRS.c_str(), nullptr),
		[](PJ* proj){ proj_destroy(proj); })
{
	if (projection == nullptr) {
		throw std::logic_error("Failed to create CRS conversion from " + fromCRS
			+ " to " + toCRS + ": " + proj_context_errno_string(ctxt.get(),
				proj_context_errno(ctxt.get())));
	}
}

/**
 * @brief Apply transformation from fromCRS to toCRS
 * 
 * @param traj reference to trajectory to transform
 * @return Eigen::Vector3d Transformed point
 */
void CRSTransformation::apply(std::vector<ATOS::Trajectory::TrajectoryPoint> &trajPoints) {
	// Put TrajPoints into array of PJ_POINTS
	PJ_COORD in[trajPoints.size()];
	auto arraySize = sizeof(in) / sizeof(in[0]);
	RCLCPP_DEBUG(logger, "Putting trajectory with %d points into PJ_COORD array", arraySize);
	for (int i = 0; i < arraySize; i++){
		in[i].xyz.x = trajPoints[i].getXCoord();
		in[i].xyz.y = trajPoints[i].getYCoord();
		in[i].xyz.z = trajPoints[i].getZCoord();
	}

	RCLCPP_DEBUG(logger, "Converting trajectory with %d points", arraySize);
	if (0 != proj_trans_array(projection.get(), PJ_FWD, arraySize, in)){
		throw std::runtime_error("Failed to convert trajectory");
	}

	// Put transformed PJ_POINTS back into TrajPoints
	for (int i = 0; i < arraySize; i++){
		trajPoints[i].setXCoord(in[i].xyz.x);
		trajPoints[i].setYCoord(in[i].xyz.y);
		trajPoints[i].setZCoord(in[i].xyz.z);
	}
}

/**
 * @brief Get the origin lat, lon, height in the given datum from a proh string
 * 
 * @parameter: projString proj string to transform
 * @parameter: datum datum to transform to 
 * @return std::vector<double> Vector with lat, lon, height
 */
const std::vector<double>
CRSTransformation::projToLLH(const std::string &projString, const std::string &datum) {
    auto pjSrc = proj_create_crs_to_crs(NULL, projString.c_str(), datum.c_str(), NULL);
    if (pjSrc == NULL) {
        throw std::runtime_error("Failed to create projPJ object");
    }

    PJ_COORD src = proj_coord(0, 0, 0, 0);
    PJ_COORD dst = proj_trans(pjSrc, PJ_FWD, src);

	std::vector<double> result = {dst.xyz.x, dst.xyz.y, dst.xyz.z};
    proj_destroy(pjSrc);

    return result;
}