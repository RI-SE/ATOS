#include "coordinateutils.hpp"
#include <math.h>

/**
 * @brief Returns a new latitude, longitude, height after offsetting x, y, z meters
 * 
 * @param llh The latitude, longitude, height [degrees, degrees, meters]
 * @param xyzOffset Meters offset from llh [meters, meters, meters]
 */
// TODO: remake as c++ function
void llhOffsetMeters(double *llh, const double *xyzOffset) {
    constexpr double EARTH_EQUATOR_RADIUS_M = 6378137.0;	// earth semimajor axis (WGS84) (m)
	const double lat = llh[0];
	const double lon = llh[1];
	const double hgt = llh[2];

	const double dx = xyzOffset[0];
	const double dy = xyzOffset[1];
	const double dz = xyzOffset[2];

	llh[0] = lat + (dy / EARTH_EQUATOR_RADIUS_M) * (180 / M_PI);
	llh[1] = lon + (dx / EARTH_EQUATOR_RADIUS_M) * (180 / M_PI) / cos(lat * M_PI / 180.0);
	llh[2] = hgt + dz;
}