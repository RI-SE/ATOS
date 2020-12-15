#include "geofence.hpp"
#include "util.h"

bool Geofence::forbids(const CartesianPosition &position) const {
	return !permits(position);
}

bool Geofence::permits(const CartesianPosition &position) const {
	char isInPolygon = UtilIsPointInPolygon(position, polygonPoints.data(),
											static_cast<unsigned int>(polygonPoints.size()));
	if (isInPolygon < 0) {
		throw std::invalid_argument("No points in geofence");
	}

	if ((isPermitted && isInPolygon)
		|| (!isPermitted && !isInPolygon)) {
			return true;
	}
	return false;
}
