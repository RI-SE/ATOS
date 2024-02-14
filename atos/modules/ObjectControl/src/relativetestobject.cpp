/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "relativetestobject.hpp"
#include "atosTime.h"
#include <cstdint>
#include <functional>

using std::placeholders::_1;

RelativeTestObject::RelativeTestObject(uint32_t id) : 
    TestObject(id),
	anchorSub(*this, std::bind(&RelativeTestObject::updateAnchor, this, _1))
{
}

RelativeTestObject::RelativeTestObject(RelativeTestObject&& other) :
	TestObject(std::move(other)),
	anchorSub(std::move(other.anchorSub))
{
}

/*! \brief updateAnchor Updates the position of the anchor object
 * \param monr Monitor data of anchor point
 */
void RelativeTestObject::updateAnchor(const ROSChannels::Monitor::message_type::SharedPtr monr) {
	lastAnchorMonr = ROSChannels::Monitor::toISOMonr(*monr);
}

/*!
 * \brief RelativeTestObject Transforms monitor data in positions/speeds relative to 
    *			an anchor point into being relative to the anchor points reference.
 * \param point Monitor data to be transformed
 * \param anchor Monitor data of anchor to which point is relative
 * \return Transformed monitor data
 */
MonitorMessage RelativeTestObject::readMonitorMessage() {
	MonitorMessage retval;
	this->comms.mntr >> retval;
	lastMonitorTime = clock::now();
	updateMonitor(retval);
	// transform the monitor data relative to anchor
	transformCoordinate(retval.second,lastAnchorMonr,true);
	return retval;
}


/*!
 * \brief transformCoordinate Transforms monitor data in positions/speeds relative to
 *			an anchor point into being relative to the anchor points reference.
 * \param point Monitor data to be transformed
 * \param anchor Monitor data of anchor to which point is relative
 * \return Transformed monitor data
 */
ObjectMonitorType RelativeTestObject::transformCoordinate(
		const ObjectMonitorType& point,
		const ObjectMonitorType& anchor,
		const bool debug) {
	using namespace Eigen;
	using namespace std::chrono;
	static auto nextPrintTime = steady_clock::now();
	static constexpr auto printInterval = seconds(5);
	bool print = false;
	if (steady_clock::now() > nextPrintTime) {
		print = debug;
		nextPrintTime = steady_clock::now() - nextPrintTime > printInterval ?
					steady_clock::now() + printInterval
				  : nextPrintTime + printInterval;
	}
	std::stringstream dbg;

	ObjectMonitorType retval = point;
	struct timeval tvdiff = {0, 0};
	std::chrono::milliseconds diff;
	if (point.isTimestampValid && anchor.isTimestampValid) {
		timersub(&point.timestamp, &anchor.timestamp, &tvdiff);
	}
	from_timeval(tvdiff, diff);

	AngleAxis anchorToGlobal(anchor.position.heading_rad, Vector3d::UnitZ());
	AngleAxis pointToGlobal(point.position.heading_rad, Vector3d::UnitZ());
	Vector3d pointPos(point.position.xCoord_m, point.position.yCoord_m, point.position.zCoord_m);
	Vector3d anchorPos(anchor.position.xCoord_m, anchor.position.yCoord_m, anchor.position.zCoord_m);
	Vector3d pointVel(point.speed.longitudinal_m_s, point.speed.lateral_m_s, 0.0);
	Vector3d anchorVel(anchor.speed.longitudinal_m_s, anchor.speed.lateral_m_s, 0.0);
	Vector3d pointAcc(point.acceleration.longitudinal_m_s2, point.acceleration.lateral_m_s2, 0.0);
	Vector3d anchorAcc(anchor.acceleration.longitudinal_m_s2, anchor.acceleration.lateral_m_s2, 0.0);

	if (print) {
		dbg << std::endl
			<< "pt:   " << pointPos << ", " << point.position.heading_rad*180.0/M_PI << "deg" << std::endl
			<< "anch: " << anchorPos << ", " << anchor.position.heading_rad*180.0/M_PI << "deg" << std::endl;
	}

	// TODO check on timestamps
	Vector3d pointInAnchorFrame = anchorPos + pointPos; // TODO subtract also anchor velocity vector times tdiff
	anchorVel = anchorToGlobal*anchorVel; // To x/y speeds
	pointVel = pointToGlobal*pointVel + anchorVel; // To x/y speeds
	pointVel = pointToGlobal.inverse()*pointVel; // To long/lat speeds
	anchorAcc = anchorToGlobal*anchorAcc; // To x/y accelerations
	pointAcc = pointToGlobal*pointAcc + anchorAcc; // To x/y accelerations
	pointAcc = pointToGlobal.inverse()*pointAcc; // To long/lat accelerations

	retval.position.xCoord_m = pointInAnchorFrame[0];
	retval.position.yCoord_m = pointInAnchorFrame[1];
	retval.position.zCoord_m = pointInAnchorFrame[2];
	retval.position.isPositionValid = anchor.position.isPositionValid && anchor.position.isHeadingValid
			&& point.position.isPositionValid;
	retval.position.heading_rad = point.position.heading_rad;
	retval.position.isHeadingValid = anchor.position.isHeadingValid && point.position.isHeadingValid;
	retval.speed.longitudinal_m_s = pointVel[0];
	retval.speed.lateral_m_s = pointVel[1];
	retval.speed.isLateralValid = retval.speed.isLongitudinalValid =
			anchor.speed.isLateralValid && anchor.speed.isLongitudinalValid
			&& point.speed.isLateralValid && point.speed.isLongitudinalValid;
	retval.acceleration.longitudinal_m_s2 = pointAcc[0];
	retval.acceleration.lateral_m_s2 = pointAcc[1];
	retval.acceleration.isLateralValid = retval.acceleration.isLongitudinalValid =
			anchor.acceleration.isLateralValid && anchor.acceleration.isLongitudinalValid
			&& point.acceleration.isLateralValid && point.acceleration.isLongitudinalValid;
	if (print) {
		dbg << "res:  " << pointInAnchorFrame << ", " << retval.position.heading_rad*180.0/M_PI << "deg";
		std::cerr << dbg.str();
	}
	return retval;
}
