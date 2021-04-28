#include "objectlistener.hpp"
#include "datadictionary.h"
#include "maestroTime.h"
#include "iso22133.h"
#include <eigen3/Eigen/Dense>


static ObjectMonitorType transformCoordinate(const ObjectMonitorType& point, const ObjectMonitorType& anchor);

ObjectListener::ObjectListener(
		ScenarioHandler* sh,
		TestObject* ob)
	:  obj(ob), handler(sh)
{
	if (!obj->isConnected()) {
		throw std::invalid_argument("Attempted to start listener for disconnected object");
	}
	LogMessage(LOG_LEVEL_DEBUG, "Starting listener thread for object %u", ob->getTransmitterID());
	listener = std::thread(&ObjectListener::listen, this);
}

ObjectListener::~ObjectListener() {
	this->quit = true;
	LogMessage(LOG_LEVEL_DEBUG, "Awaiting thread exit");
	listener.join(); // TODO this blocks if MONR timeout
}

void ObjectListener::listen() {
	try {
		while (!this->quit) {
			switch (obj->pendingMessageType(true)) {
			case MESSAGE_ID_MONR: {
				struct timeval currentTime;
				auto monr = obj->readMonitorMessage();
				TimeSetToCurrentSystemTime(&currentTime);
				if (handler->controlMode == ScenarioHandler::RELATIVE_KINEMATICS && !obj->isAnchor()) {
					monr.second = transformCoordinate(monr.second, handler->getLastAnchorData());
				}
				else {
					// TODO
				}
				DataDictionarySetMonitorData(monr.first, &monr.second, &currentTime); // TODO move to abs_kin
				break;
			}
			case MESSAGE_ID_TREO:
				LogMessage(LOG_LEVEL_WARNING, "Unhandled TREO message");
				break;
			case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO:
				obj->parseObjectPropertyMessage();
				break;
			default:
				LogMessage(LOG_LEVEL_WARNING, "Received unknown message type");
				break;
			}
		}
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, e.what());
	} catch (std::ios_base::failure& e) {
		LogMessage(LOG_LEVEL_ERROR, e.what());
		obj->disconnect();
		handler->state->disconnectedFromObject(*handler, obj->getTransmitterID());
	}
}

/*!
 * \brief transformCoordinate Transforms monitor data in positions/speeds relative to
 *			an anchor point into being relative to the anchor points reference.
 * \param point Monitor data to be transformed
 * \param anchor Monitor data of anchor to which point is relative
 * \return Transformed monitor data
 */
ObjectMonitorType transformCoordinate(
		const ObjectMonitorType& point,
		const ObjectMonitorType& anchor) {
	using namespace Eigen;
	ObjectMonitorType retval = point;
	struct timeval tvdiff = {0, 0};
	std::chrono::milliseconds diff;
	if (point.isTimestampValid && anchor.isTimestampValid) {
		timersub(&point.timestamp, &anchor.timestamp, &tvdiff);
	}
	from_timeval(tvdiff, diff);

	AngleAxis anchorToGlobal(anchor.position.heading_rad, Vector3d::UnitZ());
	AngleAxis pointToGlobal(anchor.position.heading_rad+point.position.heading_rad, Vector3d::UnitZ());
	Vector3d pointPos(point.position.xCoord_m, point.position.yCoord_m, point.position.zCoord_m);
	Vector3d anchorPos(anchor.position.xCoord_m, anchor.position.yCoord_m, anchor.position.zCoord_m);
	Vector3d pointVel(point.speed.longitudinal_m_s, point.speed.lateral_m_s, 0.0);
	Vector3d anchorVel(anchor.speed.longitudinal_m_s, anchor.speed.lateral_m_s, 0.0);
	Vector3d pointAcc(point.acceleration.longitudinal_m_s2, point.acceleration.lateral_m_s2, 0.0);
	Vector3d anchorAcc(anchor.acceleration.longitudinal_m_s2, anchor.acceleration.lateral_m_s2, 0.0);

	// TODO check on timestamps
	//std::cout << "pt: " << pointPos << std::endl << "anch: " << anchorPos <<std::endl;
	Vector3d pointInAnchorFrame = anchorPos + anchorToGlobal*pointPos; // TODO subtract also anchor velocity vector times tdiff
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
	retval.position.heading_rad = anchor.position.heading_rad + point.position.heading_rad;
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
	return retval;
}
