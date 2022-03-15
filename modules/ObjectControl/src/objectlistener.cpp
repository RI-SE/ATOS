#include "objectlistener.hpp"
#include "objectcontrol.hpp"
#include "datadictionary.h"
#include "maestroTime.h"
#include "iso22133.h"
#include "journal.h"
#include <eigen3/Eigen/Dense>
#include <csignal>


static ObjectMonitorType transformCoordinate(const ObjectMonitorType& point, const ObjectMonitorType& anchor, const bool debug = false);

ObjectListener::ObjectListener(
		ObjectControl* sh,
		TestObject* ob,
		rclcpp::Logger log)
	:  obj(ob), handler(sh), Loggable(log)
{
	if (!obj->isConnected()) {
		throw std::invalid_argument("Attempted to start listener for disconnected object");
	}
	RCLCPP_DEBUG(get_logger(), "Starting listener thread for object %u", ob->getTransmitterID());
	listener = std::thread(&ObjectListener::listen, this);
}

ObjectListener::~ObjectListener() {
	this->quit = true;
	RCLCPP_DEBUG(get_logger(), "Awaiting thread exit");
	pthread_cancel(listener.native_handle());
	listener.join(); // TODO this blocks if MONR timeout (still?)
	RCLCPP_DEBUG(get_logger(), "Thread exited");
}

void ObjectListener::listen() {
	try {
		while (!this->quit) {
			switch (obj->pendingMessageType(true)) {
			case MESSAGE_ID_MONR: {
				struct timeval currentTime;
				auto prevObjState = obj->getState();
				auto monr = obj->readMonitorMessage();
				TimeSetToCurrentSystemTime(&currentTime);
				if (handler->controlMode == ObjectControl::RELATIVE_KINEMATICS && !obj->isAnchor()) {
					monr.second = transformCoordinate(monr.second, handler->getLastAnchorData(), true);
				}

				// Disable thread cancelling while accessing shared memory and journals
				int oldCancelState;
				pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &oldCancelState);
				// Save to memory
				DataDictionarySetMonitorData(monr.first, &monr.second, &currentTime);
				auto objData = obj->getAsObjectData();
				objData.MonrData = monr.second;
				JournalRecordMonitorData(&objData);
				// Reset thread cancelling
				pthread_setcancelstate(oldCancelState, nullptr);

				// Check if state has changed
				if (obj->getState() != prevObjState) {
					switch (obj->getState()) {
					case OBJECT_STATE_DISARMED:
						if (prevObjState == OBJECT_STATE_ABORTING) {
							RCLCPP_INFO(get_logger(), "Object %u abort cleared", obj->getTransmitterID());
							handler->state->objectAbortDisarmed(*handler, obj->getTransmitterID());
						}
						else {
							RCLCPP_INFO(get_logger(), "Object %u disarmed", obj->getTransmitterID());
							handler->state->objectDisarmed(*handler, obj->getTransmitterID());
						}
						break;
					case OBJECT_STATE_POSTRUN:
						break;
					case OBJECT_STATE_ARMED:
						RCLCPP_INFO(get_logger(), "Object %u armed", obj->getTransmitterID());
						handler->state->objectArmed(*handler, obj->getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
						RCLCPP_INFO(get_logger(), "Object %u aborting", obj->getTransmitterID());
						handler->state->objectAborting(*handler, obj->getTransmitterID());
						break;
					}
				}

				handler->injectObjectData(monr);

				break;
			}
			case MESSAGE_ID_TREO:
				RCLCPP_WARN(get_logger(), "Unhandled TREO message");
				break;
			case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO:
				obj->parseObjectPropertyMessage();
				break;
			default:
				RCLCPP_WARN(get_logger(), "Received unknown message type");
				break;
			}
		}
	} catch (std::invalid_argument& e) {
		RCLCPP_ERROR(get_logger(), e.what());
	} catch (std::runtime_error& e) {
		RCLCPP_ERROR(get_logger(), e.what());
		obj->disconnect();
		handler->state->disconnectedFromObject(*handler, obj->getTransmitterID());
	}
	RCLCPP_INFO(get_logger(), "Listener thread for object %u exiting", obj->getTransmitterID());
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
		LogPrint(dbg.str().c_str());
	}
	return retval;
}
