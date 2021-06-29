#include "objectlistener.hpp"
#include "datadictionary.h"
#include "maestroTime.h"
#include "iso22133.h"
#include "journal.h"
#include <eigen3/Eigen/Dense>
#include <csignal>


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
	pthread_cancel(listener.native_handle());
	listener.join(); // TODO this blocks if MONR timeout (still?)
	LogMessage(LOG_LEVEL_DEBUG, "Thread exited");
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
				if (handler->controlMode == ScenarioHandler::RELATIVE_KINEMATICS && !obj->isAnchor()) {
					monr.second = transformCoordinate(monr.second, handler->getLastAnchorData());
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
						LogMessage(LOG_LEVEL_INFO, "Object %u disarmed", obj->getTransmitterID());
						handler->state->objectDisarmed(*handler, obj->getTransmitterID());
						break;
					case OBJECT_STATE_POSTRUN:
						break;
					case OBJECT_STATE_ARMED:
						LogMessage(LOG_LEVEL_INFO, "Object %u armed", obj->getTransmitterID());
						handler->state->objectArmed(*handler, obj->getTransmitterID());
						break;
					case OBJECT_STATE_ABORTING:
						LogMessage(LOG_LEVEL_INFO, "Object %u aborting", obj->getTransmitterID());
						handler->state->objectAborting(*handler, obj->getTransmitterID());
						break;
					}
				}

				//OSI sending here
				//TODO: Use ObjectSettingClass

				for(int i = 0; i < handler->getVehicleIDs().size(); i ++){
					if(obj->getTransmitterID() == handler->dataInjectionMaps[i].sourceID)
					{
						std::vector<char> outBuffer = 
						handler->buildOSIgogtArray(obj->getTransmitterID());
						for(int j = 0; j < handler->dataInjectionMaps[i].numberOfTargets; j ++){
						    obj->sendOsiData(outBuffer);
		  				}
		  			}
		  		}

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
	LogMessage(LOG_LEVEL_INFO, "Listener thread for object %u exiting", obj->getTransmitterID());
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
