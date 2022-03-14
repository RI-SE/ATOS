#include "state.hpp"

AbstractKinematics::RemoteControlled::RemoteControlled() {

}


void AbstractKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
}

void AbstractKinematics::RemoteControlled::sendControlSignal(
		ObjectControl& handler,
		const ControlSignalPercentage::SharedPtr csp) {
	// Translate ROS struct to RCMM
	RemoteControlManoeuvreMessageType rcmmMessage;
	rcmmMessage.command = MANOEUVRE_NONE;
	rcmmMessage.isThrottleManoeuvreValid = true;
	rcmmMessage.isBrakeManoeuvreValid = true;
	rcmmMessage.isSteeringManoeuvreValid = true;
	rcmmMessage.throttleUnit = ISO_UNIT_TYPE_THROTTLE_PERCENTAGE;
	rcmmMessage.brakeUnit = ISO_UNIT_TYPE_BRAKE_PERCENTAGE;
	rcmmMessage.steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
	rcmmMessage.throttleManoeuvre.pct = csp->throttle;
	rcmmMessage.brakeManoeuvre.pct = csp->brake;
	rcmmMessage.steeringManoeuvre.pct = csp->steering_angle;
	handler.sendRCMMToObject(rcmmMessage,csp->maestro_header.object_id);
}

/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
	setState(handler, new RelativeKinematics::Ready);
}
void RelativeKinematics::RemoteControlled::sendControlSignal(
		ObjectControl& handler,
		const ControlSignalPercentage::SharedPtr csp) {
	AbstractKinematics::RemoteControlled::sendControlSignal(handler,csp);
}

/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::RemoteControlled::disableRemoteControlRequest(
		ObjectControl& handler) {
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::RemoteControlled::sendControlSignal(
		ObjectControl& handler,
		const ControlSignalPercentage::SharedPtr csp) {
	AbstractKinematics::RemoteControlled::sendControlSignal(handler,csp);
}

