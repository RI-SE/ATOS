#include "state.hpp"

AbstractKinematics::Ready::Ready() {

}

void AbstractKinematics::Ready::onEnter(
		ObjectControl& handler) {
	handler.startSafetyThread();
}

void AbstractKinematics::Ready::armRequest(
		ObjectControl& handler) {
	if (!handler.areAllObjectsIn(std::set({OBJECT_STATE_DISARMED, OBJECT_STATE_ARMED}))) {
		throw std::invalid_argument("Not all objects in valid state prior to arm");
	}
}

void AbstractKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	handler.disconnectObjects();
}

void AbstractKinematics::Ready::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t) {
	// TODO
}

void AbstractKinematics::Ready::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	// TODO
}

void AbstractKinematics::Ready::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
		// TODO
}

void AbstractKinematics::Ready::settingModificationRequested(
		ObjectControl& handler) {
	// TODO
}
void AbstractKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& handler){
}

/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Ready::armRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::armRequest(handler);
	setState(handler, new RelativeKinematics::Armed);
}

void RelativeKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}

void RelativeKinematics::Ready::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Connecting);
}

void RelativeKinematics::Ready::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::objectAborting(handler,id);
	setState(handler, new RelativeKinematics::Aborting);
}


void RelativeKinematics::Ready::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Ready::objectAbortDisarmed(handler,id);
}

void RelativeKinematics::Ready::settingModificationRequested(
		ObjectControl& handler) {
	AbstractKinematics::Ready::settingModificationRequested(handler);
	// TODO call ObjectControl to update OSEM if modified etc.
}

void RelativeKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& handler){
	setState(handler, new RelativeKinematics::RemoteControlled);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Ready::armRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::armRequest(handler);
	setState(handler, new AbsoluteKinematics::Armed);
}

void AbsoluteKinematics::Ready::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Ready::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Ready::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Ready::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::Ready::objectAborting(handler,id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Ready::objectAbortDisarmed(
	ObjectControl& handler,
	uint32_t id) {
	AbstractKinematics::Ready::objectAbortDisarmed(handler,id);
}

void AbsoluteKinematics::Ready::settingModificationRequested(
		ObjectControl& handler) {
	AbstractKinematics::Ready::settingModificationRequested(handler);
	// TODO call ObjectControl to update OSEM if modified etc.
}

void AbsoluteKinematics::Ready::enableRemoteControlRequest(
	ObjectControl& handler){
	setState(handler, new AbsoluteKinematics::RemoteControlled);
}
