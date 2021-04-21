#include "state.hpp"

ObjectControl::Connecting::Connecting() {

}

void ObjectControl::Connecting::onEnter(
		ScenarioHandler& handler) {
	handler.beginConnectionAttempt();
}

void ObjectControl::Connecting::disconnectRequest(
		ScenarioHandler& handler) {
	handler.abortConnectionAttempt();
	handler.disconnectObjects();
}

void ObjectControl::Connecting::connectRequest(
		ScenarioHandler& handler) {
	// TODO
}

void ObjectControl::Connecting::abortRequest(
		ScenarioHandler& handler) {
	// TODO
}

void ObjectControl::Connecting::connectedToObject(
		ScenarioHandler& handler,
		uint32_t id) {
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		this->allObjectsConnected(handler);
	}
}

void ObjectControl::Connecting::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	// TODO
}

void ObjectControl::Connecting::connectedToLiveObject(
		ScenarioHandler& handler,
		uint32_t id) {
	// TODO
}

void ObjectControl::Connecting::connectedToArmedObject(
		ScenarioHandler& handler,
		uint32_t id) {
	// TODO
}

void ObjectControl::Connecting::allObjectsConnected(
		ScenarioHandler& handler) {
	// TODO
}

void RelativeKinematics::Connecting::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle());
}

void RelativeKinematics::Connecting::connectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::connectRequest(handler);
}

void RelativeKinematics::Connecting::abortRequest(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::abortRequest(handler);
	setState(handler, new RelativeKinematics::Aborting());
}


void RelativeKinematics::Connecting::connectedToObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Connecting::connectedToObject(handler, id);
}

void RelativeKinematics::Connecting::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Connecting::disconnectedFromObject(handler, id);
}

void RelativeKinematics::Connecting::connectedToLiveObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Connecting::connectedToLiveObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting());
}

void RelativeKinematics::Connecting::connectedToArmedObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Connecting::connectedToArmedObject(handler, id);
	setState(handler, new RelativeKinematics::Disarming());
}

void RelativeKinematics::Connecting::allObjectsConnected(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::allObjectsConnected(handler);
	setState(handler, new RelativeKinematics::Ready());
}
