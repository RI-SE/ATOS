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
	// TODO send OSEM
	handler.uploadObjectConfiguration(id);
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
		ScenarioHandler& handler) {
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
		ScenarioHandler& handler) {
	ObjectControl::Connecting::connectedToObject(handler);
}

void RelativeKinematics::Connecting::disconnectedFromObject(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::disconnectedFromObject(handler);
}

void RelativeKinematics::Connecting::connectedToLiveObject(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::connectedToLiveObject(handler);
	setState(handler, new RelativeKinematics::Aborting());
}

void RelativeKinematics::Connecting::connectedToArmedObject(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::connectedToArmedObject(handler);
	setState(handler, new RelativeKinematics::Disarming());
}

void RelativeKinematics::Connecting::allObjectsConnected(
		ScenarioHandler& handler) {
	ObjectControl::Connecting::connectedToArmedObject(handler);
	setState(handler, new RelativeKinematics::Ready());
}
