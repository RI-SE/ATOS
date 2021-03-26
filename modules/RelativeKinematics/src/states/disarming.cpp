#include "state.hpp"

ObjectControl::Disarming::Disarming() {

}

void ObjectControl::Disarming::disconnectRequest(
		ScenarioHandler& handler) {
	handler.disconnectObjects();
}

void ObjectControl::Disarming::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::connectedToLiveObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::allObjectsDisarmed(ScenarioHandler&) {
	// TODO
}


void RelativeKinematics::Disarming::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle());
}

void RelativeKinematics::Disarming::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::connectedToLiveObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Disarming::allObjectsDisarmed(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::allObjectsDisarmed(handler);
	setState(handler, new RelativeKinematics::Connecting());
}
