#include "state.hpp"

ObjectControl::Ready::Ready() {

}

void ObjectControl::Ready::armRequest(
		ScenarioHandler&) {
	// TODO
}

void ObjectControl::Ready::disconnectRequest(
		ScenarioHandler& handler) {
	handler.disconnectObjects();
}

void ObjectControl::Ready::disconnectedFromObject(
		ScenarioHandler&) {
	// TODO
}


void RelativeKinematics::Ready::armRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::armRequest(handler);
	setState(handler, new RelativeKinematics::Armed());
}

void RelativeKinematics::Ready::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle());
}

void RelativeKinematics::Ready::disconnectedFromObject(
		ScenarioHandler& handler) {
	ObjectControl::Ready::disconnectedFromObject(handler);
	setState(handler, new RelativeKinematics::Connecting());
}
