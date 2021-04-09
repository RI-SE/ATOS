#include "state.hpp"

ObjectControl::Armed::Armed() {

}

void ObjectControl::Armed::onEnter(
		ScenarioHandler& handler) {
	// TODO send OSTMs
	handler.armObjects();
}

void ObjectControl::Armed::startRequest(ScenarioHandler&) {
	// TODO
}

void ObjectControl::Armed::disarmRequest(ScenarioHandler&) {
	// TODO
}

void ObjectControl::Armed::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}


void RelativeKinematics::Armed::startRequest(
		ScenarioHandler& handler) {
	ObjectControl::Armed::startRequest(handler);
	setState(handler, new RelativeKinematics::TestLive());
}

void RelativeKinematics::Armed::disarmRequest(
		ScenarioHandler& handler) {
	ObjectControl::Armed::disarmRequest(handler);
	setState(handler, new RelativeKinematics::Disarming());
}

void RelativeKinematics::Armed::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Armed::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Disarming());
}
