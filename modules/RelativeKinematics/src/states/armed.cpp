#include "state.hpp"

ObjectControl::Armed::Armed() {

}

void ObjectControl::Armed::onEnter(
		ScenarioHandler& handler) {
	handler.armObjects();
}

void ObjectControl::Armed::startRequest(
		ScenarioHandler& handler) {
	if (!handler.areAllObjectsIn(OBJECT_STATE_ARMED)) {
		throw std::invalid_argument("Start attempted when not all objects were armed");
	}
}

void ObjectControl::Armed::disarmRequest(
		ScenarioHandler&) {
}

void ObjectControl::Armed::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
}

void ObjectControl::Armed::objectArmed(
		ScenarioHandler &,
		uint32_t) {
}

void ObjectControl::Armed::objectDisarmed(
		ScenarioHandler &,
		uint32_t) {
}

void ObjectControl::Armed::objectAborting(
		ScenarioHandler &,
		uint32_t) {
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

void RelativeKinematics::Armed::objectArmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Armed::objectArmed(handler, id);
	// TODO possible to do something here
}

void RelativeKinematics::Armed::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Armed::objectDisarmed(handler, id);
	setState(handler, new RelativeKinematics::Ready());
}

void RelativeKinematics::Armed::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Armed::objectAborting(handler, id);
	setState(handler, new RelativeKinematics::Aborting());
}
