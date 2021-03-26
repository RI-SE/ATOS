#include "state.hpp"

ObjectControl::TestLive::TestLive() {

}

void ObjectControl::TestLive::stopRequest(ScenarioHandler&) {
	// TODO
}

void ObjectControl::TestLive::abortRequest(ScenarioHandler&) {
	// TODO
}

void ObjectControl::TestLive::testCompleted(ScenarioHandler&) {
	// TODO
}

void ObjectControl::TestLive::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}


void RelativeKinematics::TestLive::stopRequest(ScenarioHandler&) {
	// TODO
}

void RelativeKinematics::TestLive::abortRequest(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::abortRequest(handler);
	setState(handler, new RelativeKinematics::Aborting());
}

void RelativeKinematics::TestLive::testCompleted(ScenarioHandler&) {
	// TODO
}

void RelativeKinematics::TestLive::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::TestLive::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting());
}
