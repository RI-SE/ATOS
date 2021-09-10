#include "state.hpp"

ObjectControl::TestLive::TestLive() {

}

void ObjectControl::TestLive::onEnter(
		ScenarioHandler &handler) {
	handler.startObjects();
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

void ObjectControl::TestLive::objectDisarmed(
		ScenarioHandler &,
		uint32_t) {

}

void ObjectControl::TestLive::objectAborting(
		ScenarioHandler &,
		uint32_t) {

}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::TestLive::stopRequest(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::stopRequest(handler);
	// TODO more?
}

void RelativeKinematics::TestLive::abortRequest(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::abortRequest(handler);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::TestLive::testCompleted(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::testCompleted(handler);
	setState(handler, new RelativeKinematics::Done);
}

void RelativeKinematics::TestLive::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::TestLive::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::TestLive::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::TestLive::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		RelativeKinematics::TestLive::testCompleted(handler);
	}
}

void RelativeKinematics::TestLive::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::TestLive::objectAborting(handler,id);
	setState(handler, new RelativeKinematics::Aborting);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::TestLive::stopRequest(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::stopRequest(handler);
	// TODO more?
}

void AbsoluteKinematics::TestLive::abortRequest(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::abortRequest(handler);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::TestLive::testCompleted(
		ScenarioHandler& handler) {
	ObjectControl::TestLive::testCompleted(handler);
	setState(handler, new AbsoluteKinematics::Done);
}

void AbsoluteKinematics::TestLive::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::TestLive::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::TestLive::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::TestLive::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		AbsoluteKinematics::TestLive::testCompleted(handler);
	}
}

void AbsoluteKinematics::TestLive::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::TestLive::objectAborting(handler,id);
	setState(handler, new AbsoluteKinematics::Aborting);
}
