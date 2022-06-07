#include "state.hpp"

AbstractKinematics::TestLive::TestLive() {

}

void AbstractKinematics::TestLive::onEnter(
		ObjectControl& handler) {
	handler.startObjects();
}

void AbstractKinematics::TestLive::stopRequest(ObjectControl&) {
	// TODO
}

void AbstractKinematics::TestLive::abortRequest(ObjectControl&) {
	// TODO
}

void AbstractKinematics::TestLive::testCompleted(ObjectControl&) {
	// TODO
}

void AbstractKinematics::TestLive::disconnectedFromObject(
		ObjectControl&,
		uint32_t) {
	// TODO
}

void AbstractKinematics::TestLive::objectDisarmed(
		ObjectControl&,
		uint32_t) {

}

void AbstractKinematics::TestLive::objectAborting(
		ObjectControl&,
		uint32_t) {

}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::TestLive::stopRequest(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::stopRequest(handler);
	// TODO more?
}

void RelativeKinematics::TestLive::abortRequest(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::abortRequest(handler);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::TestLive::testCompleted(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::testCompleted(handler);
	setState(handler, new RelativeKinematics::Done);
}

void RelativeKinematics::TestLive::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::TestLive::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		RelativeKinematics::TestLive::testCompleted(handler);
	}
}

void RelativeKinematics::TestLive::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::objectAborting(handler,id);
	setState(handler, new RelativeKinematics::Aborting);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::TestLive::stopRequest(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::stopRequest(handler);
	// TODO more?
}

void AbsoluteKinematics::TestLive::abortRequest(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::abortRequest(handler);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::TestLive::testCompleted(
		ObjectControl& handler) {
	AbstractKinematics::TestLive::testCompleted(handler);
	setState(handler, new AbsoluteKinematics::Done);
}

void AbsoluteKinematics::TestLive::disconnectedFromObject(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::TestLive::objectDisarmed(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		AbsoluteKinematics::TestLive::testCompleted(handler);
	}
}

void AbsoluteKinematics::TestLive::objectAborting(
		ObjectControl& handler,
		uint32_t id) {
	AbstractKinematics::TestLive::objectAborting(handler,id);
	setState(handler, new AbsoluteKinematics::Aborting);
}
