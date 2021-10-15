#include "state.hpp"

ObjectControl::Ready::Ready() {

}

void ObjectControl::Ready::onEnter(
		ScenarioHandler& handler) {
	handler.startSafetyThread();
}

void ObjectControl::Ready::armRequest(
		ScenarioHandler& handler) {
	if (!handler.areAllObjectsIn(std::set({OBJECT_STATE_DISARMED, OBJECT_STATE_ARMED}))) {
		throw std::invalid_argument("Not all objects in valid state prior to arm");
	}
}

void ObjectControl::Ready::disconnectRequest(
		ScenarioHandler& handler) {
	handler.disconnectObjects();
}

void ObjectControl::Ready::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Ready::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	// TODO
}

void ObjectControl::Ready::objectAbortDisarmed(
	ScenarioHandler &handler,
	uint32_t id) {
		// TODO
}

void ObjectControl::Ready::settingModificationRequested(
		ScenarioHandler &handler) {
	// TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Ready::armRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::armRequest(handler);
	setState(handler, new RelativeKinematics::Armed);
}

void RelativeKinematics::Ready::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}

void RelativeKinematics::Ready::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Ready::disconnectedFromObject(handler, id);
	setState(handler, new RelativeKinematics::Connecting);
}

void RelativeKinematics::Ready::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Ready::objectAborting(handler,id);
	setState(handler, new RelativeKinematics::Aborting);
}


void RelativeKinematics::Ready::objectAbortDisarmed(
	ScenarioHandler &handler,
	uint32_t id) {
	ObjectControl::Ready::objectAbortDisarmed(handler,id);
}

void RelativeKinematics::Ready::settingModificationRequested(
		ScenarioHandler &handler) {
	ObjectControl::Ready::settingModificationRequested(handler);
	// TODO call scenariohandler to update OSEM if modified etc.
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Ready::armRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::armRequest(handler);
	setState(handler, new AbsoluteKinematics::Armed);
}

void AbsoluteKinematics::Ready::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Ready::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Ready::disconnectedFromObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Ready::disconnectedFromObject(handler, id);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Ready::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Ready::objectAborting(handler,id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Ready::objectAbortDisarmed(
	ScenarioHandler &handler,
	uint32_t id) {
	ObjectControl::Ready::objectAbortDisarmed(handler,id);
}

void AbsoluteKinematics::Ready::settingModificationRequested(
		ScenarioHandler &handler) {
	ObjectControl::Ready::settingModificationRequested(handler);
	// TODO call scenariohandler to update OSEM if modified etc.
}
