#include "state.hpp"

ObjectControl::Disarming::Disarming() {

}

void ObjectControl::Disarming::onEnter(
		ScenarioHandler& handler) {
	handler.disarmObjects();
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

void ObjectControl::Disarming::allObjectsDisarmed(
		ScenarioHandler&) {
	// TODO
}

void ObjectControl::Disarming::objectArmed(
		ScenarioHandler &,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::objectDisarmed(
		ScenarioHandler &,
		uint32_t) {
	// TODO
}

void ObjectControl::Disarming::objectAborting(
		ScenarioHandler &,
		uint32_t) {
	// TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Disarming::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
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
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Disarming::connectedToLiveObject(handler, id);
	setState(handler, new RelativeKinematics::Aborting);
}

void RelativeKinematics::Disarming::allObjectsDisarmed(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::allObjectsDisarmed(handler);
	setState(handler, new RelativeKinematics::Connecting);
}

void RelativeKinematics::Disarming::objectArmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectArmed(handler,id);
	RelativeKinematics::Disarming::connectedToArmedObject(handler,id);
}

void RelativeKinematics::Disarming::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		RelativeKinematics::Disarming::allObjectsDisarmed(handler);
	}
}

void RelativeKinematics::Disarming::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectAborting(handler,id);
	RelativeKinematics::Disarming::connectedToLiveObject(handler,id);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Disarming::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}

void AbsoluteKinematics::Disarming::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Disarming::connectedToLiveObject(
		ScenarioHandler& handler,
		uint32_t id) {
	ObjectControl::Disarming::connectedToLiveObject(handler, id);
	setState(handler, new AbsoluteKinematics::Aborting);
}

void AbsoluteKinematics::Disarming::allObjectsDisarmed(
		ScenarioHandler& handler) {
	ObjectControl::Disarming::allObjectsDisarmed(handler);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Disarming::objectArmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectArmed(handler,id);
	AbsoluteKinematics::Disarming::connectedToArmedObject(handler,id);
}

void AbsoluteKinematics::Disarming::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectDisarmed(handler,id);
	if (handler.areAllObjectsIn(OBJECT_STATE_DISARMED)) {
		AbsoluteKinematics::Disarming::allObjectsDisarmed(handler);
	}
}

void AbsoluteKinematics::Disarming::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Disarming::objectAborting(handler,id);
	AbsoluteKinematics::Disarming::connectedToLiveObject(handler,id);
}
