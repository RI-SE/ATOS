#include "state.hpp"

ObjectControl::Aborting::Aborting() {

}

void ObjectControl::Aborting::onExit(
		ScenarioHandler&) {
}

void ObjectControl::Aborting::allClearRequest(
		ScenarioHandler& handler) {
		handler.allClearObjects();
}

void ObjectControl::Aborting::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Aborting::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Aborting::connectedToLiveObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Aborting::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void ObjectControl::Aborting::objectAborting(
		ScenarioHandler &,
		uint32_t) {
	// TODO
}

void ObjectControl::Aborting::objectAbortDisarmed(
	ScenarioHandler &,
	uint32_t) {
	// TODO
}

void ObjectControl::Aborting::objectDisarmed(
		ScenarioHandler &handler,
		uint32_t id) {
	LogMessage(LOG_LEVEL_WARNING, "Object disarmed while expecting abort");
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}

void ObjectControl::Aborting::objectArmed(
		ScenarioHandler &handler,
		uint32_t id) {
	LogMessage(LOG_LEVEL_WARNING, "Object armed while expecting abort");
	handler.disconnectObject(id); // TODO just stop sending HEAB to keep tracking available
}


void ObjectControl::Aborting::allObjectsAbortDisarmed(
		ScenarioHandler &handler) {
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Aborting::allClearRequest(
		ScenarioHandler &handler) {
	ObjectControl::Aborting::allClearRequest(handler);
	setState(handler, new RelativeKinematics::Ready);
}

void RelativeKinematics::Aborting::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::connectedToLiveObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void RelativeKinematics::Aborting::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Aborting::objectAborting(handler, id);
	// TODO
}

void RelativeKinematics::Aborting::objectAbortDisarmed(
	ScenarioHandler &handler,
	uint32_t id) {
	ObjectControl::Aborting::objectAbortDisarmed(handler,id);
}


void RelativeKinematics::Aborting::allObjectsAbortDisarmed(
		ScenarioHandler &handler) {
	ObjectControl::Aborting::allObjectsAbortDisarmed(handler);
	setState(handler, new RelativeKinematics::Ready);			
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Aborting::allClearRequest(
		ScenarioHandler &handler) {
	ObjectControl::Aborting::allClearRequest(handler);
	setState(handler, new AbsoluteKinematics::Ready);
}

void AbsoluteKinematics::Aborting::connectedToObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::disconnectedFromObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::connectedToLiveObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::connectedToArmedObject(
		ScenarioHandler&,
		uint32_t) {
	// TODO
}

void AbsoluteKinematics::Aborting::objectAborting(
		ScenarioHandler &handler,
		uint32_t id) {
	ObjectControl::Aborting::objectAborting(handler, id);
	// TODO
}

void AbsoluteKinematics::Aborting::objectAbortDisarmed(
	ScenarioHandler &handler,
	uint32_t id) {
	ObjectControl::Aborting::objectAbortDisarmed(handler,id);
}


void AbsoluteKinematics::Aborting::allObjectsAbortDisarmed(
		ScenarioHandler &handler) {
	setState(handler, new AbsoluteKinematics::Ready);			
}
