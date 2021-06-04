#include "state.hpp"

ObjectControl::Aborting::Aborting() {

}

void ObjectControl::Aborting::allClearRequest(ScenarioHandler&) {
	// TODO
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

void RelativeKinematics::Aborting::allClearRequest(
		ScenarioHandler &handler) {
	ObjectControl::Aborting::allClearRequest(handler);
	setState(handler, new RelativeKinematics::Ready());
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
