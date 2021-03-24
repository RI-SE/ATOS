#include "state.hpp"
#include "logging.h"
#include "journal.h"


ObjectControl::Initialized::Initialized() {

}

void ObjectControl::Initialized::connectRequest(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_INFO, "Handling connect request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "CONNECT received");
}

void ObjectControl::Initialized::disconnectRequest(
		ScenarioHandler& handler) {
	handler.clearScenario();
}


void RelativeKinematics::Initialized::connectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Initialized::connectRequest(handler);
	setState(handler, new RelativeKinematics::Connecting());
}

void RelativeKinematics::Initialized::disconnectRequest(
		ScenarioHandler& handler) {
	ObjectControl::Initialized::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle());
}

