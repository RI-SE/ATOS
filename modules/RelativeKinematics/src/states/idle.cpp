#include "state.hpp"
#include "logging.h"
#include "journal.h"

ObjectControl::Idle::Idle() {

}

void ObjectControl::Idle::onEnter(
		ScenarioHandler& handler) {
	handler.clearScenario();
}

void ObjectControl::Idle::initializeRequest(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_INFO, "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
	handler.loadScenario();
}

void RelativeKinematics::Idle::initializeRequest(
		ScenarioHandler& handler) {
	ObjectControl::Idle::initializeRequest(handler);
	auto anchorID = handler.getAnchorObjectID();
	handler.transformScenarioRelativeTo(anchorID);
	setState(handler, new RelativeKinematics::Initialized());
}
