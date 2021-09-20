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
	try {
		auto anchorID = handler.getAnchorObjectID();
		handler.transformScenarioRelativeTo(anchorID);
		handler.controlMode = ScenarioHandler::RELATIVE_KINEMATICS;
		setState(handler, new RelativeKinematics::Initialized);
		LogMessage(LOG_LEVEL_INFO, "Relative control mode enabled");
	} catch (std::invalid_argument) {
		handler.controlMode = ScenarioHandler::ABSOLUTE_KINEMATICS;
		setState(handler, new AbsoluteKinematics::Initialized);
		LogMessage(LOG_LEVEL_INFO, "Absolute control mode enabled");
	}
}
