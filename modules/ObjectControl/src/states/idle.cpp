#include "state.hpp"
#include "logging.h"
#include "journal.h"

AbstractKinematics::Idle::Idle() {

}

void AbstractKinematics::Idle::onEnter(
		ObjectControl& handler) {
	handler.clearScenario();
}

void AbstractKinematics::Idle::initializeRequest(
		ObjectControl& handler) {
	LogMessage(LOG_LEVEL_INFO, "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
	handler.loadScenario();
	try {
		auto anchorID = handler.getAnchorObjectID();
		handler.transformScenarioRelativeTo(anchorID);
		handler.controlMode = ObjectControl::RELATIVE_KINEMATICS;
		setState(handler, new RelativeKinematics::Initialized);
		LogMessage(LOG_LEVEL_INFO, "Relative control mode enabled");
	} catch (std::invalid_argument) {
		handler.controlMode = ObjectControl::ABSOLUTE_KINEMATICS;
		setState(handler, new AbsoluteKinematics::Initialized);
		LogMessage(LOG_LEVEL_INFO, "Absolute control mode enabled");
	}
}
