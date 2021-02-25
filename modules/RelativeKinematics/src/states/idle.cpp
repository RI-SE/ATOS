#include "state.hpp"
#include "logging.h"
#include "journal.h"

RelativeKinematics::Idle::Idle() {

}

void RelativeKinematics::Idle::initializeRequest(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_INFO, "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");

	try {
		handler.loadScenario();
		// TODO Transform
		setState(handler, new RelativeKinematics::Initialized());
	}
	catch (std::invalid_argument& e) {
		// TODO
	}
}
