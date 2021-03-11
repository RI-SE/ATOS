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
		auto vutIDs = handler.getVehicleUnderTestIDs();
		if (vutIDs.size() == 1) {
			handler.transformScenarioRelativeTo(vutIDs[0]);
		}
		else {
			throw std::invalid_argument("Unable to transform scenario - several configured VUTs");
		}
		setState(handler, new RelativeKinematics::Initialized());
	}
	catch (std::invalid_argument& e) {
		// TODO
		LogMessage(LOG_LEVEL_ERROR, e.what());
	}
}
