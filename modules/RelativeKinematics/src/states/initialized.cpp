#include "state.hpp"
#include "logging.h"
#include "journal.h"


RelativeKinematics::Initialized::Initialized() {

}

void RelativeKinematics::Initialized::connectRequest(
		ScenarioHandler& handler) {
	// TODO
	setState(handler, new RelativeKinematics::Connecting());
}

void RelativeKinematics::Initialized::disconnectRequest(
		ScenarioHandler& handler) {
	handler.clearScenario();
	setState(handler, new RelativeKinematics::Idle());
}

