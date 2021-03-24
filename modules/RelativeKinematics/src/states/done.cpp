#include "state.hpp"

ObjectControl::Done::Done() {

}

void ObjectControl::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_WARNING, "Nothing to be done for postprocessing"); // TODO
}

void RelativeKinematics::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	ObjectControl::Done::postProcessingCompleted(handler);
	setState(handler, new RelativeKinematics::Ready());
}
