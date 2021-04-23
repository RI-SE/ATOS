#include "state.hpp"

ObjectControl::Done::Done() {

}

void ObjectControl::Done::onEnter(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_WARNING, "Nothing to be done for postprocessing"); // TODO
	handler.state->postProcessingCompleted(handler);
}

void ObjectControl::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	// TODO
}

void RelativeKinematics::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	ObjectControl::Done::postProcessingCompleted(handler);
	setState(handler, new RelativeKinematics::Ready());
}
