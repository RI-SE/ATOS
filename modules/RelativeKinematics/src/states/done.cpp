#include "state.hpp"

ObjectControl::Done::Done() {

}

void ObjectControl::Done::onEnter(
		ScenarioHandler& handler) {
	LogMessage(LOG_LEVEL_WARNING, "Nothing to be done for postprocessing"); // TODO
	iCommSend(COMM_ABORT, nullptr, 0); // TODO temporary to trigger logging etc.
	handler.state->postProcessingCompleted(handler);
}

void ObjectControl::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	// TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	ObjectControl::Done::postProcessingCompleted(handler);
	setState(handler, new RelativeKinematics::Ready);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Done::postProcessingCompleted(
		ScenarioHandler& handler) {
	ObjectControl::Done::postProcessingCompleted(handler);
	setState(handler, new AbsoluteKinematics::Ready);
}
