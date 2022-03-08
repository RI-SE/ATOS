#include "state.hpp"

AbstractKinematics::Done::Done() {

}

void AbstractKinematics::Done::onEnter(
		ObjectControl& handler) {
	RCLCPP_WARN(handler.get_logger(), "Nothing to be done for postprocessing"); // TODO
	iCommSend(COMM_ABORT, nullptr, 0); // TODO temporary to trigger logging etc.
	handler.state->postProcessingCompleted(handler);
}

void AbstractKinematics::Done::postProcessingCompleted(
		ObjectControl& handler) {
	// TODO
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Done::postProcessingCompleted(
		ObjectControl& handler) {
	AbstractKinematics::Done::postProcessingCompleted(handler);
	setState(handler, new RelativeKinematics::Ready);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Done::postProcessingCompleted(
		ObjectControl& handler) {
	AbstractKinematics::Done::postProcessingCompleted(handler);
	setState(handler, new AbsoluteKinematics::Ready);
}
