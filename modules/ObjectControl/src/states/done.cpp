/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"

AbstractKinematics::Done::Done() {

}

void AbstractKinematics::Done::onEnter(
		ObjectControl& handler) {
	RCLCPP_WARN(handler.get_logger(), "Nothing to be done for postprocessing"); // TODO
	handler.sendAbortNotification(); // TODO temporary to trigger logging etc.
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
