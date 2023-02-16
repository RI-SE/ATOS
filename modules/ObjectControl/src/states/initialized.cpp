/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "state.hpp"
#include "journal.hpp"


AbstractKinematics::Initialized::Initialized() {

}

void AbstractKinematics::Initialized::connectRequest(
		ObjectControl& handler) {
	RCLCPP_INFO(handler.get_logger(), "Handling connect request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "CONNECT received");
}

void AbstractKinematics::Initialized::disconnectRequest(
		ObjectControl& handler) {
	handler.clearScenario();
}


/*! ******************************************************
 * \section RelativeKinematics
 *  ******************************************************
 */
void RelativeKinematics::Initialized::connectRequest(
		ObjectControl& handler) {

	if (handler.getVehicleIDs().empty()) {
		RCLCPP_INFO(handler.get_logger(), "Object directory empty. Disconnecting...");
		RelativeKinematics::Initialized::disconnectRequest(handler);
	}
	else {
		AbstractKinematics::Initialized::connectRequest(handler);
		setState(handler, new RelativeKinematics::Connecting);
	}
}

void RelativeKinematics::Initialized::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Initialized::disconnectRequest(handler);
	setState(handler, new RelativeKinematics::Idle);
}


/*! ******************************************************
 * \section AbsoluteKinematics
 *  ******************************************************
 */
void AbsoluteKinematics::Initialized::connectRequest(
		ObjectControl& handler) {

	if (handler.getVehicleIDs().empty()) {
		RCLCPP_INFO(handler.get_logger(), "Object directory empty. Disconnecting...");
		AbsoluteKinematics::Initialized::disconnectRequest(handler);
	}
	else {
		AbstractKinematics::Initialized::connectRequest(handler);
		setState(handler, new AbsoluteKinematics::Connecting);
	}
}

void AbsoluteKinematics::Initialized::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Initialized::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}


