#include "state.hpp"
#include "logging.h"
#include "journal.h"


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
	AbstractKinematics::Initialized::connectRequest(handler);
	setState(handler, new RelativeKinematics::Connecting);
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
	AbstractKinematics::Initialized::connectRequest(handler);
	setState(handler, new AbsoluteKinematics::Connecting);
}

void AbsoluteKinematics::Initialized::disconnectRequest(
		ObjectControl& handler) {
	AbstractKinematics::Initialized::disconnectRequest(handler);
	setState(handler, new AbsoluteKinematics::Idle);
}


