#include "state.hpp"
#include "logging.h"
#include "journal.h"

AbstractKinematics::Idle::Idle() {

}

void AbstractKinematics::Idle::onEnter(
		ObjectControl& handler) {
	handler.clearScenario();
}

void AbstractKinematics::Idle::initializeRequest(
		ObjectControl& handler) {
	RCLCPP_INFO(handler.get_logger(), "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
	handler.loadScenario();
	try {
		auto anchorID = handler.getAnchorObjectID();
		handler.transformScenarioRelativeTo(anchorID);
		handler.controlMode = ObjectControl::RELATIVE_KINEMATICS;
		setState(handler, new RelativeKinematics::Initialized);
		RCLCPP_INFO(handler.get_logger(), "Relative control mode enabled");
	} catch (std::invalid_argument) {
		handler.controlMode = ObjectControl::ABSOLUTE_KINEMATICS;
		setState(handler, new AbsoluteKinematics::Initialized);
		RCLCPP_INFO(handler.get_logger(), "Absolute control mode enabled");
	}
}
