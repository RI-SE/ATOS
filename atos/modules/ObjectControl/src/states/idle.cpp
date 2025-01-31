/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"
#include "journal.hpp"


namespace state_machine {

void idle_on_entry::operator()(ObjectControl* oc) const {
	RCLCPP_INFO(oc->get_logger(), "Handling initialization request");
	JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
	oc->clearScenario();
}

bool idle_to_init_guard::operator()(ObjectControl* oc) const {
	 // Reload objects on each initialize request.
	bool const successful{oc->loadScenario()};
	if (!successful) {
		RCLCPP_ERROR(oc->get_logger(), "Failed to load scenario");
		JournalRecordData(JOURNAL_RECORD_EVENT, "INIT failed");
	}
	return successful;
}

void idle_to_init_action::operator()(ObjectControl* oc) const {
	try {
		auto anchorID = oc->getAnchorObjectID();
		oc->transformScenarioRelativeTo(anchorID);
		oc->setControlMode(ControlMode::RelativeKinematics);
		RCLCPP_INFO(oc->get_logger(), "Relative control mode enabled");
	} catch (std::invalid_argument&) {
		oc->setControlMode(ControlMode::AbsoluteKinematics);
		RCLCPP_INFO(oc->get_logger(), "Absolute control mode enabled");
	}
}

} // namespace state_machine
