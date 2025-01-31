/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "journal.hpp"
#include "objectcontrol.hpp"

namespace state_machine {

void init_to_connecting_action::operator()(ObjectControl* oc) const {
    RCLCPP_INFO(oc->get_logger(), "Handling connect request");
    JournalRecordData(JOURNAL_RECORD_EVENT, "CONNECT received");
}

bool init_to_connecting_guard::operator()(ObjectControl* oc) const {
    if (oc->getVehicleIDs().empty())
    {
      RCLCPP_WARN(oc->get_logger(),
                  "No objects are configured! Canceling connect request...");
      return false;
    }
    else
    {
        return true;
    }
}

void init_to_idle_action::operator()(ObjectControl* oc) const {
    oc->clearScenario();
}

}
