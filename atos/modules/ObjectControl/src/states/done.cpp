/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"

namespace state_machine {

void done_on_entry::operator()(ObjectControl* oc) const {
	RCLCPP_WARN(oc->get_logger(), "Nothing to be done for postprocessing"); // TODO
	oc->sendAbortNotification(); // TODO temporary to trigger logging etc.
}

}
