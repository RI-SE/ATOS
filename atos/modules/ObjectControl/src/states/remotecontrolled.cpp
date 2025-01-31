/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"

namespace state_machine {

void remote_control_on_entry::operator()(ObjectControl* oc) const {
	oc->remoteControlObjects(true);
	oc->startControlSignalSubscriber();
}

void remote_control_on_exit::operator()(ObjectControl* oc) const {
	oc->stopControlSignalSubscriber();
	oc->remoteControlObjects(false);
}

}
