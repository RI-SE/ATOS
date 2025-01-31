/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"

namespace state_machine {

void armed_on_enter::operator()(ObjectControl* oc) const {
	oc->armObjects();
}

bool armed_to_testlive_guard::operator()(ObjectControl* oc) const {
	return oc->areAllObjectsIn(OBJECT_STATE_ARMED);
}

}
