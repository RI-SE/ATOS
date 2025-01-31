/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"

namespace state_machine {

void test_live_start_object::operator()(const events::StartObject& event, ObjectControl* oc) const {
	oc->startObject(event.id, event.startTime);
}

}
