/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "objectcontrol.hpp"


namespace state_machine {

void ready_on_entry::operator()(ObjectControl* oc) const {
	oc->startSafetyThread();
}

void ready_reset::operator()(ObjectControl* oc) const {
	oc->resetTestObjects();
}

void ready_reload::operator()(ObjectControl* oc) const {
	oc->reloadScenarioTrajectories();
	oc->uploadAllConfigurations();
}

}
