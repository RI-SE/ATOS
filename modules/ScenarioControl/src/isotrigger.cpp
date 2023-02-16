/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "isotrigger.h"

namespace ATOS {
	Trigger::TriggerReturnCode_t ISOTrigger::update(atos_interfaces::msg::TriggerEventOccurred::SharedPtr data)
	{
		if (data->trigger_id == getID() && data->ip == triggerObjectIP)
		{
			wasTriggeredByLastUpdate = TRIGGER_OCCURRED;
			return checkIfTriggered();
		}
		else
			return INVALID_ARGUMENT;
	}

	Trigger::TriggerReturnCode_t ISOTrigger::checkIfTriggered() const
	{
		return wasTriggeredByLastUpdate;
	}
}