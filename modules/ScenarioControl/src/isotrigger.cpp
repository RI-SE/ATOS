#include "isotrigger.h"

namespace maestro {
	Trigger::TriggerReturnCode_t ISOTrigger::update(maestro_interfaces::msg::TriggerEventOccurred::SharedPtr data)
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