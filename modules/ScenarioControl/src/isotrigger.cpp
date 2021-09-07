#include "isotrigger.h"

Trigger::TriggerReturnCode_t ISOTrigger::update(TREOData data)
{
	if (data.triggerID == getID() && data.ip == triggerObjectIP)
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
