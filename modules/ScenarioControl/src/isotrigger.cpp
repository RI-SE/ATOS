#include "isotrigger.h"

Trigger::TriggerReturnCode_t ISOTrigger::update(TREOData data)
{
    if (data.triggerID == getID())
    {
        // TODO: Check IP as well?
        wasTriggeredByLastUpdate = TRIGGER_OCCURRED;
    }
    else
        return INVALID_ARGUMENT;
}
