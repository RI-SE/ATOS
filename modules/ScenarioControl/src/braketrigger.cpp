#include "braketrigger.h"

static constexpr Trigger::TriggerParameter_t acceptedParameters[] = {
    Trigger::TRIGGER_PARAMETER_FALSE,
    Trigger::TRIGGER_PARAMETER_TRUE,
    Trigger::TRIGGER_PARAMETER_RELEASED,
    Trigger::TRIGGER_PARAMETER_PRESSED,
    Trigger::TRIGGER_PARAMETER_LOW,
    Trigger::TRIGGER_PARAMETER_HIGH,
    Trigger::TRIGGER_PARAMETER_RISING_EDGE,
    Trigger::TRIGGER_PARAMETER_FALLING_EDGE,
    Trigger::TRIGGER_PARAMETER_ANY_EDGE,
    Trigger::TRIGGER_PARAMETER_EQUAL_TO,
    Trigger::TRIGGER_PARAMETER_NOT_EQUAL_TO
};

Trigger::TriggerReturnCode_t BrakeTrigger::appendTriggerParameter(Trigger::TriggerParameter_t triggerParameter)
{
    TriggerReturnCode_t retval = NOT_OK;

    if( (retval = checkTriggerParameter(triggerParameter)) != OK)
        return retval;

}

Trigger::TriggerReturnCode_t BrakeTrigger::checkTriggerParameter(TriggerParameter_t triggerParameter)
{
    for (const TriggerParameter_t param : acceptedParameters) {
        if (triggerParameter == param)
            return OK;
    }
    return NOT_OK;
}
