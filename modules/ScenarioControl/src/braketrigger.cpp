
#include "braketrigger.h"

BrakeTrigger::BrakeTrigger(Trigger::TriggerID_t triggerID) : BooleanTrigger(triggerID, Trigger::TriggerTypeCode_t::TRIGGER_BRAKE) { }

/*!
 * \brief BooleanTrigger::parseParameters Parses the parameter vector and sets the trigger mode accordingly
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BrakeTrigger::parseParameters()
{
    if (parameters.size() == 1)
    {
        switch (parameters.front())
        {
        case TRIGGER_PARAMETER_PRESSED:
        case TRIGGER_PARAMETER_TRUE:
        case TRIGGER_PARAMETER_HIGH:
            mode = HIGH;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_RELEASED:
        case TRIGGER_PARAMETER_FALSE:
        case TRIGGER_PARAMETER_LOW:
            mode = LOW;
            isStateTrue = true;
            wasStateTrue = true;
            return OK;
        case TRIGGER_PARAMETER_RISING_EDGE:
            mode = EDGE_RISING;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_FALLING_EDGE:
            mode = EDGE_FALLING;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        case TRIGGER_PARAMETER_ANY_EDGE:
            mode = EDGE_ANY;
            isStateTrue = false;
            wasStateTrue = false;
            return OK;
        default:
            return INVALID_ARGUMENT;
        }
    }
    else return INVALID_ARGUMENT;
}


