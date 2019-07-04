#include "booleantrigger.h"

/*!
 * \brief BooleanTrigger::update Updates the signal connected to the trigger to the value specified
 * \param isBrakeCurrentlyPressed Boolean describing if the boolean
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BooleanTrigger::update(bool currentStateValue)
{
    wasStateTrue = isStateTrue;
    isStateTrue = currentStateValue;
    return checkIfTriggered();
}

/*!
 * \brief BooleanTrigger::checkIfTriggered Check if the trigger has occurred based on the mode and state
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BooleanTrigger::checkIfTriggered()
{
    switch (mode) {
    case HIGH:
        return isStateTrue ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case LOW:
        return !isStateTrue ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_ANY:
        return (isStateTrue != wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_RISING:
        return (isStateTrue && !wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_FALLING:
        return (!isStateTrue && wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case INVALID_MODE:
        return NOT_OK;
    }
}


