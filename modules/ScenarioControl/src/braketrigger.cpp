#include "braketrigger.h"

BrakeTrigger::BrakeTrigger(Trigger::TriggerID_t triggerID) : Trigger (triggerID)
{

}

BrakeTrigger::~BrakeTrigger()
{
    parameters.clear();
}

/*!
 * \brief BrakeTrigger::update Updates the brake signal connected to the trigger to the value specified
 * \param isBrakeCurrentlyPressed Boolean describing if the brake is currently pressed
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BrakeTrigger::update(bool isBrakeCurrentlyPressed)
{
    wasBrakePressed = isBrakePressed;
    isBrakePressed = isBrakeCurrentlyPressed;
    return checkIfTriggered();
}

/*!
 * \brief BrakeTrigger::checkIfTriggered Check if the trigger has occurred based on the mode and state
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BrakeTrigger::checkIfTriggered()
{
    switch (mode) {
    case PRESSED:
        return isBrakePressed ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case RELEASED:
        return !isBrakePressed ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_ANY:
        return (isBrakePressed != wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_RISING:
        return (isBrakePressed && !wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_FALLING:
        return (!isBrakePressed && wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case INVALID_MODE:
        return NOT_OK;
    }
}

/*!
 * \brief BrakeTrigger::parseParameters Parses the parameter vector and sets the trigger mode accordingly
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
            mode = PRESSED;
            isBrakePressed = false;
            wasBrakePressed = false;
            return OK;
        case TRIGGER_PARAMETER_RELEASED:
        case TRIGGER_PARAMETER_FALSE:
        case TRIGGER_PARAMETER_LOW:
            mode = RELEASED;
            isBrakePressed = true;
            wasBrakePressed = true;
            return OK;
        case TRIGGER_PARAMETER_RISING_EDGE:
            mode = EDGE_RISING;
            isBrakePressed = false;
            wasBrakePressed = false;
            return OK;
        case TRIGGER_PARAMETER_FALLING_EDGE:
            mode = EDGE_FALLING;
            isBrakePressed = false;
            wasBrakePressed = false;
            return OK;
        case TRIGGER_PARAMETER_ANY_EDGE:
            mode = EDGE_ANY;
            isBrakePressed = false;
            wasBrakePressed = false;
            return OK;
        default:
            return INVALID_ARGUMENT;
        }
    }
    else return INVALID_ARGUMENT;
}




