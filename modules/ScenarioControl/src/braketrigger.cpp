#include "braketrigger.h"


BrakeTrigger::BrakeTrigger(Trigger::TriggerID_t triggerID) : Trigger (triggerID)
{

}

BrakeTrigger::~BrakeTrigger()
{
    parameters.clear();
}

Trigger::TriggerReturnCode_t BrakeTrigger::parseParameters()
{
    for (const TriggerParameter_t param : parameters)
    {

    }
}

Trigger::TriggerReturnCode_t BrakeTrigger::update(bool isBrakeCurrentlyPressed)
{
    wasBrakePressed = isBrakePressed;
    isBrakePressed = isBrakeCurrentlyPressed;
    return checkIfTriggered();
}

Trigger::TriggerReturnCode_t BrakeTrigger::checkIfTriggered()
{
    switch (mode) {
    case PRESSED:
        return isBrakePressed ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case RELEASED:
        return isBrakePressed ? NO_TRIGGER_OCCURRED : TRIGGER_OCCURRED;
    case EDGE_ANY:
        return (isBrakePressed != wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_RISING:
        return (isBrakePressed && !wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    case EDGE_FALLING:
        return (!isBrakePressed && wasBrakePressed) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
    }
}
