#include "braketrigger.h"


BrakeTrigger::BrakeTrigger(Trigger::TriggerID_t triggerID) : Trigger (triggerID)
{

}

BrakeTrigger::~BrakeTrigger()
{
    parameters.clear();
}

Trigger::TriggerReturnCode_t BrakeTrigger::appendParameter(Trigger::TriggerParameter_t triggerParameter)
{
    TriggerReturnCode_t retval = NOT_OK;

    if( (retval = checkTriggerParameter(triggerParameter)) != OK)
        return retval;

    parameters.push_back(triggerParameter);
    return OK;
}

Trigger::TriggerReturnCode_t BrakeTrigger::parseParameters()
{

}
