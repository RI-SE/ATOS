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
