#ifndef BRAKETRIGGER_H
#define BRAKETRIGGER_H

#include "trigger.h"

class BrakeTrigger : public Trigger
{
public:
    //BrakeTrigger(TriggerType_t triggerID);
    ~BrakeTrigger();

    TriggerType_t getType() { return TRIGGER_BRAKE; }

    TriggerReturnCode_t appendTriggerParameter(TriggerParameter_t triggerParameter);

private:
    std::set<TriggerParameter_t> triggerParameters;

    TriggerReturnCode_t checkTriggerParameter(TriggerParameter_t triggerParameter);
};

#endif // BRAKETRIGGER_H
