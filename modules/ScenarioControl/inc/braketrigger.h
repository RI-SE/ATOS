#ifndef BRAKETRIGGER_H
#define BRAKETRIGGER_H

#include "booleantrigger.h"

#include <set>
#include <vector>

class BrakeTrigger : public BooleanTrigger
{
public:
    BrakeTrigger(TriggerID_t triggerID);
    TriggerReturnCode_t parseParameters() override;

private:

    const std::set<TriggerParameter_t> getAcceptedParameters() const override
    {
        std::set<TriggerParameter_t> accParams;
        accParams.insert(TRIGGER_PARAMETER_FALSE);
        accParams.insert(TRIGGER_PARAMETER_TRUE);
        accParams.insert(TRIGGER_PARAMETER_RELEASED);
        accParams.insert(TRIGGER_PARAMETER_PRESSED);
        accParams.insert(TRIGGER_PARAMETER_LOW);
        accParams.insert(TRIGGER_PARAMETER_HIGH);
        accParams.insert(TRIGGER_PARAMETER_RISING_EDGE);
        accParams.insert(TRIGGER_PARAMETER_FALLING_EDGE);
        accParams.insert(TRIGGER_PARAMETER_ANY_EDGE);
        return accParams;
    }
};
#endif // BRAKETRIGGER_H
