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
    std::vector<TriggerParameter_t> parameters;

    std::set<Trigger::TriggerParameter_t> getAcceptedParameters() const override
    {
        return {
            Trigger::TRIGGER_PARAMETER_FALSE,
            Trigger::TRIGGER_PARAMETER_TRUE,
            Trigger::TRIGGER_PARAMETER_RELEASED,
            Trigger::TRIGGER_PARAMETER_PRESSED,
            Trigger::TRIGGER_PARAMETER_LOW,
            Trigger::TRIGGER_PARAMETER_HIGH,
            Trigger::TRIGGER_PARAMETER_RISING_EDGE,
            Trigger::TRIGGER_PARAMETER_FALLING_EDGE,
            Trigger::TRIGGER_PARAMETER_ANY_EDGE
        };
    }
};
#endif // BRAKETRIGGER_H
