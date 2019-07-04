#ifndef BRAKETRIGGER_H
#define BRAKETRIGGER_H

#include "booleantrigger.h"

#include <set>
#include <vector>

class BrakeTrigger : public BooleanTrigger
{
public:
    using BooleanTrigger::BooleanTrigger;

    TriggerReturnCode_t parseParameters() override;

private:
    TriggerTypeCode_t triggerTypeCode = TRIGGER_BRAKE;

    std::vector<TriggerParameter_t> parameters;

    std::set<Trigger::TriggerParameter_t> getAcceptedParameters() override
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
