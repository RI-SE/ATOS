#ifndef ISOTRIGGER_H
#define ISOTRIGGER_H

#include "util.h"
#include "trigger.h"

class ISOTrigger : public Trigger
{
public:
    using Trigger::Trigger;

    TriggerReturnCode_t parseParameters() override { return OK; }

    using Trigger::update;
    TriggerReturnCode_t update(void) override { return wasTriggeredByLastUpdate = NO_TRIGGER_OCCURRED; }
    TriggerReturnCode_t update(TREOData) override;
private:
    TriggerReturnCode_t checkIfTriggered() const override;
    const std::set<TriggerParameter_t> getAcceptedParameters() const override
    {
        std::set<Trigger::TriggerParameter_t> retval;

        retval.insert(TRIGGER_PARAMETER_FALSE);
        retval.insert(TRIGGER_PARAMETER_TRUE);
        retval.insert(TRIGGER_PARAMETER_RELEASED);
        retval.insert(TRIGGER_PARAMETER_PRESSED);
        retval.insert(TRIGGER_PARAMETER_LOW);
        retval.insert(TRIGGER_PARAMETER_HIGH);
        retval.insert(TRIGGER_PARAMETER_RISING_EDGE);
        retval.insert(TRIGGER_PARAMETER_FALLING_EDGE);
        retval.insert(TRIGGER_PARAMETER_ANY_EDGE);
        retval.insert(TRIGGER_PARAMETER_RELATIVE);
        retval.insert(TRIGGER_PARAMETER_ABSOLUTE);
        retval.insert(TRIGGER_PARAMETER_VALUE);
        retval.insert(TRIGGER_PARAMETER_MIN);
        retval.insert(TRIGGER_PARAMETER_MAX);
        retval.insert(TRIGGER_PARAMETER_MEAN);
        retval.insert(TRIGGER_PARAMETER_EQUAL_TO);
        retval.insert(TRIGGER_PARAMETER_GREATER_THAN);
        retval.insert(TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO);
        retval.insert(TRIGGER_PARAMETER_LESS_THAN);
        retval.insert(TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO);
        retval.insert(TRIGGER_PARAMETER_NOT_EQUAL_TO);
        retval.insert(TRIGGER_PARAMETER_X);
        retval.insert(TRIGGER_PARAMETER_Y);
        retval.insert(TRIGGER_PARAMETER_Z);
        retval.insert(TRIGGER_PARAMETER_TIME);
        retval.insert(TRIGGER_PARAMETER_DATE);
        retval.insert(TRIGGER_PARAMETER_RULE);
        retval.insert(TRIGGER_PARAMETER_UNAVAILABLE);

        return retval;
    }
};

#endif // ISOTRIGGER_H
