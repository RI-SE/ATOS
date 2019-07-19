#ifndef BOOLEANTRIGGER_H
#define BOOLEANTRIGGER_H

#include "trigger.h"

#include <vector>
#include <set>

class BooleanTrigger : public Trigger
{
public:
    using Trigger::Trigger;

    virtual TriggerReturnCode_t parseParameters() override = 0;

    TriggerReturnCode_t update(bool) override;

protected:
    TriggerReturnCode_t checkIfTriggered(void) override;

    enum TriggerMode {
        INVALID_MODE,
        HIGH,
        LOW,
        EDGE_RISING,
        EDGE_FALLING,
        EDGE_ANY}
    mode = INVALID_MODE;

    bool isStateTrue = false, wasStateTrue = false;

private:

    virtual std::set<Trigger::TriggerParameter_t> getAcceptedParameters() override
    {
        return {
            Trigger::TRIGGER_PARAMETER_FALSE,
            Trigger::TRIGGER_PARAMETER_TRUE,
            Trigger::TRIGGER_PARAMETER_RISING_EDGE,
            Trigger::TRIGGER_PARAMETER_FALLING_EDGE,
            Trigger::TRIGGER_PARAMETER_ANY_EDGE
        };
    }


};

#endif // BOOLEANTRIGGER_H
