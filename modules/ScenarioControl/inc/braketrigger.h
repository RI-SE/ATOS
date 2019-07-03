#ifndef BRAKETRIGGER_H
#define BRAKETRIGGER_H

#include "trigger.h"

#include <list>

class BrakeTrigger : public Trigger
{
public:
    BrakeTrigger(TriggerID_t triggerID);
    ~BrakeTrigger();

    TriggerReturnCode_t parseParameters();

    TriggerReturnCode_t update(bool);

private:
    static constexpr TriggerTypeCode_t triggerTypeCode = TRIGGER_BRAKE;

    std::list<TriggerParameter_t> parameters;

    bool isBrakePressed = false, wasBrakePressed = false;

    std::set<Trigger::TriggerParameter_t> getAcceptedParameters()
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
            Trigger::TRIGGER_PARAMETER_ANY_EDGE,
            Trigger::TRIGGER_PARAMETER_EQUAL_TO,
            Trigger::TRIGGER_PARAMETER_NOT_EQUAL_TO
        };
    }

    TriggerReturnCode_t checkIfTriggered(void);

    enum TriggerMode {PRESSED, RELEASED, EDGE_RISING, EDGE_FALLING, EDGE_ANY} mode;
};

#endif // BRAKETRIGGER_H
