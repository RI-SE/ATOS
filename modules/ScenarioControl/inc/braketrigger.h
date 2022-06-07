#ifndef BRAKETRIGGER_H
#define BRAKETRIGGER_H

#include "booleantrigger.h"

#include <set>
#include <vector>
namespace maestro {
    class BrakeTrigger : public BooleanTrigger
    {
    public:
        BrakeTrigger(TriggerID_t triggerID);
        TriggerReturnCode_t parseParameters() override;

        using BooleanTrigger::update;
        TriggerReturnCode_t update(double newValue, struct timeval measurementTime) override;
        TriggerReturnCode_t update(float newValue, struct timeval measurementTime) override { return update(static_cast<double>(newValue),measurementTime); }

        void setBrakeRetardationThreshold(double threshold_m_s2);
    private:

        double brakeRetardationThreshold_m_s2;
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
}
#endif // BRAKETRIGGER_H
