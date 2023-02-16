/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef BOOLEANTRIGGER_H
#define BOOLEANTRIGGER_H

#include "trigger.h"

#include <vector>
#include <set>
namespace ATOS {
    class BooleanTrigger : public Trigger
    {
    public:
        using Trigger::Trigger;

        virtual TriggerReturnCode_t parseParameters() override = 0;

        TriggerReturnCode_t update(bool, struct timeval measurementTime) override;

    protected:
        TriggerReturnCode_t checkIfTriggered(void) const override;

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
        virtual const std::set<TriggerParameter_t> getAcceptedParameters() const override
        {
            std::set<TriggerParameter_t> accParams;
            accParams.insert(TRIGGER_PARAMETER_FALSE);
            accParams.insert(TRIGGER_PARAMETER_TRUE);
            accParams.insert(TRIGGER_PARAMETER_RISING_EDGE);
            accParams.insert(TRIGGER_PARAMETER_FALLING_EDGE);
            accParams.insert(TRIGGER_PARAMETER_ANY_EDGE);
            return accParams;
        }
    };
}
#endif // BOOLEANTRIGGER_H
