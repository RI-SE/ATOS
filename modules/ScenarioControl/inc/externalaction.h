#ifndef EXTERNALACTION_H
#define EXTERNALACTION_H

#include "action.h"


class ExternalAction : public Action
{
public:
    using Action::Action;

    ActionReturnCode_t execute(void) override;
};

class InfrastructureAction : public ExternalAction
{
public:
    InfrastructureAction(ActionID_t actionID = 0, uint32_t allowedNumberOfRuns = 1);

private:
    const std::set<ActionParameter_t> getAcceptedParameters(void) const
    {
        std::set<ActionParameter_t> accParams;
        accParams.insert(ACTION_PARAMETER_VS_BRAKE_WARNING);
        return accParams;
    }
};

#endif
