
#include "action.h"

Action::Action(ActionID_t actionID, ActionType_t actionType, uint32_t numberOfRuns)
{
    this->actionID = actionID;
    this->actionType = actionType;
    this->remainingRuns = numberOfRuns;
}

/*!
 * \brief Action::runOnce Runs the action if allowed and decrements the number of remaining allowed action executions.
 * \return Value according to ::ActionReturnCode_t
 */
Action::ActionReturnCode_t Action::execute(void)
{
    if (remainingRuns == 0)
        return NO_REMAINING_RUNS;
    else {
        // TODO: Maybe add some more functionality when it is more well specified
        remainingRuns--;
        return OK;
    }
}
