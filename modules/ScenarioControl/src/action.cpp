
#include "action.h"

/*!
 * \brief Action::Action Constructor for Action objects.
 * \param actionID ISO ID of the action
 * \param actionType ISO action type of the action
 * \param allowedNumberOfRuns Number of times the action is allowed to be run
 */
Action::Action(ActionID_t actionID, ActionType_t actionType, uint32_t allowedNumberOfRuns)
{
    this->actionID = actionID;
    this->actionType = actionType;
    this->remainingAllowedRuns = allowedNumberOfRuns;
}

/*!
 * \brief Action::execute Runs the action if allowed and decrements the number of remaining allowed action executions.
 * \return Value according to ::ActionReturnCode_t
 */
Action::ActionReturnCode_t Action::execute(void)
{
    if (remainingAllowedRuns == 0)
        return NO_REMAINING_RUNS;
    else {
        // TODO: Maybe add some more functionality when it is more well specified
        remainingAllowedRuns--;
        return OK;
    }
}
