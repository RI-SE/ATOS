#include "externalaction.h"
#include "logging.h"
#include "util.h"

Action::ActionReturnCode_t ExternalAction::execute(void)
{
    EXACData data;

    if (remainingAllowedRuns == 0)
        return NO_REMAINING_RUNS;
    else {
        data.actionID = actionID;
        data.delayTime_qms = actionDelayTime_qms;
        data.ip = actionObjectIP;

        LogMessage(LOG_LEVEL_INFO, "Sending execute action message over message bus (action ID %u)", actionID);
        if(iCommSendEXAC(data) == -1)
            return NOT_OK;

        remainingAllowedRuns--;
        return OK;
    }
}

InfrastructureAction::InfrastructureAction(ActionID_t actionID, uint32_t allowedNumberOfRuns)
    : ExternalAction(actionID, Action::ACTION_INFRASTRUCTURE, allowedNumberOfRuns)
{
}
