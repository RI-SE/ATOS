#include "externalaction.h"
#include "logging.h"
#include "util.h"
#include "maestroTime.h"

Action::ActionReturnCode_t ExternalAction::execute(void)
{
    EXACData data;
    struct timeval systemTime;

    if (remainingAllowedRuns == 0)
        return NO_REMAINING_RUNS;
    else {
        TimeSetToCurrentSystemTime(&systemTime); // TODO: Set system time according to timecontrol

        data.actionID = actionID;
        data.executionTime_qmsoW  = actionDelayTime_qms == 0 ? 0 : TimeGetAsGPSqmsOfWeek(&systemTime) + actionDelayTime_qms;

        data.ip = actionObjectIP;

        LogMessage(LOG_LEVEL_INFO, "Sending execute action message over message bus (action ID %u)", actionID);
        if(iCommSendEXAC(data) == -1)
            return NOT_OK;

        remainingAllowedRuns--;
        return OK;
    }
}

InfrastructureAction::InfrastructureAction(ActionID_t actionID, uint32_t allowedNumberOfRuns)
    : ExternalAction(actionID, Action::ActionTypeCode_t::ACTION_INFRASTRUCTURE, allowedNumberOfRuns)
{
}
