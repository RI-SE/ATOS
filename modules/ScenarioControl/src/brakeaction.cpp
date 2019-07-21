#include "brakeaction.h"
#include "logging.h"
#include "util.h"

Action::ActionReturnCode_t BrakeAction::execute(void)
{
    constexpr unsigned int datalen = sizeof(actionID) + sizeof(actionTypeCode) + sizeof(brakeObjAddr) + ACTION_NUMBER_PARAMETER_FIELDS*sizeof(parameters);
    char commData[datalen];
    char* ptr = commData;

    if (parameters.size() > ACTION_NUMBER_PARAMETER_FIELDS)
        return INVALID_ARGUMENT;

    bzero(commData,datalen);

    if (remainingAllowedRuns == 0)
        return NO_REMAINING_RUNS;
    else {
        LogMessage(LOG_LEVEL_DEBUG, "Action ID %u triggered", actionID);

        memcpy(ptr, &actionID, sizeof(actionID));
        ptr += sizeof(actionID);

        memcpy(ptr, &actionTypeCode, sizeof(actionTypeCode));
        ptr += sizeof(actionTypeCode);

        memcpy(ptr, &brakeObjAddr, sizeof(brakeObjAddr));
        ptr += sizeof(brakeObjAddr);

        for (ActionParameter_t ap : parameters)
        {
            memcpy(ptr, &ap, sizeof(ap));
            ptr += sizeof(ap);
        }

        LogMessage(LOG_LEVEL_INFO, "Sending brake message (action ID %u)", actionID);
        if(iCommSend(COMM_BRAKE,commData,datalen) == -1)
            return NOT_OK;

        remainingAllowedRuns--;
        return OK;
    }
}
