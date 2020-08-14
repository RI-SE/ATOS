#include "externalaction.h"

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>

#include "logging.h"
#include "journal.h"
#include "util.h"
#include "maestroTime.h"

Action::ActionReturnCode_t ExternalAction::execute(void)
{
    EXACData data;
    struct timeval systemTime;

    if (remainingAllowedRuns == 0)
        return NO_REMAINING_RUNS;
    else {
		TimeSetToCurrentSystemTime(&systemTime);

        data.actionID = actionID;
		data.executionTime_qmsoW  = actionDelayTime_qms == 0 ? TimeGetAsGPSqmsOfWeek(&systemTime) : TimeGetAsGPSqmsOfWeek(&systemTime) + actionDelayTime_qms;

        data.ip = actionObjectIP;
		std::string type = getTypeAsString(getTypeCode());
		JournalRecordEvent("Executing action %s (ID %d) - to occur at time %u [seconds of week]",
						   type.c_str(), actionID, data.executionTime_qmsoW);

        LogMessage(LOG_LEVEL_INFO, "Sending execute action message over message bus (action ID %u)", actionID);
        if(iCommSendEXAC(data) == -1)
            return NOT_OK;

        remainingAllowedRuns--;
        return OK;
    }
}


// ******* Test scenario command action
TestScenarioCommandAction::TestScenarioCommandAction(ActionID_t actionID, uint32_t allowedNumberOfRuns)
	: ExternalAction(actionID, Action::ActionTypeCode_t::ACTION_TEST_SCENARIO_COMMAND, allowedNumberOfRuns)
{
}

Action::ActionParameter_t TestScenarioCommandAction::asParameterCode(const std::string &inputStr) const
{
	try {
		return Action::asParameterCode(inputStr);
	} catch (std::invalid_argument e) {
		std::string str = inputStr;
		for (char &ch : str)
			ch = toUpper(ch);
		if (!str.compare("SEND_START"))
			return ACTION_PARAMETER_VS_SEND_START;
		throw e;
	}
}

Action::ActionReturnCode_t TestScenarioCommandAction::appendParameter(std::string inputStr) {
	try {
		// String represented an action parameter defined by ISO
		ActionParameter_t param = asParameterCode(inputStr);
		return Action::appendParameter(param);
	} catch (std::invalid_argument e) {
		// String may have represented a number
		return parseNumericParameter(inputStr);
	}
}

Action::ActionReturnCode_t TestScenarioCommandAction::parseNumericParameter(std::string inputStr) {
	std::istringstream ss(inputStr);
	double param;

	if (ss >> param) {
		if (std::find(this->parameters.begin(), this->parameters.end(),
					  ACTION_PARAMETER_VS_SEND_START) != this->parameters.end()) {
			// Scenario command was start, interpret numeric parameter as delay
			struct timeval delay;
			delay.tv_sec = static_cast<long>(param);
			delay.tv_usec = static_cast<long>((param - delay.tv_sec)*1000000);
			this->setExecuteDelayTime(delay);
			return OK;
		}
		else {
			throw std::invalid_argument("Numeric parameter cannot be linked to test scenario command");
		}
	}
	else {
		throw std::invalid_argument("Test scenario command action unable to parse " + inputStr + " as numeric parameter");
	}
}


// ******* Infrastructure action
InfrastructureAction::InfrastructureAction(ActionID_t actionID, uint32_t allowedNumberOfRuns)
    : ExternalAction(actionID, Action::ActionTypeCode_t::ACTION_INFRASTRUCTURE, allowedNumberOfRuns)
{
}

Action::ActionParameter_t InfrastructureAction::asParameterCode(const std::string &inputStr) const
{
    try {
        return Action::asParameterCode(inputStr);
    } catch (std::invalid_argument e) {
        std::string str = inputStr;
        for (char &ch : str)
            ch = toUpper(ch);
        if (!str.compare("DENM_BRAKE_WARNING"))
            return ACTION_PARAMETER_VS_BRAKE_WARNING;
        throw e;
    }
}
