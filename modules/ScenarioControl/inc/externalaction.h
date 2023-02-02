#ifndef EXTERNALACTION_H
#define EXTERNALACTION_H

#include "action.h"

namespace ATOS {
	class ExternalAction : public Action
	{
	public:
		using Action::Action;

		ActionReturnCode_t execute(ROSChannels::ExecuteAction::Pub&) override;
		ActionReturnCode_t parseParameters(void) override { return OK; }
	};

	class TestScenarioCommandAction : public ExternalAction {
	public:
		TestScenarioCommandAction(rclcpp::Logger, ActionID_t actionID = 0, uint32_t allowedNumberOfRuns = 1);

		ActionReturnCode_t appendParameter(std::string) override;
		ActionReturnCode_t parseParameters(void) override { return parameters.size() == 1 ? OK : NOT_OK; }
	protected:
		ActionParameter_t asParameterCode(const std::string &parameterCodeString) const override;
	private:
		const std::set<ActionParameter_t> getAcceptedParameters(void) const override
		{
			std::set<ActionParameter_t> accParams;
			accParams.insert(ACTION_PARAMETER_VS_SEND_START);
			return accParams;
		}
		ActionReturnCode_t parseNumericParameter(std::string);
	};

	class InfrastructureAction : public ExternalAction
	{
	public:
		InfrastructureAction(rclcpp::Logger, ActionID_t actionID = 0, uint32_t allowedNumberOfRuns = 1);
		ActionReturnCode_t parseParameters(void) override { return parameters.size() == 1 ? OK : NOT_OK; }
	protected:
		ActionParameter_t asParameterCode(const std::string &parameterCodeString) const;

	private:
		const std::set<ActionParameter_t> getAcceptedParameters(void) const
		{
			std::set<ActionParameter_t> accParams;
			accParams.insert(ACTION_PARAMETER_VS_BRAKE_WARNING);
			return accParams;
		}
	};
}
#endif
