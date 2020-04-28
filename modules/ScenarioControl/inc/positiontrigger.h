#ifndef POSITIONTRIGGER_H
#define POSITIONTRIGGER_H

#include "trigger.h"

#include <vector>
#include <set>

class PositionTrigger : public Trigger
{
public:
	using Trigger::Trigger;

	virtual TriggerReturnCode_t parseParameters() override = 0;

	TriggerReturnCode_t update(bool, struct timeval measurementTime) override;

protected:
	TriggerReturnCode_t checkIfTriggered(void) const override;

	bool isStateTrue = false;

private:
	virtual const std::set<TriggerParameter_t> getAcceptedParameters() const override
	{
		std::set<TriggerParameter_t> accParams;
		accParams.insert(TRIGGER_PARAMETER_X);
		accParams.insert(TRIGGER_PARAMETER_Y);
		accParams.insert(TRIGGER_PARAMETER_Z);
		accParams.insert(TRIGGER_PARAMETER_EQUAL_TO);
		accParams.insert(TRIGGER_PARAMETER_GREATER_THAN);
		accParams.insert(TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO);
		accParams.insert(TRIGGER_PARAMETER_LESS_THAN);
		accParams.insert(TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO);
		accParams.insert(TRIGGER_PARAMETER_NOT_EQUAL_TO);
		accParams.insert(TRIGGER_PARAMETER_RELATIVE);
		accParams.insert(TRIGGER_PARAMETER_ABSOLUTE);
		accParams.insert(TRIGGER_PARAMETER_MIN);
		accParams.insert(TRIGGER_PARAMETER_MAX);
		accParams.insert(TRIGGER_PARAMETER_MEAN);
		return accParams;
	}


};

#endif // POSITIONTRIGGER_H
