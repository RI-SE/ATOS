#ifndef DISTANCETRIGGER_H
#define DISTANCETRIGGER_H

#include "booleantrigger.h"

#include <set>
#include <vector>
#include <string>

class DistanceTrigger : public BooleanTrigger
{
public:
	DistanceTrigger(TriggerID_t triggerID);

	TriggerReturnCode_t appendParameter(std::string inputStr) override;
	TriggerReturnCode_t parseParameters() override;

	using BooleanTrigger::update;
	TriggerReturnCode_t update(MonitorDataType newValue) override;

	void setTriggerDistance(double distance_m) { this->triggerDistance_m = distance_m; }
	void setReferencePoint(CartesianPosition point) { this->referencePoint = point; }

	double getTriggerDistance(void) const { return this->triggerDistance_m; }
	CartesianPosition getReferencePoint(void) const { return this->referencePoint; }
private:
	static constexpr CartesianPosition defaultReferencePoint = {0.0, 0.0, 0.0, 0.0, true, false};

	double triggerDistance_m;
	CartesianPosition referencePoint;
	enum {LESS_THAN, GREATER_THAN} oper = LESS_THAN;

	const std::set<TriggerParameter_t> getAcceptedParameters() const override
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

	TriggerReturnCode_t parseNumericParameter(std::string inputStr);
};
#endif // DISTANCETRIGGER_H
