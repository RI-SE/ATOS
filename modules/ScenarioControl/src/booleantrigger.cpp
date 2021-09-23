#include "booleantrigger.h"

/*!
 * \brief BooleanTrigger::update Updates the tracked signal (i.e. which causes the trigger) to the value specified
 * \param isBrakeCurrentlyPressed Boolean describing if the boolean
 * \param Unused time of measurement
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BooleanTrigger::update(bool currentStateValue, struct timeval)
{
	wasStateTrue = isStateTrue;
	isStateTrue = currentStateValue;
	wasTriggeredByLastUpdate = checkIfTriggered();
	return wasTriggeredByLastUpdate;
}

/*!
 * \brief BooleanTrigger::checkIfTriggered Check if the trigger has occurred based on the mode and state
 * \return Value according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t BooleanTrigger::checkIfTriggered() const
{
	switch (mode) {
	case HIGH:
		return isStateTrue ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
	case LOW:
		return !isStateTrue ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
	case EDGE_ANY:
		return (isStateTrue != wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
	case EDGE_RISING:
		return (isStateTrue && !wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
	case EDGE_FALLING:
		return (!isStateTrue && wasStateTrue) ? TRIGGER_OCCURRED : NO_TRIGGER_OCCURRED;
	case INVALID_MODE:
		throw std::logic_error("Boolean trigger cannot be triggered if mode invalid");
	}
}


