#include "trigger.h"

#include "logging.h"

Trigger::Trigger(TriggerID_t triggerID, TriggerTypeCode_t triggerType)
{
    this->triggerID = triggerID;
    this->triggerTypeCode = triggerType;
}

Trigger::~Trigger()
{
    parameters.clear();
}


/*!
 * \brief Trigger::checkTriggerParameter Checks if the queried parameter is within the list of accepted parameters
 * \param triggerParameter Queried parameter
 * \return According to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t Trigger::checkTriggerParameter(TriggerParameter_t triggerParameter) const
{
    std::set<TriggerParameter_t> acceptedParameters = getAcceptedParameters();
    for (const TriggerParameter_t param : acceptedParameters) {
        if (triggerParameter == param)
            return OK;
    }
    return NOT_OK;
}

/*!
 * \brief Trigger::appendParameter Adds a parameter to the parameter list if it is among the accepted parameters
 * \param triggerParameter Parameter to append
 * \return Return code according to ::TriggerReturnCode_t
 */
Trigger::TriggerReturnCode_t Trigger::appendParameter(Trigger::TriggerParameter_t triggerParameter)
{
    TriggerReturnCode_t retval = NOT_OK;

    if( (retval = checkTriggerParameter(triggerParameter)) != OK)
        return retval;

    parameters.push_back(triggerParameter);
    return parseParameters();
}

/*!
 * \brief Trigger::getParametersString Converts the list of loaded parameters into a string
 * \return String describing all parameters
 */
std::string Trigger::getParametersString() const
{
    std::string retval;
    if (parameters.size() == 0)
        return "EMPTY";

    for (const TriggerParameter_t param : parameters)
    {
        retval.append(getParameterAsString(param) + " ");
    }
    retval.pop_back();
    return retval;
}

/*!
 * \brief Trigger::getParameterAsString Converts a ::TriggerParameter_t into a string
 * \param param Parameter to be converted
 * \return String describing the parameter
 */
std::string Trigger::getParameterAsString(TriggerParameter_t param)
{
    switch (param)
    {
    case TRIGGER_PARAMETER_FALSE:
        return "FALSE";
    case TRIGGER_PARAMETER_TRUE:
        return "TRUE";
    case TRIGGER_PARAMETER_RELEASED:
        return "RELEASED";
    case TRIGGER_PARAMETER_PRESSED:
        return "PRESSED";
    case TRIGGER_PARAMETER_LOW:
        return "LOW";
    case TRIGGER_PARAMETER_HIGH:
        return "HIGH";
    case TRIGGER_PARAMETER_RISING_EDGE:
        return "RISING EDGE";
    case TRIGGER_PARAMETER_FALLING_EDGE:
        return "FALLING EDGE";
    case TRIGGER_PARAMETER_ANY_EDGE:
        return "ANY EDGE";
    case TRIGGER_PARAMETER_RELATIVE:
        return "RELATIVE";
    case TRIGGER_PARAMETER_ABSOLUTE:
        return "ABSOLUTE";
    case TRIGGER_PARAMETER_VALUE:
        return "VALUE";
    case TRIGGER_PARAMETER_MIN:
        return "MIN";
    case TRIGGER_PARAMETER_MAX:
        return "MAX";
    case TRIGGER_PARAMETER_MEAN:
        return "MEAN";
    case TRIGGER_PARAMETER_EQUAL_TO:
        return "EQUAL TO";
    case TRIGGER_PARAMETER_GREATER_THAN:
        return "GREATER THAN";
    case TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO:
        return "GREATER THAN OR EQUAL TO";
    case TRIGGER_PARAMETER_LESS_THAN:
        return "LESS THAN";
    case TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO:
        return "LESS THAN OR EQUAL TO";
    case TRIGGER_PARAMETER_NOT_EQUAL_TO:
        return "NOT EQUAL TO";
    case TRIGGER_PARAMETER_X:
        return "X";
    case TRIGGER_PARAMETER_Y:
        return "Y";
    case TRIGGER_PARAMETER_Z:
        return "Z";
    case TRIGGER_PARAMETER_TIME:
        return "TIME";
    case TRIGGER_PARAMETER_DATE:
        return "DATE";
    case TRIGGER_PARAMETER_RULE:
        return "RULE";
    case TRIGGER_PARAMETER_UNAVAILABLE:
        return "UNAVAILABLE";
    }
    return "<<unimplemented>>";
}

/*!
 * \brief Trigger::getTypeAsString Maps a ::TriggerTypeCode_t to a string
 * \param typeCode Type code
 * \return String describing the trigger type
 */
std::string Trigger::getTypeAsString(Trigger::TriggerTypeCode_t typeCode)
{
    switch (typeCode)
    {
    case TRIGGER_UNDEFINED:
        return "UNDEFINED";
    case TRIGGER_TYPE_1:
        return "TYPE 1";
    case TRIGGER_SPEED:
        return "SPEED";
    case TRIGGER_DISTANCE:
        return "DISTANCE";
    case TRIGGER_ACCELERATION:
        return "ACCELERATION";
    case TRIGGER_LANE_CHANGED:
        return "LANE CHANGED";
    case TRIGGER_LANE_OFFSET:
        return "LANE OFFSET";
    case TRIGGER_POSITION_REACHED:
        return "POSITION REACHED";
    case TRIGGER_POSITION_LEFT:
        return "POSITION LEFT";
    case TRIGGER_POSITION_OFFSET:
        return "POSITION OFFSET";
    case TRIGGER_STEERING_ANGLE:
        return "STEERING ANGLE";
    case TRIGGER_THROTTLE_VALUE:
        return "THROTTLE VALUE";
    case TRIGGER_BRAKE:
        return "BRAKE";
    case TRIGGER_ACTIVE_TRAJECTORY:
        return "ACTIVE TRAJECTORY";
    case TRIGGER_OTHER_OBJECT_FEATURE:
        return "OTHER OBJECT FEATURE";
    case TRIGGER_INFRASTRUCTURE:
        return "INFRASTRUCTURE";
    case TRIGGER_TEST_SCENARIO_EVENT:
        return "TEST SCENARIO EVENT";
    case TRIGGER_MISC_DIGITAL_INPUT:
        return "MISC DIGITAL INPUT";
    case TRIGGER_MISC_ANALOG_INPUT:
        return "MISC ANALOG INPUT";
    case TRIGGER_TIMER_EVENT_OCCURRED:
        return "TIMER EVENT OCCURRED";
    case TRIGGER_MODE_CHANGED:
        return "MODE CHANGED";
    case TRIGGER_UNAVAILABLE:
        return "TRIGGER UNAVAILABLE";
    }
    return "<<unimplemented>>";
}

/*!
 * \brief Trigger::isActive Check if the last update to tracked signal caused trigger to occur
 * \return Boolean according to trigger status
 */
bool Trigger::isActive() const
{
    if(wasTriggeredByLastUpdate == TRIGGER_OCCURRED)
        return true;
    else if(wasTriggeredByLastUpdate == NO_TRIGGER_OCCURRED)
        return false;
    throw std::logic_error("Trigger in undefined state");
}

/*!
 * \brief Trigger::getConfigurationMessageData Constructs a TRCMData struct from object members
 * \return A struct which can be sent on message bus
 */
TRCMData Trigger::getConfigurationMessageData(void) const
{
    TRCMData retval;
    retval.triggerID = triggerID;
    retval.triggerType = triggerTypeCode;

    if (triggerObjectIP == 0) LogMessage(LOG_LEVEL_WARNING, "Constructing trigger configuration message with no IP");

    retval.ip = triggerObjectIP;

    switch(parameters.size())
    {
    case 3:
        retval.triggerTypeParameter1 = parameters[0];
        retval.triggerTypeParameter2 = parameters[1];
        retval.triggerTypeParameter3 = parameters[2];
        break;
    case 2:
        retval.triggerTypeParameter1 = parameters[0];
        retval.triggerTypeParameter2 = parameters[1];
        retval.triggerTypeParameter3 = TRIGGER_PARAMETER_UNAVAILABLE;
        break;
    case 1:
        retval.triggerTypeParameter1 = parameters[0];
        retval.triggerTypeParameter2 = TRIGGER_PARAMETER_UNAVAILABLE;
        retval.triggerTypeParameter3 = TRIGGER_PARAMETER_UNAVAILABLE;
        break;
    case 0:
        retval.triggerTypeParameter1 = TRIGGER_PARAMETER_UNAVAILABLE;
        retval.triggerTypeParameter2 = TRIGGER_PARAMETER_UNAVAILABLE;
        retval.triggerTypeParameter3 = TRIGGER_PARAMETER_UNAVAILABLE;
        break;
    default:
        throw std::invalid_argument("Trigger contains too many parameters for an ISO message");
    }
    return retval;
}
