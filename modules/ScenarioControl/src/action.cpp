
#include <limits>
#include "action.h"

#include "logging.h"
#include "maestroTime.h"



/*!
 * \brief Action::Action Constructor for Action objects.
 * \param actionID ISO ID of the action
 * \param actionType ISO action type of the action
 * \param allowedNumberOfRuns Number of times the action is allowed to be run
 */
Action::Action(ActionID_t actionID, ActionTypeCode_t actionType, uint32_t allowedNumberOfRuns)
{
    this->actionID = actionID;
    this->actionTypeCode = actionType;
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
        LogMessage(LOG_LEVEL_DEBUG, "Executing action with ID %u", actionID);
        remainingAllowedRuns--;
        return OK;
    }
}

std::string Action::getParametersString() const
{
    std::string retval;
    if (parameters.size() == 0)
        return "EMPTY";

    for (const ActionParameter_t param : parameters)
    {
        retval.append(getParameterAsString(param) + " ");
    }
    retval.pop_back();
    return retval;
}

/*!
 * \brief Action::getParameterAsString Converts an ::ActionParameter_t into a string
 * \param param Parameter to be converted
 * \return String describing the parameter
 */
std::string Action::getParameterAsString(ActionParameter_t param)
{
    switch (param) {
    case ACTION_PARAMETER_SET_FALSE:
        return "SET FALSE";
    case ACTION_PARAMETER_SET_TRUE:
        return "SET TRUE";
    case ACTION_PARAMETER_RELEASE:
        return "RELEASE";
    case ACTION_PARAMETER_PRESS:
        return "PRESS";
    case ACTION_PARAMETER_SET_VALUE:
        return "SET VALUE";
    case ACTION_PARAMETER_MIN:
        return "MIN";
    case ACTION_PARAMETER_MAX:
        return "MAX";
    case ACTION_PARAMETER_X:
        return "X";
    case ACTION_PARAMETER_Y:
        return "Y";
    case ACTION_PARAMETER_Z:
        return "Z";
    case ACTION_PARAMETER_UNAVAILABLE:
        return "UNAVAILABLE";
    }
    return "<<unimplemented>>";
}

/*!
 * \brief Action::getTypeAsString Maps an ::ActionTypeCode_t to a string
 * \param typeCode Type code
 * \return String describing the action type
 */
std::string Action::getTypeAsString(Action::ActionTypeCode_t typeCode)
{
    switch (typeCode)
    {
    case (ACTION_NONE):
        return "NONE";
    case (ACTION_TYPE_1):
        return "TYPE 1";
    case ACTION_TYPE_2:
        return "TYPE 2";
    case ACTION_SET_SPEED:
        return "SET SPEED";
    case ACTION_SET_DISTANCE:
        return "SET DISTANCE";
    case ACTION_SET_ACCELERATION:
        return "SET ACCELERATION";
    case ACTION_LANE_CHANGE:
        return "LANE CHANGE";
    case ACTION_LANE_OFFSET:
        return "LANE OFFSET";
    case ACTION_SET_POSITION:
        return "SET POSITION";
    case ACTION_SET_STEERING_ANGLE:
        return "SET STEERING ANGLE";
    case ACTION_SET_TRHOTTLE_VALUE:
        return "SET THROTTLE VALUE";
    case ACTION_BRAKE:
        return "BRAKE";
    case ACTION_FOLLOW_TRAJECTORY:
        return "FOLLOW TRAJECTORY";
    case ACTION_OTHER_OBJECT_FEATURE:
        return "OTHER OBJECT FEATURE";
    case ACTION_INFRASTRUCTURE:
        return "INFRASTRUCTURE";
    case ACTION_TEST_SCENARIO_COMMAND:
        return "TEST SCENARIO COMMAND";
    case ACTION_MISC_DIGITAL_OUTPUT:
        return "MISC DIGITAL OUTPUT";
    case ACTION_MISC_ANALOG_OUTPUT:
        return "MISC ANALOG OUTPUT";
    case ACTION_START_TIMER:
        return "START TIMER";
    case ACTION_MODE_CHANGE:
        return "MODE CHANGE";
    case ACTION_UNAVAILABLE:
        return "ACTION UNAVAILABLE";
    }
    return "<<unimplemented>>";
}

/*!
 * \brief Action::checkActionParameter Checks if the queried parameter is within the list of accepted parameters
 * \param actionParameter Queried parameter
 * \return According to ::ActionReturnCode_t
 */
Action::ActionReturnCode_t Action::checkActionParameter(ActionParameter_t actionParameter) const
{
    std::set<ActionParameter_t> acceptedParameters = getAcceptedParameters();
    for (const ActionParameter_t param : acceptedParameters) {
        if (actionParameter == param)
            return OK;
    }
    return NOT_OK;
}

/*!
 * \brief Action::appendParameter
 * \param actionParameter
 * \return
 */
Action::ActionReturnCode_t Action::appendParameter(ActionParameter_t actionParameter)
{
    ActionReturnCode_t retval = NOT_OK;

    if( (retval = checkActionParameter(actionParameter)) != OK)
        return retval;

    parameters.push_back(actionParameter);
    return OK;
}

/*!
 * \brief Action::getConfigurationMessageData Constructs a ACCMData struct from object members
 * \return A struct which can be sent on message bus
 */
ACCMData Action::getConfigurationMessageData(void) const
{
    ACCMData retval;
    retval.actionID = actionID;
    retval.actionType = actionTypeCode;

    if (actionObjectIP == 0) LogMessage(LOG_LEVEL_WARNING, "Constructing action configuration message with no IP");

    retval.ip = actionObjectIP;

    switch(parameters.size())
    {
    case 3:
        retval.actionTypeParameter1 = parameters[0];
        retval.actionTypeParameter2 = parameters[1];
        retval.actionTypeParameter3 = parameters[2];
        break;
    case 2:
        retval.actionTypeParameter1 = parameters[0];
        retval.actionTypeParameter2 = parameters[1];
        retval.actionTypeParameter3 = ACTION_PARAMETER_UNAVAILABLE;
        break;
    case 1:
        retval.actionTypeParameter1 = parameters[0];
        retval.actionTypeParameter2 = ACTION_PARAMETER_UNAVAILABLE;
        retval.actionTypeParameter3 = ACTION_PARAMETER_UNAVAILABLE;
        break;
    case 0:
        retval.actionTypeParameter1 = ACTION_PARAMETER_UNAVAILABLE;
        retval.actionTypeParameter2 = ACTION_PARAMETER_UNAVAILABLE;
        retval.actionTypeParameter3 = ACTION_PARAMETER_UNAVAILABLE;
        break;
    default:
        throw std::invalid_argument("Action contains too many parameters for an ISO message");
    }
    return retval;
}

/*!
 * \brief Action::setExecuteDalayTime Sets the delay from when EXAC is sent to when action is to be executed
 * \param tm Timeval struct representing the delay
 */
void Action::setExecuteDalayTime(struct timeval tm)
{
    int64_t time_qms = tm.tv_sec*4000 + tm.tv_usec/250;

    if (time_qms < 0) throw std::invalid_argument("Attempted to set a negative execution delay");
    if (time_qms > std::numeric_limits<uint32_t>::max()) throw std::invalid_argument("Attempted to set a too large execution delay");

    actionDelayTime_qms = static_cast<uint32_t>(time_qms);
}

/*!
 * \brief Action::getExecuteDelayTime Returns the delay time
 * \return Delay time as timeval struct
 */
struct timeval Action::getExecuteDelayTime(void) const
{
    struct timeval tm;
    tm.tv_sec = actionDelayTime_qms / 4000;
    tm.tv_usec = (actionDelayTime_qms % 4000) * 250;
    return tm;
}
