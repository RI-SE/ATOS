#ifndef ACTION_H
#define ACTION_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <set>
#include <netinet/in.h>


#include "util.h"

#define ACTION_NUMBER_PARAMETER_FIELDS 3

class Action
{
public:
    /*! Typedefs */
    typedef enum {
        ACTION_NONE                     = 0x0000,
        ACTION_TYPE_1                   = 0x0001,
        ACTION_TYPE_2                   = 0x0002,
        ACTION_SET_SPEED                = 0x0010,
        ACTION_SET_DISTANCE             = 0x0020,
        ACTION_SET_ACCELERATION         = 0x0030,
        ACTION_LANE_CHANGE              = 0x0040,
        ACTION_LANE_OFFSET              = 0x0050,
        ACTION_SET_POSITION             = 0x0060,
        ACTION_SET_STEERING_ANGLE       = 0x0070,
        ACTION_SET_TRHOTTLE_VALUE       = 0x0080,
        ACTION_BRAKE                    = 0x0090,
        ACTION_FOLLOW_TRAJECTORY        = 0x00A0,
        ACTION_OTHER_OBJECT_FEATURE     = 0x00B0,
        ACTION_INFRASTRUCTURE           = 0x00C0,
        ACTION_TEST_SCENARIO_COMMAND    = 0x00D0,
        ACTION_MISC_DIGITAL_OUTPUT      = 0x00E0,
        ACTION_MISC_ANALOG_OUTPUT       = 0x00F0,
        ACTION_START_TIMER              = 0x0100,
        ACTION_MODE_CHANGE              = 0x0110,
        ACTION_UNAVAILABLE              = 0xFFFF
    } ActionTypeCode_t;

    typedef enum {
        ACTION_PARAMETER_SET_FALSE          = 0x00000000,
        ACTION_PARAMETER_SET_TRUE           = 0x00000001,
        ACTION_PARAMETER_RELEASE            = 0x00000010,
        ACTION_PARAMETER_PRESS              = 0x00000011,
        ACTION_PARAMETER_SET_VALUE          = 0x00000020,
        ACTION_PARAMETER_MIN                = 0x00000040,
        ACTION_PARAMETER_MAX                = 0x00000041,
        ACTION_PARAMETER_X                  = 0x00000070,
        ACTION_PARAMETER_Y                  = 0x00000071,
        ACTION_PARAMETER_Z                  = 0x00000072,
        ACTION_PARAMETER_VS_BRAKE_WARNING   = 0xA0000000,
        ACTION_PARAMETER_UNAVAILABLE        = 0xFFFFFFFF
    } ActionParameter_t;

    typedef enum {
        OK,
        NOT_OK,
        INVALID_ARGUMENT,
        NO_REMAINING_RUNS
    } ActionReturnCode_t;

    typedef uint16_t ActionID_t;

    /*! Constructor */
    Action(ActionID_t actionID = 0, ActionTypeCode_t actionTypeCode = ACTION_NONE, uint32_t allowedNumberOfRuns = 1);

    /*! Destructor */
    virtual ~Action() { parameters.clear(); }

    /*! Getters */
    ActionID_t getID() const { return actionID; }
    ActionTypeCode_t getTypeCode() const { return actionTypeCode; }

    /*! Run the action once, if allowed */
    virtual ActionReturnCode_t execute(void);

    /*! To string */
    friend std::ostream& operator<<(std::ostream &strm, const Action &act) {
        return strm << "ACTION ID " << act.actionID <<
                       " TYPE " << getTypeAsString(act.getTypeCode()) <<
                       " PARAMETERS " << act.getParametersString();
    }

    static std::string getTypeAsString(ActionTypeCode_t type);
    static std::string getParameterAsString(ActionParameter_t param);
    std::string getParametersString(void) const;
    ACCMData getConfigurationMessageData(void) const;
    in_addr_t getObjectIP(void) const { return actionObjectIP; }
    void setObjectIP(in_addr_t ipAddr) { actionObjectIP = ipAddr; }

    ActionReturnCode_t appendParameter(ActionParameter_t actionParameter);

protected:
    ActionTypeCode_t actionTypeCode = ACTION_NONE;
    ActionID_t actionID = 0;
    uint32_t remainingAllowedRuns = 0;
    std::vector<ActionParameter_t> parameters;
    uint32_t actionDelayTime_qms;
    in_addr_t actionObjectIP = 0;

    ActionReturnCode_t checkActionParameter(ActionParameter_t actionParameter) const;

private:
    virtual const std::set<ActionParameter_t> getAcceptedParameters(void) const
    {
        std::set<ActionParameter_t> accParams;
        accParams.insert(ACTION_PARAMETER_SET_FALSE);
        accParams.insert(ACTION_PARAMETER_SET_TRUE);
        accParams.insert(ACTION_PARAMETER_RELEASE);
        accParams.insert(ACTION_PARAMETER_PRESS);
        accParams.insert(ACTION_PARAMETER_SET_VALUE);
        accParams.insert(ACTION_PARAMETER_MIN);
        accParams.insert(ACTION_PARAMETER_MAX);
        accParams.insert(ACTION_PARAMETER_X);
        accParams.insert(ACTION_PARAMETER_Y);
        accParams.insert(ACTION_PARAMETER_Z);
        accParams.insert(ACTION_PARAMETER_VS_BRAKE_WARNING);
        accParams.insert(ACTION_PARAMETER_UNAVAILABLE);
        return accParams;
    }
};

#endif
