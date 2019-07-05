#ifndef ACTION_H
#define ACTION_H

#include <cstdint>

class Action
{
public:
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
    } ActionType_t;

    typedef enum {
        ACTION_PARAMETER_SET_FALSE      = 0x00000000,
        ACTION_PARAMETER_SET_TRUE       = 0x00000001,
        ACTION_PARAMETER_RELEASE        = 0x00000010,
        ACTION_PARAMETER_PRESS          = 0x00000011,
        ACTION_PARAMETER_SET_VALUE      = 0x00000020,
        ACTION_PARAMETER_MIN            = 0x00000040,
        ACTION_PARAMETER_MAX            = 0x00000041,
        ACTION_PARAMETER_X              = 0x00000070,
        ACTION_PARAMETER_Y              = 0x00000071,
        ACTION_PARAMETER_Z              = 0x00000072,
        ACTION_PARAMETER_UNAVAILABLE    = 0xFFFFFFFF
    } ActionParameter_t;

    typedef enum {
        OK,
        NOT_OK,
        INVALID_ARGUMENT
    } ActionReturnCode_t;

    typedef uint16_t ActionID_t;
    Action(ActionID_t actionID, ActionType_t actionType, uint32_t numberOfFires);

    ActionID_t getID() { return actionID; }
    ActionType_t getType() { return actionType; }

private:
    ActionID_t actionID = 0;
    ActionType_t actionType = ACTION_NONE;

    uint32_t remainingFires = 0;
};

#endif
