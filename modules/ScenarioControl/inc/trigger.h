#ifndef TRIGGER_H
#define TRIGGER_H

#include <cstdint>

class Trigger
{
public:
    Trigger();
    uint16_t getTriggerType();

private:
    enum TriggerType {
        UNDEFINED               = 0x0000,
        TRIGGER_TYPE_1          = 0x0001,
        SPEED                   = 0x0010,
        DISTANCE                = 0x0020,
        ACCELERATION            = 0x0030,
        LANE_CHANGED            = 0x0040,
        LANE_OFFSET             = 0x0050,
        POSITION_REACHED        = 0x0060,
        POSITION_LEFT           = 0x0061,
        POSITION_OFFSET         = 0x0062,
        STEERING_ANGLE          = 0x0070,
        THROTTLE_VALUE          = 0x0080,
        BRAKE                   = 0x0090,
        ACTIVE_TRAJECTORY       = 0x00A0,
        OTHER_OBJECT_FEATURE    = 0x00B0,
        INFRASTRUCTURE          = 0x00C0,
        TEST_SCENARIO_EVENT     = 0x00D0,
        MISC_DIGITAL_INPUT      = 0x00E0,
        MISC_ANALOG_INPUT       = 0x00F0,
        TIMER_EVENT_OCCURRED    = 0x0100,
        MODE_CHANGED            = 0x0110,
        UNAVAILABLE             = 0xFFFF
    } triggerType = UNDEFINED;


};

#endif // TRIGGER_H
