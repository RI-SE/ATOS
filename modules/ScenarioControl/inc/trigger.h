#ifndef TRIGGER_H
#define TRIGGER_H

#include <cstdint>
#include <set>
#include <iostream>

class Trigger
{
public:
    /*! Typedefs */
    typedef enum {
        TRIGGER_UNDEFINED               = 0x0000,
        TRIGGER_TYPE_1                  = 0x0001,
        TRIGGER_SPEED                   = 0x0010,
        TRIGGER_DISTANCE                = 0x0020,
        TRIGGER_ACCELERATION            = 0x0030,
        TRIGGER_LANE_CHANGED            = 0x0040,
        TRIGGER_LANE_OFFSET             = 0x0050,
        TRIGGER_POSITION_REACHED        = 0x0060,
        TRIGGER_POSITION_LEFT           = 0x0061,
        TRIGGER_POSITION_OFFSET         = 0x0062,
        TRIGGER_STEERING_ANGLE          = 0x0070,
        TRIGGER_THROTTLE_VALUE          = 0x0080,
        TRIGGER_BRAKE                   = 0x0090,
        TRIGGER_ACTIVE_TRAJECTORY       = 0x00A0,
        TRIGGER_OTHER_OBJECT_FEATURE    = 0x00B0,
        TRIGGER_INFRASTRUCTURE          = 0x00C0,
        TRIGGER_TEST_SCENARIO_EVENT     = 0x00D0,
        TRIGGER_MISC_DIGITAL_INPUT      = 0x00E0,
        TRIGGER_MISC_ANALOG_INPUT       = 0x00F0,
        TRIGGER_TIMER_EVENT_OCCURRED    = 0x0100,
        TRIGGER_MODE_CHANGED            = 0x0110,
        TRIGGER_UNAVAILABLE             = 0xFFFF
    } TriggerType_t;

    typedef enum {
        TRIGGER_PARAMETER_FALSE                     = 0x00000000,
        TRIGGER_PARAMETER_TRUE                      = 0x00000001,
        TRIGGER_PARAMETER_RELEASED                  = 0x00000010,
        TRIGGER_PARAMETER_PRESSED                   = 0x00000011,
        TRIGGER_PARAMETER_LOW                       = 0x00000020,
        TRIGGER_PARAMETER_HIGH                      = 0x00000021,
        TRIGGER_PARAMETER_RISING_EDGE               = 0x00000022,
        TRIGGER_PARAMETER_FALLING_EDGE              = 0x00000023,
        TRIGGER_PARAMETER_ANY_EDGE                  = 0x00000024,
        TRIGGER_PARAMETER_RELATIVE                  = 0x00000030,
        TRIGGER_PARAMETER_ABSOLUTE                  = 0x00000031,
        TRIGGER_PARAMETER_VALUE                     = 0x00000040,
        TRIGGER_PARAMETER_MIN                       = 0x00000050,
        TRIGGER_PARAMETER_MAX                       = 0x00000051,
        TRIGGER_PARAMETER_MEAN                      = 0x00000052,
        TRIGGER_PARAMETER_EQUAL_TO                  = 0x00000060,
        TRIGGER_PARAMETER_GREATER_THAN              = 0x00000061,
        TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO  = 0x00000062,
        TRIGGER_PARAMETER_LESS_THAN                 = 0x00000063,
        TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO     = 0x00000064,
        TRIGGER_PARAMETER_NOT_EQUAL_TO              = 0x00000065,
        TRIGGER_PARAMETER_X                         = 0x00000070,
        TRIGGER_PARAMETER_Y                         = 0x00000071,
        TRIGGER_PARAMETER_Z                         = 0x00000072,
        TRIGGER_PARAMETER_TIME                      = 0x00000080,
        TRIGGER_PARAMETER_DATE                      = 0x00000081,
        TRIGGER_PARAMETER_RULE                      = 0x000000A0,
        TRIGGER_PARAMETER_UNAVAILABLE               = 0xFFFFFFFF
    } TriggerParameter_t;

    typedef enum {OK} TriggerReturnCode_t;

    /*! Constructor */
    Trigger(TriggerType_t triggerID);

    /*! Destructor */
    virtual ~Trigger();

    /*! Getters */
    virtual TriggerType_t getType();
    uint16_t getID() { return this->triggerID; }

    /*! Setters */
    void setID(uint16_t triggerID) { this->triggerID = triggerID; }

    virtual TriggerReturnCode_t appendTriggerParameter(uint32_t triggerParameter);

    /*! To string */
    friend std::ostream& operator<<(std::ostream &strm, const Trigger &a) {
        return strm << "TODO";
    }


private:
    uint16_t triggerID;
    TriggerType_t triggerType = TRIGGER_UNDEFINED;
    std::set<TriggerParameter_t> triggerParameters;

};

#endif // TRIGGER_H
