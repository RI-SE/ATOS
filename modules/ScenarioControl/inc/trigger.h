#ifndef TRIGGER_H
#define TRIGGER_H

#include <cstdint>
#include <set>
#include <iostream>
#include <vector>



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
    } TriggerTypeCode_t;

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

    typedef uint16_t TriggerID_t;

    typedef enum {
        OK,
        NOT_OK,
        INVALID_ARGUMENT,
        TRIGGER_OCCURRED,
        NO_TRIGGER_OCCURRED
    } TriggerReturnCode_t;


    /*! Constructor */
    Trigger(TriggerID_t triggerID, TriggerTypeCode_t triggerType);


    /*! Destructor */
    virtual ~Trigger();


    /*! Getters */
    virtual TriggerTypeCode_t getTypeCode() { return triggerTypeCode; }
    uint16_t getID() { return triggerID; }
    std::vector<TriggerParameter_t> getParameters() { return parameters; }


    /*! Setters */
    void setID(uint16_t triggerID) { this->triggerID = triggerID; }

    TriggerReturnCode_t appendParameter(TriggerParameter_t triggerParameter);
    virtual TriggerReturnCode_t parseParameters() = 0;


    /*! To string */
    friend std::ostream& operator<<(std::ostream &strm, Trigger &trig) {
        return strm << "TRIGGER ID " << trig.triggerID <<
                       " TYPE " << getTypeAsString(trig.getTypeCode()) <<
                       " PARAMETERS " << trig.getParametersString();
    }

    static std::string getTypeAsString(TriggerTypeCode_t typeCode);
    static std::string getParameterAsString(TriggerParameter_t param);
    std::string getParametersString();

    virtual TriggerReturnCode_t update(void)    { return INVALID_ARGUMENT; }
    virtual TriggerReturnCode_t update(bool)    { return INVALID_ARGUMENT; }
    virtual TriggerReturnCode_t update(char)    { return INVALID_ARGUMENT; }
    virtual TriggerReturnCode_t update(int)     { return INVALID_ARGUMENT; }
    virtual TriggerReturnCode_t update(float)   { return INVALID_ARGUMENT; }
    virtual TriggerReturnCode_t update(double)  { return INVALID_ARGUMENT; }

protected:
    TriggerReturnCode_t checkTriggerParameter(TriggerParameter_t triggerParameter);
    TriggerTypeCode_t triggerTypeCode;

private:
    TriggerID_t triggerID;
    std::vector<TriggerParameter_t> parameters;

    virtual std::set<TriggerParameter_t> getAcceptedParameters()
        { return {TRIGGER_PARAMETER_UNAVAILABLE}; }

    virtual TriggerReturnCode_t checkIfTriggered(void) = 0;
};

#endif // TRIGGER_H
