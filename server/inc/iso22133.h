#ifndef ISO22133_H
#define ISO22133_H
#ifdef __cplusplus
extern "C" {
#endif
/*! This file contains all definitions pertaining to the ISO standard 22133
 *
 *
 *
 *
 */

#include <inttypes.h>

#pragma pack(push,1)

typedef struct
{
  uint16_t SyncWordU16;
  uint8_t TransmitterIdU8;
  uint8_t MessageCounterU8;
  uint8_t AckReqProtVerU8;
  uint16_t MessageIdU16;
  uint32_t MessageLengthU32;
} HeaderType; //11 bytes

typedef struct
{
  uint16_t Crc;
} FooterType; //2 bytes

//! *************************** TRCM
#define COMMAND_TRCM_CODE 0x0011
typedef struct
{
    HeaderType header;
    uint16_t triggerIDValueID;
    uint16_t triggerIDContentLength;
    uint16_t triggerID;
    uint16_t triggerTypeValueID;
    uint16_t triggerTypeContentLength;
    uint16_t triggerType;
    uint16_t triggerTypeParameter1ValueID;
    uint16_t triggerTypeParameter1ContentLength;
    uint32_t triggerTypeParameter1;
    uint16_t triggerTypeParameter2ValueID;
    uint16_t triggerTypeParameter2ContentLength;
    uint32_t triggerTypeParameter2;
    uint16_t triggerTypeParameter3ValueID;
    uint16_t triggerTypeParameter3ContentLength;
    uint32_t triggerTypeParameter3;
    FooterType footer;
} TRCMType;

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
} TriggerTypeParameter_t;


//! *************************** ACCM
#define COMMAND_ACCM_CODE 0x0012
typedef struct
{
    HeaderType header;
    uint16_t actionIDValueID;
    uint16_t actionIDContentLength;
    uint16_t actionID;
    uint16_t actionTypeValueID;
    uint16_t actionTypeContentLength;
    uint16_t actionType;
    uint16_t actionTypeParameter1ValueID;
    uint16_t actionTypeParameter1ContentLength;
    uint32_t actionTypeParameter1;
    uint16_t actionTypeParameter2ValueID;
    uint16_t actionTypeParameter2ContentLength;
    uint32_t actionTypeParameter2;
    uint16_t actionTypeParameter3ValueID;
    uint16_t actionTypeParameter3ContentLength;
    uint32_t actionTypeParameter3;
    FooterType footer;
} ACCMType;

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
} ActionTypeParameter_t;


//! *************************** TREO
#define COMMAND_TREO_CODE 0x0013
typedef struct
{
    HeaderType header;
    uint16_t triggerIDValueID;
    uint16_t triggerIDContentLength;
    uint16_t triggerID;
    uint16_t timestamp_qmsowValueID;
    uint16_t timestamp_qmsowContentLength;
    uint32_t timestamp_qmsow;
    FooterType footer;
} TREOType;


//! *************************** EXAC
#define COMMAND_EXAC_CODE 0x0014
typedef struct
{
    HeaderType header;
    uint16_t actionIDValueID;
    uint16_t actionIDContentLength;
    uint16_t actionID;
    uint16_t executionTime_qmsoWValueID;
    uint16_t executionTime_qmsoWContentLength;
    uint32_t executionTime_qmsoW;
    FooterType footer;
} EXACType;


//! ACCM / EXAC / CATA value IDs
#define VALUE_ID_ACTION_ID 0x0002
#define VALUE_ID_ACTION_TYPE 0x0003
#define VALUE_ID_ACTION_TYPE_PARAM1 0x00A1
#define VALUE_ID_ACTION_TYPE_PARAM2 0x00A2
#define VALUE_ID_ACTION_TYPE_PARAM3 0x00A3
#define VALUE_ID_ACTION_EXECUTE_TIME 0x0003

//! TRCM / TREO / CATA value IDs
#define VALUE_ID_TRIGGER_ID 0x0001
#define VALUE_ID_TRIGGER_TYPE 0x0002
#define VALUE_ID_TRIGGER_TYPE_PARAM1 0x0011
#define VALUE_ID_TRIGGER_TYPE_PARAM2 0x0012
#define VALUE_ID_TRIGGER_TYPE_PARAM3 0x0013
#define VALUE_ID_TRIGGER_TIMESTAMP 0x0002

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
#endif
