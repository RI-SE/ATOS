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
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>

#define ISO_PROTOCOL_VERSION 2
#define ACK_REQ 0

#define ISO_SYNC_WORD 0x7E7E

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



//! Predefined integer values with special meaning
#define LATITUDE_UNAVAILABLE_VALUE 900000000001
#define LATITUDE_ONE_DEGREE_VALUE 10000000000
#define LONGITUDE_UNAVAILABLE_VALUE 1800000000001
#define LONGITUDE_ONE_DEGREE_VALUE 10000000000
#define ALTITUDE_UNAVAILABLE_VALUE 800001
#define ALTITUDE_ONE_METER_VALUE 100
#define DATE_UNAVAILABLE_VALUE 0
#define GPS_WEEK_UNAVAILABLE_VALUE 10001
#define GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE 2419200000
#define MAX_WAY_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_WAY_DEVIATION_ONE_METER_VALUE 1000
#define MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_LATERAL_DEVIATION_ONE_METER_VALUE 1000
#define MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE 0
#define MIN_POSITIONING_ACCURACY_ONE_METER_VALUE 1000 // ISO specification unclear on this value



//! *************************** OSEM
typedef struct
{
  HeaderType header;
  uint16_t latitudeValueID;
  uint16_t latitudeContentLength;
  int64_t latitude;
  uint16_t longitudeValueID;
  uint16_t longitudeContentLength;
  int64_t longitude;
  uint16_t altitudeValueID;
  uint16_t altitudeContentLength;
  int32_t altitude;
  uint16_t dateValueID;
  uint16_t dateContentLength;
  uint32_t date;
  uint16_t GPSWeekValueID;
  uint16_t GPSWeekContentLength;
  uint16_t GPSWeek;
  uint16_t GPSQmsOfWeekValueID;
  uint16_t GPSQmsOfWeekContentLength;
  uint32_t GPSQmsOfWeek;
  uint16_t maxWayDeviationValueID;
  uint16_t maxWayDeviationContentLength;
  uint16_t maxWayDeviation;
  uint16_t maxLateralDeviationValueID;
  uint16_t maxLateralDeviationContentLength;
  uint16_t maxLateralDeviation;
  uint16_t minPosAccuracyValueID;
  uint16_t minPosAccuracyContentLength;
  uint16_t minPosAccuracy;
  FooterType footer;
} OSEMType; //85 bytes

//! OSEM value IDs
#define VALUE_ID_OSEM_LATITUDE 0x0020
#define VALUE_ID_OSEM_LONGITUDE 0x0021
#define VALUE_ID_OSEM_ALTITUDE 0x0022
#define VALUE_ID_OSEM_DATE 0x0004
#define VALUE_ID_OSEM_GPS_WEEK 0x0003
#define VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK 0x0002
#define VALUE_ID_OSEM_MAX_WAY_DEVIATION 0x0070
#define VALUE_ID_OSEM_MAX_LATERAL_DEVIATION 0x0072
#define VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY 0x0074


//! *************************** OSTM
typedef struct
{
  HeaderType header;
  uint16_t stateValueID;
  uint16_t stateContentLength;
  uint8_t state;
  FooterType footer;
} OSTMType; //16 bytes

//! OSTM value IDs
#define VALUE_ID_OSTM_STATE_CHANGE_REQUEST 0x0064
typedef enum {
	OBJECT_COMMAND_ARM = 0x02,				//!< Request to arm the target object
	OBJECT_COMMAND_DISARM = 0x03,			//!< Request to disarm the target object
	OBJECT_COMMAND_REMOTE_CONTROL = 0x06	//!< Request for remote control of the target object
} ObjectCommandType;


//! *************************** STRT
typedef struct
{
	HeaderType header;
	uint16_t StartTimeValueIdU16;
	uint16_t StartTimeContentLengthU16;
	uint32_t StartTimeU32;
	uint16_t GPSWeekValueID;
	uint16_t GPSWeekContentLength;
	uint16_t GPSWeek;
	FooterType footer;
} STRTType; //27 bytes

//! STRT value IDs
#define VALUE_ID_STRT_GPS_QMS_OF_WEEK 0x0002
#define VALUE_ID_STRT_GPS_WEEK 0x0003


//! *************************** HEAB
#define HEAB_FREQUENCY_HZ 100
typedef struct
{
  HeaderType header;
  uint16_t HEABStructValueID;
  uint16_t HEABStructContentLength;
  uint32_t GPSQmsOfWeek;
  uint8_t controlCenterStatus;
  FooterType footer;
} HEABType; //16 bytes

//! HEAB value IDs
#define VALUE_ID_HEAB_STRUCT 0x0090
typedef enum {
	CONTROL_CENTER_STATUS_INIT = 0x00,
	CONTROL_CENTER_STATUS_READY = 0x01,
	CONTROL_CENTER_STATUS_ABORT = 0x02,
	CONTROL_CENTER_STATUS_RUNNING = 0x03,
	CONTROL_CENTER_STATUS_TEST_DONE = 0x04,
	CONTROL_CENTER_STATUS_NORMAL_STOP = 0x05
} ControlCenterStatusType;

//! *************************** MONR
#define MONR_EXPECTED_FREQUENCY_HZ 100
typedef struct
{
	HeaderType header;
	uint16_t monrStructValueID;
	uint16_t monrStructContentLength;
	uint32_t gpsQmsOfWeek;
	int32_t xPosition;
	int32_t yPosition;
	int32_t zPosition;
	uint16_t heading;
	int16_t longitudinalSpeed;
	int16_t lateralSpeed;
	int16_t longitudinalAcc;
	int16_t lateralAcc;
	uint8_t driveDirection;
	uint8_t state;
	uint8_t readyToArm;
	uint8_t errorStatus;
	FooterType footer;
} MONRType;

//! MONR value IDs
#define VALUE_ID_MONR_STRUCT 0x80


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

typedef enum {
	MESSAGE_OK,
	MESSAGE_LENGTH_ERROR,
	MESSAGE_TYPE_ERROR,
	MESSAGE_CRC_ERROR,
	MESSAGE_VERSION_ERROR,
	MESSAGE_VALUE_ID_ERROR,
	MESSAGE_SYNC_WORD_ERROR
} ISOMessageReturnValue;

typedef enum {
	MESSAGE_ID_INVALID = 0x0000,
	MESSAGE_ID_TRAJ = 0x0001,
	MESSAGE_ID_OSEM = 0x0002,
	MESSAGE_ID_OSTM = 0x0003,
	MESSAGE_ID_STRT = 0x0004,
	MESSAGE_ID_HEAB = 0x0005,
	MESSAGE_ID_MONR = 0x0006,
	MESSAGE_ID_MONR2 = 0x0007,
	MESSAGE_ID_SOWM = 0x0008,
	MESSAGE_ID_INFO = 0x0009,
	MESSAGE_ID_RCMM = 0x000A,
	MESSAGE_ID_SYPM = 0x000B,
	MESSAGE_ID_MTSP = 0x000C,
	MESSAGE_ID_TRCM = 0x0011,
	MESSAGE_ID_ACCM = 0x0012,
	MESSAGE_ID_TREO = 0x0013,
	MESSAGE_ID_EXAC = 0x0014,
	MESSAGE_ID_CATA = 0x0015,
	MESSAGE_ID_RCCM = 0x0020,
	MESSAGE_ID_RCRT = 0x0021,
	MESSAGE_ID_PIME = 0x0030,
	MESSAGE_ID_COSE = 0x0031,
	MESSAGE_ID_MOMA = 0x0032,
	MESSAGE_ID_RESERVE_RANGE_1_LOWER_LIMIT = 0x0100,
	MESSAGE_ID_RESERVE_RANGE_1_UPPER_LIMIT = 0x0FFF,
	MESSAGE_ID_RESERVE_RANGE_2_LOWER_LIMIT = 0xF000,
	MESSAGE_ID_RESERVE_RANGE_2_UPPER_LIMIT = 0xFFFF,
	MESSAGE_ID_VENDOR_SPECIFIC_LOWER_LIMIT = 0xA100,
	MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT = 0xBFFF
} ISOMessageID;


ISOMessageReturnValue decodeMONRMessage(const char * MonrData, const size_t length, MONRType * MONRData, const char debug);
ssize_t encodeSTRTMessage(const struct timeval* timeOfStart, char * strtDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeOSEMMessage(const double * latitude_deg, const double * longitude_deg, const float * altitude_m, const float * maxPositionDeviation_m, const float * maxLateralDeviation_m, const float * minimumPositioningAccuracy_m, char * osemDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeOSTMMessage(const ObjectCommandType command, char * ostmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeHEABMessage(const ControlCenterStatusType status, char * heabDataBuffer, const size_t bufferLength, const char debug);
ISOMessageReturnValue MONRToASCII(const MONRType * MONRData, char * asciiBuffer, const size_t bufferLength, const char debug);
ISOMessageReturnValue ASCIIToMONR(const char * asciiBuffer, MONRType * MONRData, const char debug);
ISOMessageID getISOMessageType(const char * messageData, const size_t length, const char debug);


#ifdef __cplusplus
}
#endif
#endif
