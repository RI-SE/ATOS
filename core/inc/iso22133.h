#ifndef ISO22133_H
#define ISO22133_H
#ifdef __cplusplus
extern "C" {
#endif
/*! ------------------------------------------------------------------------------
 *  -- Copyright	: (C) AstaZero AB
 *  ------------------------------------------------------------------------------
 *  -- File			: iso22133.h
 *  -- Author		: Lukas Wikander
 *  -- Description	: This file specifies an interface for converting from network
 *					  messages in the ISO 22133 format to native data types.
 *  -- Purpose		: Reduce the amount of boilerplate needed to read an ISO 22133
 *					  message.
 *  -- Reference	: ISO/TC 22/SC 33/WG 16 - ISO/WD 22133-1
 *  ------------------------------------------------------------------------------
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include "positioning.h"

/*! OSTM commands */
typedef enum {
	OBJECT_COMMAND_ARM = 0x02,				//!< Request to arm the target object
	OBJECT_COMMAND_DISARM = 0x03,			//!< Request to disarm the target object
	OBJECT_COMMAND_REMOTE_CONTROL = 0x06	//!< Request for remote control of the target object
} ObjectCommandType;


#define HEAB_FREQUENCY_HZ 100
/*! HEAB control center statuses */
typedef enum {
	CONTROL_CENTER_STATUS_INIT = 0x00,			//!<
	CONTROL_CENTER_STATUS_READY = 0x01,			//!<
	CONTROL_CENTER_STATUS_ABORT = 0x02,			//!<
	CONTROL_CENTER_STATUS_RUNNING = 0x03,		//!<
	CONTROL_CENTER_STATUS_TEST_DONE = 0x04,		//!<
	CONTROL_CENTER_STATUS_NORMAL_STOP = 0x05	//!<
} ControlCenterStatusType;

#define MONR_EXPECTED_FREQUENCY_HZ 100

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


typedef enum {
	MESSAGE_OK,
	MESSAGE_LENGTH_ERROR,
	MESSAGE_TYPE_ERROR,
	MESSAGE_CRC_ERROR,
	MESSAGE_VERSION_ERROR,
	MESSAGE_VALUE_ID_ERROR,
	MESSAGE_SYNC_WORD_ERROR
} ISOMessageReturnValue;

/*! Valid ISO message identifiers */
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
	MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT = 0xBFFF,
	MESSAGE_ID_VENDOR_SPECIFIC_RISE_INSUP = 0xA102
} ISOMessageID;

/*! Remote control command */

typedef enum {
	MANOEUVRE_BACK_TO_START = 3
} RemoteControlManoeuvreType;

/*! Supervisor command */
typedef enum {
	SUPERVISOR_COMMAND_NORMAL = 1,	//!< Place supervisor in normal mode
	SUPERVISOR_COMMAND_DEBUG = 2	//!< Place supervisor in debug mode
} SupervisorCommandType;

ISOMessageReturnValue decodeMONRMessage(const char * monrDataBuffer, const size_t bufferLength, uint32_t * objectID, ObjectMonitorType * MonitorData, const char debug);
ssize_t encodeTRAJMessageHeader(const uint16_t trajectoryID, const uint16_t trajectoryVersion, const char * trajectoryName, const size_t nameLength, const uint32_t numberOfPointsInTraj, char * trajDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeTRAJMessagePoint(const struct timeval * pointTimeFromStart, const CartesianPosition position, const SpeedType speed, const AccelerationType acceleration, const float curvature, char * trajDataBufferPointer, const size_t remainingBufferLength, const char debug);
ssize_t encodeTRAJMessageFooter(char * trajDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeSTRTMessage(const struct timeval* timeOfStart, char * strtDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeOSEMMessage(const double_t * latitude_deg, const double_t * longitude_deg, const float * altitude_m, const float * maxPositionDeviation_m, const float * maxLateralDeviation_m, const float * minimumPositioningAccuracy_m, char * osemDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeOSTMMessage(const ObjectCommandType command, char * ostmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeHEABMessage(const ControlCenterStatusType status, char * heabDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeSYPMMessage(const struct timeval synchronizationTime, const struct timeval freezeTime, char * sypmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeMTSPMessage(const struct timeval * estSyncPointTime, char * mtspDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeTRCMMessage(const uint16_t* triggerID, const TriggerType_t* triggerType, const TriggerTypeParameter_t* param1, const TriggerTypeParameter_t* param2, const TriggerTypeParameter_t* param3, char * trcmDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeTREOMessage();
ssize_t encodeACCMMessage(const uint16_t* actionID, const ActionType_t* actionType, const ActionTypeParameter_t* param1, const ActionTypeParameter_t* param2, const ActionTypeParameter_t* param3, char * accmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeEXACMessage(const uint16_t* actionID, const struct timeval * executionTime, char * exacDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeINSUPMessage(const SupervisorCommandType, char * insupDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeRCMMMessage(const RemoteControlManoeuvreType command, char *rcmmDataBuffer, const size_t bufferLength, const char debug);
ISOMessageID getISOMessageType(const char * messageData, const size_t length, const char debug);
void setISOCRCVerification(const int8_t enabled);

#ifdef __cplusplus
}
#endif
#endif
