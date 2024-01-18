#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

// ************************* Type definitions according ISO protocol specification *******************************
//! Predefined integer values with special meaning
#define ISO_SYNC_WORD 0x7E7F
#define TRANSMITTER_ID_UNAVAILABLE_VALUE UINT32_MAX
#define CONTROL_CENTER_STATUS_UNAVAILABLE 255
#define LATITUDE_UNAVAILABLE_VALUE (-140737488355328)
#define LATITUDE_ONE_DEGREE_VALUE 10000000000.0
#define LONGITUDE_UNAVAILABLE_VALUE (-140737488355328)
#define LONGITUDE_ONE_DEGREE_VALUE 10000000000.0
#define ALTITUDE_UNAVAILABLE_VALUE (-2147483648)
#define ALTITUDE_ONE_METER_VALUE 100.0
#define ROTATION_ONE_DEGREE_VALUE 100.0
#define DATE_UNAVAILABLE_VALUE 4294967295
#define GPS_WEEK_UNAVAILABLE_VALUE 65535
#define GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE 4294967295
#define MAX_WAY_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_WAY_DEVIATION_ONE_METER_VALUE 1000.0
#define MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_LATERAL_DEVIATION_ONE_METER_VALUE 1000.0
#define MAX_YAW_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_YAW_DEVIATION_ONE_DEGREE_VALUE 100.0
#define MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE 65535
#define MIN_POSITIONING_ACCURACY_ONE_METER_VALUE 100.0
#define COMMUNICATION_TIMEOUT_ONE_SECOND_VALUE 100.0
#define MONR_RATE_ONE_HZ_VALUE 1.0
#define MONR2_RATE_ONE_HZ_VALUE 1.0
#define HEAB_RATE_ONE_HZ_VALUE 1.0
#define TRIGGER_ID_UNAVAILABLE 65535
#define TRIGGER_TYPE_UNAVAILABLE 65535
#define TRIGGER_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define TRIGGER_TIMESTAMP_UNAVAILABLE 4294967295
#define ACTION_ID_UNAVAILABLE 65535
#define ACTION_TYPE_UNAVAILABLE 65535
#define ACTION_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define ACTION_EXECUTE_TIME_UNAVAILABLE 4294967295
#define POSITION_ONE_METER_VALUE 1000.0
#define POSITION_UNAVAILABLE_VALUE (-2147483648)
#define LENGTH_ONE_METER_VALUE 1000.0
#define LENGTH_UNAVAILABLE_VALUE UINT32_MAX
#define MASS_ONE_KILOGRAM_VALUE 1000.0
#define MASS_UNAVAILABLE_VALUE UINT32_MAX
#define STEERING_UNAVAILABLE_VALUE (-32768)
#define YAW_UNAVAILABLE_VALUE 65535
#define YAW_ONE_DEGREE_VALUE 100.0
#define PITCH_UNAVAILABLE_VALUE -32768
#define PITCH_ONE_DEGREE_VALUE 100.0
#define ROLL_UNAVAILABLE_VALUE -32768
#define ROLL_ONE_DEGREE_VALUE 100.0
#define SPEED_UNAVAILABLE_VALUE (-32768)
#define SPEED_ONE_METER_PER_SECOND_VALUE 100.0
#define ACCELERATION_UNAVAILABLE_VALUE (-32768)
#define ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE 1000.0
#define RELATIVE_TIME_ONE_SECOND_VALUE 1000.0
#define STEERING_ANGLE_ONE_DEGREE_VALUE 100.0
#define STEERING_ANGLE_MAX_VALUE_DEG 18000
#define STEERING_ANGLE_MIN_VALUE_DEG (-18000)
#define STEERING_ANGLE_MAX_VALUE_RAD M_PI
#define STEERING_ANGLE_MIN_VALUE_RAD (-M_PI)
#define STEERING_ANGLE_UNAVAILABLE_VALUE 65535
#define ESTIMATED_SYNC_POINT_TIME_UNAVAILABLE_VALUE 4294967295
#define MAX_VALUE_PERCENTAGE 100
#define MIN_VALUE_PERCENTAGE (-100)



#define DEFAULT_CRC_INIT_VALUE 0x0000
#define DEFAULT_CRC_CHECK_ENABLED 1

enum DriveDirectionValues {
	ISO_DRIVE_DIRECTION_FORWARD = 0,
	ISO_DRIVE_DIRECTION_BACKWARD = 1,
	ISO_DRIVE_DIRECTION_UNAVAILABLE = 255
};
enum ObjectStateValues {
	ISO_OBJECT_STATE_OFF = 0,
	ISO_OBJECT_STATE_INIT = 1,
	ISO_OBJECT_STATE_ARMED = 2,
	ISO_OBJECT_STATE_DISARMED = 3,
	ISO_OBJECT_STATE_RUNNING = 4,
	ISO_OBJECT_STATE_POSTRUN = 5,
	ISO_OBJECT_STATE_REMOTE_CONTROLLED = 6,
	ISO_OBJECT_STATE_ABORTING = 7,
	ISO_OBJECT_STATE_PRE_ARMING = 8,
	ISO_OBJECT_STATE_PRE_RUNNING = 9,
	ISO_OBJECT_STATE_UNAVALIABLE = 255
};
enum ArmReadinessValues {
	ISO_NOT_READY_TO_ARM = 0,
	ISO_READY_TO_ARM = 1,
	ISO_READY_TO_ARM_UNAVAILABLE = 255
};

#define BITMASK_ERROR_ABORT_REQUEST				0x80
#define BITMASK_ERROR_OUTSIDE_GEOFENCE			0x40
#define BITMASK_ERROR_BAD_POSITIONING_ACCURACY	0x20
#define BITMASK_ERROR_ENGINE_FAULT				0x10
#define BITMASK_ERROR_BATTERY_FAULT				0x08
#define BITMASK_ERROR_OTHER						0x04
#define BITMASK_ERROR_SYNC_POINT_ENDED			0x02
#define BITMASK_ERROR_VENDOR_SPECIFIC			0x01


// ************************* Global ISO protocol settings ********************************************************
static const uint8_t SupportedProtocolVersions[] = { 2 };

#define ISO_PROTOCOL_VERSION 2	//!< ISO protocol version of messages sent
#define ACK_REQ 0

// Temporary fix since HAVE_BYTESWAP_H does not appear to work for linux
// TODO: remove once several systems start using this library
// Windows mostly runs on little endian architectures
// TODO: Future-proof - make below endian conversion host independent on Windows
#if defined _WIN32 || defined __APPLE__
#	define htole16(value) (value)
#	define htole32(value) (value)
#	define htole64(value) (value)

#	define le16toh(value) (value)
#	define le32toh(value) (value)

#	if defined __GNUC__
#		define htobe16(value) __builtin_bswap16(value)

#	elif defined (_MSC_VER)
#		include <stdlib.h>
#		define htobe16(x) _byteswap_ushort(x)
#   endif

#elif __linux__
#	include <byteswap.h>
#	include <endian.h>
#endif

// ************************* Non-ISO type definitions and defines ************************************************
// Byte swapper definitions for 6 byte values and floats
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define le48toh(x) (x)
#define htole48(x) (x)
#define htolef(x) (x)
#else
#define le48toh(x) (le64toh(x) >> 16)
#define htole48(x) (htole64(x) >> 16)
#define htolef_a(x) \
	htole32((union { uint32_t i; float f; }){ .f = (x) }.i)
#define htolef(x) \
  ((union { uint32_t i; float f; }){ .i = htolef_a(x) }.f)
#endif

// Time constants
// Leap seconds between UTC and GPS
#define MS_LEAP_SEC_DIFF_UTC_GPS (18000)
// Length of a week
#define WEEK_TIME_QMS 2419200000
#define WEEK_TIME_MS 604800000
// Time between 1970-01-01 and 1980-01-06
#define MS_TIME_DIFF_UTC_GPS ((uint64_t)(315964800000))

#ifdef __cplusplus
}
#endif