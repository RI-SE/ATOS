#include "iso22133.h"
#include "logging.h"
#include "maestroTime.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <endian.h>
#include <byteswap.h>
#include <math.h>

// ************************* Global ISO protocol settings ********************************************************
static const uint8_t SupportedProtocolVersions[] = { 2 };

#define ISO_PROTOCOL_VERSION 2	//!< ISO protocol version of messages sent
#define ACK_REQ 0


// ************************* Type definitions according ISO protocol specification *******************************
//! Predefined integer values with special meaning
#define ISO_SYNC_WORD 0x7E7E
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
#define MIN_POSITIONING_ACCURACY_ONE_METER_VALUE 1000	// ISO specification unclear on this value
#define TRIGGER_ID_UNAVAILABLE 65535
#define TRIGGER_TYPE_UNAVAILABLE 65535
#define TRIGGER_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define ACTION_ID_UNAVAILABLE 65535
#define ACTION_TYPE_UNAVAILABLE 65535
#define ACTION_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define POSITION_ONE_METER_VALUE 1000
#define HEADING_UNAVAILABLE_VALUE 36001
#define HEADING_ONE_DEGREE_VALUE 100
#define SPEED_UNAVAILABLE_VALUE (-32768)
#define SPEED_ONE_METER_PER_SECOND_VALUE 100
#define ACCELERATION_UNAVAILABLE_VALUE 32001
#define ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE 1000
#define RELATIVE_TIME_ONE_SECOND_VALUE 1000

#define DEFAULT_CRC_INIT_VALUE 0x0000
#define DEFAULT_CRC_CHECK_ENABLED 1

typedef enum {
	ISO_DRIVE_DIRECTION_FORWARD = 0,
	ISO_DRIVE_DIRECTION_BACKWARD = 1,
	ISO_DRIVE_DIRECTION_UNAVAILABLE = 2
} DriveDirectionValues;
typedef enum {
	ISO_OBJECT_STATE_OFF = 0,
	ISO_OBJECT_STATE_INIT = 1,
	ISO_OBJECT_STATE_ARMED = 2,
	ISO_OBJECT_STATE_DISARMED = 3,
	ISO_OBJECT_STATE_RUNNING = 4,
	ISO_OBJECT_STATE_POSTRUN = 5,
	ISO_OBJECT_STATE_REMOTE_CONTROLLED = 6,
	ISO_OBJECT_STATE_ABORTING = 7
} ObjectStateValues;
typedef enum {
	ISO_NOT_READY_TO_ARM = 0,
	ISO_READY_TO_ARM = 1,
	ISO_READY_TO_ARM_UNAVAILABLE = 2
} ArmReadinessValues;

#define BITMASK_ERROR_ABORT_REQUEST				0x80
#define BITMASK_ERROR_OUTSIDE_GEOFENCE			0x40
#define BITMASK_ERROR_BAD_POSITIONING_ACCURACY	0x20
#define BITMASK_ERROR_ENGINE_FAULT				0x10
#define BITMASK_ERROR_BATTERY_FAULT				0x08
#define BITMASK_ERROR_OTHER						0x04
#define BITMASK_ERROR_SYNC_POINT_ENDED			0x02
#define BITMASK_ERROR_VENDOR_SPECIFIC			0x01



#pragma pack(push,1)			// Ensure sizeof() is useable for (most) network byte lengths
/*! ISO message header */
typedef struct {
	uint16_t SyncWordU16;
	uint8_t TransmitterIdU8;
	uint8_t MessageCounterU8;
	uint8_t AckReqProtVerU8;
	uint16_t MessageIdU16;
	uint32_t MessageLengthU32;
} HeaderType;


/*! ISO message footer */
typedef struct {
	uint16_t Crc;
} FooterType;


/*! TRAJ message */
#define TRAJ_NAME_STRING_MAX_LENGTH 64
typedef struct {
	HeaderType header;
	uint16_t trajectoryIDValueID;
	uint16_t trajectoryIDContentLength;
	uint16_t trajectoryID;
	uint16_t trajectoryNameValueID;
	uint16_t trajectoryNameContentLength;
	char trajectoryName[TRAJ_NAME_STRING_MAX_LENGTH];
	uint16_t trajectoryVersionValueID;
	uint16_t trajectoryVersionContentLength;
	uint16_t trajectoryVersion;
} TRAJHeaderType;

typedef struct {
	uint16_t relativeTimeValueID;
	uint16_t relativeTimeContentLength;
	uint32_t relativeTime;
	uint16_t xPositionValueID;
	uint16_t xPositionContentLength;
	int32_t xPosition;
	uint16_t yPositionValueID;
	uint16_t yPositionContentLength;
	int32_t yPosition;
	uint16_t zPositionValueID;
	uint16_t zPositionContentLength;
	int32_t zPosition;
	uint16_t headingValueID;
	uint16_t headingContentLength;
	uint16_t heading;
	uint16_t longitudinalSpeedValueID;
	uint16_t longitudinalSpeedContentLength;
	int16_t longitudinalSpeed;
	uint16_t lateralSpeedValueID;
	uint16_t lateralSpeedContentLength;
	int16_t lateralSpeed;
	uint16_t longitudinalAccelerationValueID;
	uint16_t longitudinalAccelerationContentLength;
	int16_t longitudinalAcceleration;
	uint16_t lateralAccelerationValueID;
	uint16_t lateralAccelerationContentLength;
	int16_t lateralAcceleration;
	uint16_t curvatureValueID;
	uint16_t curvatureContentLength;
	float_t curvature;
} TRAJPointType;

typedef struct {
	FooterType footer;
} TRAJFooterType;

//! TRAJ value IDs
#define VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER 0x0101
#define VALUE_ID_TRAJ_TRAJECTORY_NAME 0x0102
#define VALUE_ID_TRAJ_TRAJECTORY_VERSION 0x0103
#define VALUE_ID_TRAJ_RELATIVE_TIME 0x0001
#define VALUE_ID_TRAJ_X_POSITION 0x0010
#define VALUE_ID_TRAJ_Y_POSITION 0x0011
#define VALUE_ID_TRAJ_Z_POSITION 0x0012
#define VALUE_ID_TRAJ_HEADING 0x0030
#define VALUE_ID_TRAJ_LONGITUDINAL_SPEED 0x0040
#define VALUE_ID_TRAJ_LATERAL_SPEED 0x0041
#define VALUE_ID_TRAJ_LONGITUDINAL_ACCELERATION 0x0050
#define VALUE_ID_TRAJ_LATERAL_ACCELERATION 0x0051
#define VALUE_ID_TRAJ_CURVATURE 0x0052


/*! OSEM message */
typedef struct {
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
} OSEMType;						//85 bytes

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


/*! OSTM message */
typedef struct {
	HeaderType header;
	uint16_t stateValueID;
	uint16_t stateContentLength;
	uint8_t state;
	FooterType footer;
} OSTMType;						//16 bytes

//! OSTM value IDs
#define VALUE_ID_OSTM_STATE_CHANGE_REQUEST 0x0064


/*! STRT message */
typedef struct {
	HeaderType header;
	uint16_t StartTimeValueIdU16;
	uint16_t StartTimeContentLengthU16;
	uint32_t StartTimeU32;
	uint16_t GPSWeekValueID;
	uint16_t GPSWeekContentLength;
	uint16_t GPSWeek;
	FooterType footer;
} STRTType;						//27 bytes

//! STRT value IDs
#define VALUE_ID_STRT_GPS_QMS_OF_WEEK 0x0002
#define VALUE_ID_STRT_GPS_WEEK 0x0003


//! MONR message */
typedef struct {
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


/*! HEAB message */
typedef struct {
	HeaderType header;
	uint16_t HEABStructValueID;
	uint16_t HEABStructContentLength;
	uint32_t GPSQmsOfWeek;
	uint8_t controlCenterStatus;
	FooterType footer;
} HEABType;						//16 bytes

//! HEAB value IDs
#define VALUE_ID_HEAB_STRUCT 0x0090


/*! SYPM message */
typedef struct {
	HeaderType header;
	uint16_t syncPointTimeValueID;
	uint16_t syncPointTimeContentLength;
	uint32_t syncPointTime;
	uint16_t freezeTimeValueID;
	uint16_t freezeTimeContentLength;
	uint32_t freezeTime;
	FooterType footer;
} SYPMType;

//! SYPM value IDs
#define VALUE_ID_SYPM_SYNC_POINT_TIME 0x0001
#define VALUE_ID_SYPM_FREEZE_TIME 0x0002


/*! MTSP message */
typedef struct {
	HeaderType header;
	uint16_t estSyncPointTimeValueID;
	uint16_t estSyncPointTimeContentLength;
	uint32_t estSyncPointTime;
	FooterType footer;
} MTSPType;

//! MTSP value IDs
#define VALUE_ID_MTSP_EST_SYNC_POINT_TIME 0x0001


/*! TRCM message */
typedef struct {
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

//! TRCM value IDs
#define VALUE_ID_TRCM_TRIGGER_ID 0x0001
#define VALUE_ID_TRCM_TRIGGER_TYPE 0x0002
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1 0x0011
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2 0x0012
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3 0x0013


/*! TREO message */
typedef struct {
	HeaderType header;
	uint16_t triggerIDValueID;
	uint16_t triggerIDContentLength;
	uint16_t triggerID;
	uint16_t timestamp_qmsowValueID;
	uint16_t timestamp_qmsowContentLength;
	uint32_t timestamp_qmsow;
	FooterType footer;
} TREOType;

//! TREO value IDs
#define VALUE_ID_TREO_TRIGGER_ID 0x0001
#define VALUE_ID_TREO_TRIGGER_TIMESTAMP 0x0002


/*! ACCM message */
typedef struct {
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

//! ACCM value IDs
#define VALUE_ID_ACCM_ACTION_ID 0x0002
#define VALUE_ID_ACCM_ACTION_TYPE 0x0003
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM1 0x00A1
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM2 0x00A2
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM3 0x00A3


/*! EXAC message */
typedef struct {
	HeaderType header;
	uint16_t actionIDValueID;
	uint16_t actionIDContentLength;
	uint16_t actionID;
	uint16_t executionTime_qmsoWValueID;
	uint16_t executionTime_qmsoWContentLength;
	uint32_t executionTime_qmsoW;
	FooterType footer;
} EXACType;

//! EXAC value IDs
#define VALUE_ID_EXAC_ACTION_ID 0x0002
#define VALUE_ID_EXAC_ACTION_EXECUTE_TIME 0x0003

/*! CATA message */
// TODO
//! CATA value IDs
// TODO


/*! INSUP message */
typedef struct {
	HeaderType header;
	uint16_t modeValueID;
	uint16_t modeContentLength;
	uint8_t mode;
	FooterType footer;
} INSUPType;

//! INSUP value IDs
#define VALUE_ID_INSUP_MODE 0x0200


/*! RCMM message */
typedef struct {
	HeaderType header;
	uint16_t RCMMControlValueID;
	uint16_t RCMMControlContentLength;
	uint8_t rcmmControlU8;
	FooterType footer;
} RCMMType;						//18 bytes

//! HEAB value IDs
#define VALUE_ID_RCMM_CONTROL 0x0201



#pragma pack(pop)


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

// ************************** static function declarations ********************************************************
static ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);
static ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length,
											 FooterType * HeaderData, const char debug);
static HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug);
static FooterType buildISOFooter(const void *message, const size_t sizeExclFooter, const char debug);
static char isValidMessageID(const uint16_t id);
static double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad);
static double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad);
static void convertMONRToHostRepresentation(const MONRType * MONRData, ObjectMonitorType * monitorData);
static ISOMessageReturnValue verifyChecksum(const void *data, const size_t dataLen, const uint16_t crc,
											const char debug);
static uint16_t crcByte(const uint16_t crc, const uint8_t byte);
static uint16_t crc16(const uint8_t * data, size_t dataLen);

// ************************** static variables ********************************************************************
static uint16_t trajectoryMessageCrc = 0;
static int8_t isCRCVerificationEnabled = DEFAULT_CRC_CHECK_ENABLED;

// ************************** function definitions ****************************************************************

/*!
 * \brief decodeISOHeader Convert data in a buffer to an ISO heade
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length, HeaderType * HeaderData,
									  const char debug) {

	const char *p = MessageBuffer;
	ISOMessageReturnValue retval = MESSAGE_OK;
	const char ProtocolVersionBitmask = 0x7F;
	char messageProtocolVersion = 0;
	char isProtocolVersionSupported = 0;

	// If not enough data to fill header, generate error
	if (length < sizeof (HeaderData)) {
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO header");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode ISO header
	memcpy(&HeaderData->SyncWordU16, p, sizeof (HeaderData->SyncWordU16));
	HeaderData->SyncWordU16 = le16toh(HeaderData->SyncWordU16);
	p += sizeof (HeaderData->SyncWordU16);

	// If sync word is not correct, generate error
	if (HeaderData->SyncWordU16 != ISO_SYNC_WORD) {
		LogMessage(LOG_LEVEL_ERROR, "Sync word error when decoding ISO header");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_SYNC_WORD_ERROR;
	}

	memcpy(&HeaderData->TransmitterIdU8, p, sizeof (HeaderData->TransmitterIdU8));
	p += sizeof (HeaderData->TransmitterIdU8);

	memcpy(&HeaderData->MessageCounterU8, p, sizeof (HeaderData->MessageCounterU8));
	p += sizeof (HeaderData->MessageCounterU8);

	memcpy(&HeaderData->AckReqProtVerU8, p, sizeof (HeaderData->AckReqProtVerU8));
	p += sizeof (HeaderData->AckReqProtVerU8);

	// Loop over permitted protocol versions to see if current version is among them
	messageProtocolVersion = HeaderData->AckReqProtVerU8 & ProtocolVersionBitmask;
	for (size_t i = 0; i < sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]); ++i) {
		if (SupportedProtocolVersions[i] == messageProtocolVersion) {
			isProtocolVersionSupported = 1;
			break;
		}
	}

	// Generate error if protocol version not supported
	if (!isProtocolVersionSupported) {
		LogMessage(LOG_LEVEL_WARNING, "Protocol version %u not supported", messageProtocolVersion);
		retval = MESSAGE_VERSION_ERROR;
		memset(HeaderData, 0, sizeof (*HeaderData));
		return retval;
	}

	memcpy(&HeaderData->MessageIdU16, p, sizeof (HeaderData->MessageIdU16));
	p += sizeof (HeaderData->MessageIdU16);
	HeaderData->MessageIdU16 = le16toh(HeaderData->MessageIdU16);

	memcpy(&HeaderData->MessageLengthU32, p, sizeof (HeaderData->MessageLengthU32));
	p += sizeof (HeaderData->MessageLengthU32);
	HeaderData->MessageLengthU32 = le32toh(HeaderData->MessageLengthU32);

	if (debug) {
		LogPrint("SyncWordU16 = 0x%x", HeaderData->SyncWordU16);
		LogPrint("TransmitterIdU8 = 0x%x", HeaderData->TransmitterIdU8);
		LogPrint("MessageCounterU8 = 0x%x", HeaderData->MessageCounterU8);
		LogPrint("AckReqProtVerU8 = 0x%x", HeaderData->AckReqProtVerU8);
		LogPrint("MessageIdU16 = 0x%x", HeaderData->MessageIdU16);
		LogPrint("MessageLengthU32 = 0x%x", HeaderData->MessageLengthU32);
	}

	return retval;
}

/*!
 * \brief decodeISOFooter Convert data in a buffer to an ISO footer
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length, FooterType * FooterData,
									  const char debug) {

	// If too little data, generate error
	if (length < sizeof (FooterData->Crc)) {
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO footer");
		memset(FooterData, 0, sizeof (*FooterData));
		return MESSAGE_LENGTH_ERROR;
	}
	memcpy(&FooterData->Crc, MessageBuffer, sizeof (FooterData->Crc));
	FooterData->Crc = le16toh(FooterData->Crc);

	if (debug) {
		LogPrint("Decoded ISO footer:\n\tCRC: 0x%x", FooterData->Crc);
	}

	return MESSAGE_OK;
}

/*!
 * \brief buildISOHeader Constructs an ISO header based on the supplied message ID and content length
 * \param id Message ID of the message for which the header is to be used
 * \param messageLength Length of the message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO header data
 */
HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug) {
	HeaderType header;

	header.SyncWordU16 = ISO_SYNC_WORD;
	header.TransmitterIdU8 = 0;
	header.MessageCounterU8 = 0;
	header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	if (messageLength >= sizeof (HeaderType) + sizeof (FooterType)) {
		header.MessageIdU16 = (uint16_t) id;
		header.MessageLengthU32 = messageLength - sizeof (HeaderType) - sizeof (FooterType);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Supplied message length too small to hold header and footer");
		header.MessageIdU16 = (uint16_t) MESSAGE_ID_INVALID;
		header.MessageLengthU32 = 0;
	}

	if (debug) {
		LogPrint("Encoded ISO header:\n\tSync word: 0x%x\n\tTransmitter ID: %u\n\tMessage counter: %u\n\t"
				 "Ack request | Protocol version: 0x%x\n\tMessage ID: 0x%x\n\tMessage length: %u",
				 header.SyncWordU16, header.TransmitterIdU8, header.MessageCounterU8, header.AckReqProtVerU8,
				 header.MessageIdU16, header.MessageLengthU32);
	}

	// Convert from host endianness to little endian
	header.SyncWordU16 = htole16(header.SyncWordU16);
	header.MessageIdU16 = htole16(header.MessageIdU16);
	header.MessageLengthU32 = htole32(header.MessageLengthU32);

	return header;
}

/*!
 * \brief buildISOFooter Constructs a footer for an ISO message
 * \param message Pointer to start of message header
 * \param messageSize Size of the entire message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO footer data
 */
FooterType buildISOFooter(const void *message, const size_t messageSize, const char debug) {
	FooterType footer;

	// Calculate CRC - remembering that message begins with header and messageSize will include header and footer
	footer.Crc = crc16(message, messageSize - sizeof (FooterType));

	if (debug) {
		LogPrint("Encoded ISO footer:\n\tCRC: 0x%x", footer.Crc);
	}

	footer.Crc = htole16(footer.Crc);

	return footer;
}

/*!
 * \brief isValidMessageID Determines if specified id is a valid ISO message ID. The reserved range is deemed
 * invalid and vendor specific range is deemed valid.
 * \param id An ISO message id to be checked
 * \return 1 if valid, 0 if not
 */
char isValidMessageID(const uint16_t id) {
	return id == MESSAGE_ID_MONR || id == MESSAGE_ID_HEAB || id == MESSAGE_ID_TRAJ || id == MESSAGE_ID_OSEM
		|| id == MESSAGE_ID_OSTM || id == MESSAGE_ID_STRT || id == MESSAGE_ID_MONR2 || id == MESSAGE_ID_SOWM
		|| id == MESSAGE_ID_INFO || id == MESSAGE_ID_RCMM || id == MESSAGE_ID_SYPM || id == MESSAGE_ID_MTSP
		|| id == MESSAGE_ID_TRCM || id == MESSAGE_ID_ACCM || id == MESSAGE_ID_TREO || id == MESSAGE_ID_EXAC
		|| id == MESSAGE_ID_CATA || id == MESSAGE_ID_RCMM || id == MESSAGE_ID_RCRT || id == MESSAGE_ID_PIME
		|| id == MESSAGE_ID_COSE || id == MESSAGE_ID_MOMA
		|| (id >= MESSAGE_ID_VENDOR_SPECIFIC_LOWER_LIMIT && id <= MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT);
}

/*!
 * \brief getISOMessageType Determines the ISO message type of a raw data buffer
 * \param messageData Buffer containing raw data to be parsed into an ISO message
 * \param length Size of buffer to be parsed
 * \param debug Flag for enabling debugging information
 * \return Value according to ::ISOMessageID
 */
ISOMessageID getISOMessageType(const char *messageData, const size_t length, const char debug) {
	HeaderType header;

	// Decode header
	if (decodeISOHeader(messageData, length, &header, debug) != MESSAGE_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to parse raw data into ISO message header");
		return MESSAGE_ID_INVALID;
	}

	// Check if header contains valid message ID, if so return it
	if (isValidMessageID(header.MessageIdU16))
		return (ISOMessageID) header.MessageIdU16;
	else {
		LogMessage(LOG_LEVEL_WARNING, "Message ID %u does not match any known ISO message",
				   header.MessageIdU16);
		return MESSAGE_ID_INVALID;
	}
}


/*!
 * \brief crcByte Updates the given CRC based on an input byte from data
 * \param crc CRC from previous byte
 * \param byte New data byte
 * \return New CRC value
 */
uint16_t crcByte(const uint16_t crc, const uint8_t byte) {
	static const uint16_t crcTable[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
	};

	return (uint16_t) ((crc << 8) ^ crcTable[(crc >> 8) ^ byte]);
}


/*!
 * \brief crc16 Calculates the 16 bit CCITT checksum value for the polynomial
 *				x^16 + x^12 + x^5 + 1
 * \param data Block of data for which CRC is to be calculated
 * \param dataLen Length of the block of data
 * \return CRC checksum
 */
uint16_t crc16(const uint8_t * data, size_t dataLen) {
	uint16_t crc = DEFAULT_CRC_INIT_VALUE;

	while (dataLen-- > 0) {
		crc = crcByte(crc, *data++);
	}
	return crc;
}


/*!
 * \brief verifyChecksum Generates a checksum for specified data and checks if it matches against
 *			the specified CRC. If the specified CRC is 0, the message does not contain a CRC value
 *			and the message is assumed uncorrupted.
 * \param data Data for which checksum is to be verified
 * \param dataLen Length of the data
 * \param CRC Received CRC value for the data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue verifyChecksum(const void *data, const size_t dataLen, const uint16_t CRC,
									 const char debug) {
	if (!isCRCVerificationEnabled || CRC == 0) {
		return MESSAGE_OK;
	}

	const uint16_t dataCRC = crc16(data, dataLen);

	if (debug) {
		LogPrint("CRC given: %u, CRC calculated: %u", CRC, dataCRC);
	}
	return dataCRC == CRC ? MESSAGE_OK : MESSAGE_CRC_ERROR;
}

/*!
 * \brief setISOCRCVerification Enables or disables checksum verification on received messages (default
 *			is to enable checksum verification)
 * \param enabled Boolean for enabling or disabling the checksum verification
 */
void setISOCRCVerification(const int8_t enabled) {
	isCRCVerificationEnabled = enabled ? 1 : 0;
	return;
}


/*!
 * \brief encodeTRAJMessageHeader Creates a TRAJ message header based on supplied values and resets
 *	an internal CRC to be used in the corresponding footer. The header is printed to a buffer.
 * \param trajectoryID ID of the trajectory
 * \param trajectoryVersion Version of the trajectory
 * \param trajectoryName A string of maximum length 63 excluding the null terminator
 * \param nameLength Length of the name string excluding the null terminator
 * \param numberOfPointsInTraj Number of points in the subsequent trajectory
 * \param trajDataBuffer Buffer to which TRAJ header is to be printed
 * \param bufferLength Length of buffer to which TRAJ header is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold header
 *		EMSGSIZE	if trajectory name is too long
 */
ssize_t encodeTRAJMessageHeader(const uint16_t trajectoryID, const uint16_t trajectoryVersion,
								const char *trajectoryName, const size_t nameLength,
								const uint32_t numberOfPointsInTraj, char *trajDataBuffer,
								const size_t bufferLength, const char debug) {

	TRAJHeaderType TRAJData;
	size_t dataLen;

	memset(trajDataBuffer, 0, bufferLength);

	// Error guarding
	if (trajectoryName == NULL && nameLength > 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Trajectory name length and pointer mismatch");
		return -1;
	}
	else if (trajDataBuffer == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Trajectory data buffer invalid");
		return -1;
	}
	else if (bufferLength < sizeof (TRAJHeaderType)) {
		errno = ENOBUFS;
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary TRAJ header data");
		return -1;
	}
	else if (nameLength >= sizeof (TRAJData.trajectoryName)) {
		errno = EMSGSIZE;
		LogMessage(LOG_LEVEL_ERROR, "Trajectory name <%s> too long for TRAJ message", trajectoryName);
		return -1;
	}

	// Construct ISO header
	TRAJData.header = buildISOHeader(MESSAGE_ID_TRAJ, sizeof (TRAJHeaderType)
									 + numberOfPointsInTraj * sizeof (TRAJPointType) +
									 sizeof (TRAJFooterType), debug);

	// Fill contents
	TRAJData.trajectoryIDValueID = VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER;
	TRAJData.trajectoryIDContentLength = sizeof (TRAJData.trajectoryID);
	TRAJData.trajectoryID = trajectoryID;

	TRAJData.trajectoryVersionValueID = VALUE_ID_TRAJ_TRAJECTORY_VERSION;
	TRAJData.trajectoryVersionContentLength = sizeof (TRAJData.trajectoryVersion);
	TRAJData.trajectoryVersion = trajectoryVersion;

	TRAJData.trajectoryNameValueID = VALUE_ID_TRAJ_TRAJECTORY_NAME;
	TRAJData.trajectoryNameContentLength = sizeof (TRAJData.trajectoryName);
	memset(TRAJData.trajectoryName, 0, sizeof (TRAJData.trajectoryName));
	if (trajectoryName != NULL) {
		memcpy(&TRAJData.trajectoryName, trajectoryName, nameLength);
	}

	if (debug) {
		LogPrint("TRAJ message header:\n\t"
				 "Trajectory ID value ID: 0x%x\n\t"
				 "Trajectory ID content length: %u\n\t"
				 "Trajectory ID: %u\n\t"
				 "Trajectory name value ID: 0x%x\n\t"
				 "Trajectory name content length: %u\n\t"
				 "Trajectory name: %s\n\t"
				 "Trajectory version value ID: 0x%x\n\t"
				 "Trajectory version content length: %u\n\t"
				 "Trajectory version: %u", TRAJData.trajectoryIDValueID,
				 TRAJData.trajectoryIDContentLength, TRAJData.trajectoryID,
				 TRAJData.trajectoryNameValueID, TRAJData.trajectoryNameContentLength,
				 TRAJData.trajectoryName, TRAJData.trajectoryVersionValueID,
				 TRAJData.trajectoryVersionContentLength, TRAJData.trajectoryVersion);
	}

	// Switch endianness to little endian for all fields
	TRAJData.trajectoryIDValueID = htole16(TRAJData.trajectoryIDValueID);
	TRAJData.trajectoryIDContentLength = htole16(TRAJData.trajectoryIDContentLength);
	TRAJData.trajectoryID = htole16(TRAJData.trajectoryID);
	TRAJData.trajectoryVersionValueID = htole16(TRAJData.trajectoryVersionValueID);
	TRAJData.trajectoryVersionContentLength = htole16(TRAJData.trajectoryVersionContentLength);
	TRAJData.trajectoryVersion = htole16(TRAJData.trajectoryVersion);
	TRAJData.trajectoryNameValueID = htole16(TRAJData.trajectoryNameValueID);
	TRAJData.trajectoryNameContentLength = htole16(TRAJData.trajectoryNameContentLength);

	// Reset CRC
	trajectoryMessageCrc = DEFAULT_CRC_INIT_VALUE;

	memcpy(trajDataBuffer, &TRAJData, sizeof (TRAJData));

	// Update CRC
	dataLen = sizeof (TRAJData);
	while (dataLen-- > 0) {
		trajectoryMessageCrc = crcByte(trajectoryMessageCrc, (uint8_t) (*trajDataBuffer++));
	}
	return sizeof (TRAJHeaderType);
}


/*!
 * \brief encodeTRAJMessagePoint Creates a TRAJ message point based on supplied values and updates an internal
 * CRC to be used in the footer. Also prints the TRAJ point to a buffer.
 * \param pointTimeFromStart Time from start of the trajectory point
 * \param position Position of the point
 * \param speed Speed at the point
 * \param acceleration Acceleration at the point
 * \param curvature Curvature of the trajectory at the point
 * \param trajDataBufferPointer Buffer to which the message is to be printed
 * \param remainingBufferLength Remaining bytes in the buffer to which the message is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold point
 */
ssize_t encodeTRAJMessagePoint(const struct timeval *pointTimeFromStart, const CartesianPosition position,
							   const SpeedType speed, const AccelerationType acceleration,
							   const float curvature, char *trajDataBufferPointer,
							   const size_t remainingBufferLength, const char debug) {

	TRAJPointType TRAJData;
	size_t dataLen;

	if (remainingBufferLength < sizeof (TRAJPointType)) {
		errno = ENOBUFS;
		LogMessage(LOG_LEVEL_DEBUG, "Buffer too small to hold necessary TRAJ point data");
		return -1;
	}
	else if (trajDataBufferPointer == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Trajectory data buffer invalid");
		return -1;
	}

	// Fill contents
	TRAJData.relativeTimeValueID = VALUE_ID_TRAJ_RELATIVE_TIME;
	TRAJData.relativeTimeContentLength = sizeof (TRAJData.relativeTime);
	TRAJData.relativeTime =
		(uint32_t) TimeGetAsUTCms(pointTimeFromStart) / (1000 / RELATIVE_TIME_ONE_SECOND_VALUE);

	TRAJData.xPositionValueID = VALUE_ID_TRAJ_X_POSITION;
	TRAJData.xPositionContentLength = sizeof (TRAJData.xPosition);
	TRAJData.yPositionValueID = VALUE_ID_TRAJ_Y_POSITION;
	TRAJData.yPositionContentLength = sizeof (TRAJData.yPosition);
	TRAJData.zPositionValueID = VALUE_ID_TRAJ_Z_POSITION;
	TRAJData.zPositionContentLength = sizeof (TRAJData.zPosition);
	if (position.isPositionValid) {
		TRAJData.xPosition = (int32_t) (position.xCoord_m * POSITION_ONE_METER_VALUE);
		TRAJData.yPosition = (int32_t) (position.yCoord_m * POSITION_ONE_METER_VALUE);
		TRAJData.zPosition = (int32_t) (position.zCoord_m * POSITION_ONE_METER_VALUE);
	}
	else {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Position is a required field in TRAJ messages");
		return -1;
	}

	TRAJData.headingValueID = VALUE_ID_TRAJ_HEADING;
	TRAJData.headingContentLength = sizeof (TRAJData.heading);
	if (position.isHeadingValid) {
		TRAJData.heading = (uint16_t) (mapHostHeadingToISOHeading(position.heading_rad)
									   * 180.0 / M_PI * HEADING_ONE_DEGREE_VALUE);
	}
	else {
		TRAJData.heading = HEADING_UNAVAILABLE_VALUE;
	}

	TRAJData.longitudinalSpeedValueID = VALUE_ID_TRAJ_LONGITUDINAL_SPEED;
	TRAJData.longitudinalSpeedContentLength = sizeof (TRAJData.longitudinalSpeed);
	TRAJData.lateralSpeedValueID = VALUE_ID_TRAJ_LATERAL_SPEED;
	TRAJData.lateralSpeedContentLength = sizeof (TRAJData.lateralSpeed);
	if (speed.isLongitudinalValid) {
		TRAJData.longitudinalSpeed = (int16_t) (speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE);
	}
	else {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Longitudinal speed is a required field in TRAJ messages");
		return -1;
	}
	TRAJData.lateralSpeed =
		speed.isLateralValid ? (int16_t) (speed.lateral_m_s *
										  SPEED_ONE_METER_PER_SECOND_VALUE) : SPEED_UNAVAILABLE_VALUE;

	TRAJData.longitudinalAccelerationValueID = VALUE_ID_TRAJ_LONGITUDINAL_ACCELERATION;
	TRAJData.longitudinalAccelerationContentLength = sizeof (TRAJData.longitudinalAcceleration);
	TRAJData.lateralAccelerationValueID = VALUE_ID_TRAJ_LATERAL_ACCELERATION;
	TRAJData.lateralAccelerationContentLength = sizeof (TRAJData.lateralAcceleration);
	TRAJData.longitudinalAcceleration = acceleration.isLongitudinalValid ?
		(int16_t) (acceleration.longitudinal_m_s2 *
				   ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) : ACCELERATION_UNAVAILABLE_VALUE;
	TRAJData.lateralAcceleration =
		acceleration.isLateralValid ? (int16_t) (acceleration.lateral_m_s2 *
												 ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) :
		ACCELERATION_UNAVAILABLE_VALUE;

	TRAJData.curvatureValueID = VALUE_ID_TRAJ_CURVATURE;
	TRAJData.curvatureContentLength = sizeof (TRAJData.curvature);
	TRAJData.curvature = curvature;

	if (debug) {
		LogPrint("TRAJ message point:\n\t"
				 "Relative time value ID: 0x%x\n\t"
				 "Relative time content length: %u\n\t"
				 "Relative time: %u\n\t"
				 "x position value ID: 0x%x\n\t"
				 "x position content length: %u\n\t"
				 "x position: %d\n\t"
				 "y position value ID: 0x%x\n\t"
				 "y position content length: %u\n\t"
				 "y position: %d\n\t"
				 "z position value ID: 0x%x\n\t"
				 "z position content length: %u\n\t"
				 "z position: %d\n\t"
				 "Heading value ID: 0x%x\n\t"
				 "Heading content length: %u\n\t"
				 "Heading: %u\n\t"
				 "Longitudinal speed value ID: 0x%x\n\t"
				 "Longitudinal speed content length: %u\n\t"
				 "Longitudinal speed: %d\n\t"
				 "Lateral speed value ID: 0x%x\n\t"
				 "Lateral speed content length: %u\n\t"
				 "Lateral speed: %d\n\t"
				 "Longitudinal acceleration value ID: 0x%x\n\t"
				 "Longitudinal acceleration content length: %u\n\t"
				 "Longitudinal acceleration: %d\n\t"
				 "Lateral acceleration value ID: 0x%x\n\t"
				 "Lateral acceleration content length: %u\n\t"
				 "Lateral acceleration: %d\n\t"
				 "Curvature value ID: 0x%x\n\t"
				 "Curvature content length: %u\n\t"
				 "Curvature: %.6f",
				 TRAJData.relativeTimeValueID, TRAJData.relativeTimeContentLength,
				 TRAJData.relativeTime, TRAJData.xPositionValueID, TRAJData.xPositionContentLength,
				 TRAJData.xPosition, TRAJData.yPositionValueID, TRAJData.yPositionContentLength,
				 TRAJData.yPosition, TRAJData.zPositionValueID, TRAJData.zPositionContentLength,
				 TRAJData.zPosition, TRAJData.headingValueID, TRAJData.headingContentLength,
				 TRAJData.heading, TRAJData.longitudinalSpeedValueID, TRAJData.longitudinalSpeedContentLength,
				 TRAJData.longitudinalSpeed, TRAJData.lateralSpeedValueID, TRAJData.lateralSpeedContentLength,
				 TRAJData.lateralSpeed, TRAJData.longitudinalAccelerationValueID,
				 TRAJData.longitudinalAccelerationContentLength, TRAJData.longitudinalAcceleration,
				 TRAJData.lateralAccelerationValueID, TRAJData.lateralAccelerationContentLength,
				 TRAJData.lateralAcceleration, TRAJData.curvatureValueID, TRAJData.curvatureContentLength,
				 (double_t) TRAJData.curvature);
	}

	// Convert from host endianness to little endian
	TRAJData.relativeTimeValueID = htole16(TRAJData.relativeTimeValueID);
	TRAJData.relativeTimeContentLength = htole16(TRAJData.relativeTimeContentLength);
	TRAJData.relativeTime = htole32(TRAJData.relativeTime);
	TRAJData.xPositionValueID = htole16(TRAJData.xPositionValueID);
	TRAJData.xPositionContentLength = htole16(TRAJData.xPositionContentLength);
	TRAJData.xPosition = (int32_t) htole32(TRAJData.xPosition);
	TRAJData.yPositionValueID = htole16(TRAJData.yPositionValueID);
	TRAJData.yPositionContentLength = htole16(TRAJData.yPositionContentLength);
	TRAJData.yPosition = (int32_t) htole32(TRAJData.yPosition);
	TRAJData.zPositionValueID = htole16(TRAJData.zPositionValueID);
	TRAJData.zPositionContentLength = htole16(TRAJData.zPositionContentLength);
	TRAJData.zPosition = (int32_t) htole32(TRAJData.zPosition);
	TRAJData.headingValueID = htole16(TRAJData.headingValueID);
	TRAJData.headingContentLength = htole16(TRAJData.headingContentLength);
	TRAJData.heading = htole16(TRAJData.heading);
	TRAJData.longitudinalSpeedValueID = htole16(TRAJData.longitudinalSpeedValueID);
	TRAJData.longitudinalSpeedContentLength = htole16(TRAJData.longitudinalSpeedContentLength);
	TRAJData.longitudinalSpeed = (int16_t) htole16(TRAJData.longitudinalSpeed);
	TRAJData.lateralSpeedValueID = htole16(TRAJData.lateralSpeedValueID);
	TRAJData.lateralSpeedContentLength = htole16(TRAJData.lateralSpeedContentLength);
	TRAJData.lateralSpeed = (int16_t) htole16(TRAJData.lateralSpeed);
	TRAJData.longitudinalAccelerationValueID = htole16(TRAJData.longitudinalAccelerationValueID);
	TRAJData.longitudinalAccelerationContentLength = htole16(TRAJData.longitudinalAccelerationContentLength);
	TRAJData.longitudinalAcceleration = (int16_t) htole16(TRAJData.longitudinalAcceleration);
	TRAJData.lateralAccelerationValueID = htole16(TRAJData.lateralAccelerationValueID);
	TRAJData.lateralAccelerationContentLength = htole16(TRAJData.lateralAccelerationContentLength);
	TRAJData.lateralAcceleration = (int16_t) htole16(TRAJData.lateralAcceleration);
	TRAJData.curvatureValueID = htole16(TRAJData.curvatureValueID);
	TRAJData.curvatureContentLength = htole16(TRAJData.curvatureContentLength);
	TRAJData.curvature = htolef(TRAJData.curvature);

	memcpy(trajDataBufferPointer, &TRAJData, sizeof (TRAJData));

	// Update CRC
	dataLen = sizeof (TRAJData);
	while (dataLen-- > 0) {
		trajectoryMessageCrc = crcByte(trajectoryMessageCrc, (uint8_t) (*trajDataBufferPointer++));
	}

	return sizeof (TRAJPointType);
}


/*!
 * \brief encodeTRAJMessageFooter Creates a TRAJ message footer based on an internal CRC from previous header
 * and points, and prints it to a buffer.
 * \param trajDataBuffer Buffer to which TRAJ message is to be printed
 * \param remainingBufferLength Remaining bytes in the buffer to which the message is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold footer
 */
ssize_t encodeTRAJMessageFooter(char *trajDataBuffer, const size_t remainingBufferLength, const char debug) {

	TRAJFooterType TRAJData;

	if (remainingBufferLength < sizeof (TRAJFooterType)) {
		errno = ENOBUFS;
		LogMessage(LOG_LEVEL_DEBUG, "Buffer too small to hold TRAJ footer data");
		return -1;
	}
	else if (trajDataBuffer == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Invalid trajectory data buffer supplied");
		return -1;
	}

	TRAJData.footer.Crc = trajectoryMessageCrc;

	memcpy(trajDataBuffer, &TRAJData, sizeof (TRAJData));

	if (debug) {
		LogPrint("Encoded ISO footer:\n\tCRC: 0x%x", TRAJData.footer.Crc);
	}

	return sizeof (TRAJFooterType);
}


/*!
 * \brief encodeOSEMMessage Creates an OSEM message and writes it into a buffer based on supplied values. All values are passed as pointers and
 *  passing them as NULL causes the OSEM message to contain a default value for that field (a value representing "unavailable" or similar).
 * \param latitude_deg Latitude in degrees of the test origin
 * \param longitude_deg Longitude in degrees of the test origin
 * \param altitude_m Altitude in meters above sea level of the test origin
 * \param maxWayDeviation_m Maximum allowed deviation from target trajectory point, in meters
 * \param maxLateralDeviation_m Maximum lateral deviation from trajectory allowed, in meters
 * \param minimumPositioningAccuracy_m Minimum positioning accuracy required of the object
 * \param osemDataBuffer Buffer to which OSEM message is to be written
 * \param bufferLength Size of the buffer to which OSEM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to the buffer, or -1 in case of an error
 */
ssize_t encodeOSEMMessage(const double_t * latitude_deg, const double_t * longitude_deg,
						  const float *altitude_m, const float *maxWayDeviation_m,
						  const float *maxLateralDeviation_m, const float *minimumPositioningAccuracy_m,
						  char *osemDataBuffer, const size_t bufferLength, const char debug) {

	const char SizeDifference64bitTo48bit = 2;
	OSEMType OSEMData;
	struct timeval currentTime;
	struct tm *printableTime;
	char *p = osemDataBuffer;

	TimeSetToCurrentSystemTime(&currentTime);
	printableTime = localtime(&currentTime.tv_sec);

	memset(osemDataBuffer, 0, bufferLength);

	// If buffer too small to hold OSEM data, generate an error
	if (bufferLength < sizeof (OSEMData) - 2 * SizeDifference64bitTo48bit) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary OSEM data");
		return -1;
	}

	// Build header, and account for the two values which are 48 bit in the message
	OSEMData.header = buildISOHeader(MESSAGE_ID_OSEM, sizeof (OSEMData)
									 - 2 * SizeDifference64bitTo48bit, debug);

	// Fill the OSEM struct with relevant values
	OSEMData.latitudeValueID = VALUE_ID_OSEM_LATITUDE;
	OSEMData.latitudeContentLength = sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit;
	OSEMData.latitude = (latitude_deg == NULL) ?
		LATITUDE_UNAVAILABLE_VALUE : (int64_t) (*latitude_deg * LATITUDE_ONE_DEGREE_VALUE);

	OSEMData.longitudeValueID = VALUE_ID_OSEM_LONGITUDE;
	OSEMData.longitudeContentLength = sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit;
	OSEMData.longitude = (longitude_deg == NULL) ?
		LONGITUDE_UNAVAILABLE_VALUE : (int64_t) (*longitude_deg * LONGITUDE_ONE_DEGREE_VALUE);

	OSEMData.altitudeValueID = VALUE_ID_OSEM_ALTITUDE;
	OSEMData.altitudeContentLength = sizeof (OSEMData.altitude);
	OSEMData.altitude = (altitude_m == NULL) ?
		ALTITUDE_UNAVAILABLE_VALUE : (int32_t) (*altitude_m * ALTITUDE_ONE_METER_VALUE);

	OSEMData.dateValueID = VALUE_ID_OSEM_DATE;
	OSEMData.dateContentLength = sizeof (OSEMData.date);
	OSEMData.date = (uint32_t) ((printableTime->tm_year + 1900) * 10000 + (printableTime->tm_mon + 1) * 100
								+ (printableTime->tm_mday));

	OSEMData.GPSWeekValueID = VALUE_ID_OSEM_GPS_WEEK;
	OSEMData.GPSWeekContentLength = sizeof (OSEMData.GPSWeek);
	OSEMData.GPSWeek = TimeGetAsGPSweek(&currentTime);

	OSEMData.GPSQmsOfWeekValueID = VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK;
	OSEMData.GPSQmsOfWeekContentLength = sizeof (OSEMData.GPSQmsOfWeek);
	OSEMData.GPSQmsOfWeek = TimeGetAsGPSqmsOfWeek(&currentTime);

	OSEMData.maxWayDeviationValueID = VALUE_ID_OSEM_MAX_WAY_DEVIATION;
	OSEMData.maxWayDeviationContentLength = sizeof (OSEMData.maxWayDeviation);
	OSEMData.maxWayDeviation = (maxWayDeviation_m == NULL) ?
		MAX_WAY_DEVIATION_UNAVAILABLE_VALUE : (uint16_t) (*maxWayDeviation_m *
														  MAX_WAY_DEVIATION_ONE_METER_VALUE);

	OSEMData.maxLateralDeviationValueID = VALUE_ID_OSEM_MAX_LATERAL_DEVIATION;
	OSEMData.maxLateralDeviationContentLength = sizeof (OSEMData.maxLateralDeviation);
	OSEMData.maxLateralDeviation = (maxLateralDeviation_m == NULL) ?
		MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE : (uint16_t) (*maxLateralDeviation_m *
															  MAX_LATERAL_DEVIATION_ONE_METER_VALUE);

	OSEMData.minPosAccuracyValueID = VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY;
	OSEMData.minPosAccuracyContentLength = sizeof (OSEMData.minPosAccuracy);
	OSEMData.minPosAccuracy = (minimumPositioningAccuracy_m == NULL) ?
		MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE : (uint16_t) (*minimumPositioningAccuracy_m *
																  MIN_POSITIONING_ACCURACY_ONE_METER_VALUE);

	if (debug) {
		LogPrint
			("OSEM message:\n\tLatitude value ID: 0x%x\n\tLatitude content length: %u\n\tLatitude: %ld [100 nanodegrees]\n\t"
			 "Longitude value ID: 0x%x\n\tLongitude content length: %u\n\tLongitude: %ld [100 nanodegrees]\n\t"
			 "Altitude value ID: 0x%x\n\tAltitude content length: %u\n\tAltitude: %d [cm]\n\t"
			 "Date value ID: 0x%x\n\tDate content length: %u\n\tDate: %u\n\t"
			 "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\tGPS week: %u\n\t"
			 "GPS second of week value ID: 0x%x\n\tGPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			 "Max way deviation value ID: 0x%x\n\tMax way deviation content length: %u\n\tMax way deviation: %u\n\t"
			 "Max lateral deviation value ID: 0x%x\n\tMax lateral deviation content length: %u\n\t"
			 "Min positioning accuracy value ID: 0x%x\n\tMin positioning accuracy content length: %u\n\tMin positioning accuracy: %u",
			 OSEMData.latitudeValueID, OSEMData.latitudeContentLength, OSEMData.latitude,
			 OSEMData.longitudeValueID, OSEMData.longitudeContentLength, OSEMData.longitude,
			 OSEMData.altitudeValueID, OSEMData.altitudeContentLength, OSEMData.altitude,
			 OSEMData.dateValueID, OSEMData.dateContentLength, OSEMData.date, OSEMData.GPSWeekValueID,
			 OSEMData.GPSWeekContentLength, OSEMData.GPSWeek, OSEMData.GPSQmsOfWeekValueID,
			 OSEMData.GPSQmsOfWeekContentLength, OSEMData.GPSQmsOfWeek, OSEMData.maxWayDeviationValueID,
			 OSEMData.maxWayDeviationContentLength, OSEMData.maxWayDeviation,
			 OSEMData.maxLateralDeviationValueID, OSEMData.maxLateralDeviationContentLength,
			 OSEMData.maxLateralDeviation, OSEMData.minPosAccuracyValueID,
			 OSEMData.minPosAccuracyContentLength, OSEMData.minPosAccuracy);
	}

	// Switch endianness to little endian for all fields
	OSEMData.latitudeValueID = htole16(OSEMData.latitudeValueID);
	OSEMData.latitudeContentLength = htole16(OSEMData.latitudeContentLength);
	OSEMData.latitude = (int64_t) htole48(OSEMData.latitude);
	OSEMData.longitudeValueID = htole16(OSEMData.longitudeValueID);
	OSEMData.longitudeContentLength = htole16(OSEMData.longitudeContentLength);
	OSEMData.longitude = (int64_t) htole48(OSEMData.longitude);
	OSEMData.altitudeValueID = htole16(OSEMData.altitudeValueID);
	OSEMData.altitudeContentLength = htole16(OSEMData.altitudeContentLength);
	OSEMData.altitude = (int32_t) htole32(OSEMData.altitude);
	OSEMData.dateValueID = htole16(OSEMData.dateValueID);
	OSEMData.dateContentLength = htole16(OSEMData.dateContentLength);
	OSEMData.date = htole32(OSEMData.date);
	OSEMData.GPSWeekValueID = htole16(OSEMData.GPSWeekValueID);
	OSEMData.GPSWeekContentLength = htole16(OSEMData.GPSWeekContentLength);
	OSEMData.GPSWeek = htole16(OSEMData.GPSWeek);
	OSEMData.GPSQmsOfWeekValueID = htole16(OSEMData.GPSQmsOfWeekValueID);
	OSEMData.GPSQmsOfWeekContentLength = htole16(OSEMData.GPSQmsOfWeekContentLength);
	OSEMData.GPSQmsOfWeek = htole32(OSEMData.GPSQmsOfWeek);
	OSEMData.maxWayDeviationValueID = htole16(OSEMData.maxWayDeviationValueID);
	OSEMData.maxWayDeviationContentLength = htole16(OSEMData.maxWayDeviationContentLength);
	OSEMData.maxWayDeviation = htole16(OSEMData.maxWayDeviation);
	OSEMData.maxLateralDeviationValueID = htole16(OSEMData.maxLateralDeviationValueID);
	OSEMData.maxLateralDeviationContentLength = htole16(OSEMData.maxLateralDeviationContentLength);
	OSEMData.maxLateralDeviation = htole16(OSEMData.maxLateralDeviation);
	OSEMData.minPosAccuracyValueID = htole16(OSEMData.minPosAccuracyValueID);
	OSEMData.minPosAccuracyContentLength = htole16(OSEMData.minPosAccuracyContentLength);
	OSEMData.minPosAccuracy = htole16(OSEMData.minPosAccuracy);


	// Copy data from OSEM struct into the buffer
	// Must be done before constructing the footer due to the two 48bit size anomalies
	memcpy(p, &OSEMData.header, sizeof (OSEMData.header));
	p += sizeof (OSEMData.header);

	// Special handling of 48 bit value
	memcpy(p, &OSEMData.latitudeValueID,
		   sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength));
	p += sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength);
	memcpy(p, &OSEMData.latitude, sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit;

	// Special handling of 48 bit value
	memcpy(p, &OSEMData.longitudeValueID,
		   sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength));
	p += sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength);
	memcpy(p, &OSEMData.longitude, sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit;

	// Copy rest of struct (excluding footer) directly into buffer since no more byte anomalies remain
	memcpy(p, &OSEMData.altitudeValueID, sizeof (OSEMData) - sizeof (OSEMData.footer)
		   - (size_t) (p - osemDataBuffer + 2 * SizeDifference64bitTo48bit));
	p += sizeof (OSEMData) - sizeof (OSEMData.footer) - (size_t) (p - osemDataBuffer +
																  2 * SizeDifference64bitTo48bit);

	// Build footer
	OSEMData.footer =
		buildISOFooter(osemDataBuffer, sizeof (OSEMType) - 2 * SizeDifference64bitTo48bit, debug);
	memcpy(p, &OSEMData.footer, sizeof (OSEMData.footer));

	return sizeof (OSEMType) - 2 * SizeDifference64bitTo48bit;
}

/*!
 * \brief encodeOSTMMessage Constructs an ISO OSTM message based on specified command
 * \param command Command to send to object according to ::ObjectCommandType
 * \param ostmDataBuffer Data buffer to which OSTM is to be written
 * \param bufferLength Length of data buffer to which OSTM is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeOSTMMessage(const ObjectCommandType command, char *ostmDataBuffer, const size_t bufferLength,
						  const char debug) {

	OSTMType OSTMData;

	memset(ostmDataBuffer, 0, bufferLength);

	// Check so buffer can hold message
	if (bufferLength < sizeof (OSTMData)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary OSTM data");
		return -1;
	}

	// Check vs allowed commands
	if (!
		(command == OBJECT_COMMAND_ARM || command == OBJECT_COMMAND_DISARM
		 || command == OBJECT_COMMAND_REMOTE_CONTROL)) {
		LogMessage(LOG_LEVEL_ERROR, "OSTM does not support command %u", (uint8_t) command);
		return -1;
	}

	// Construct header
	OSTMData.header = buildISOHeader(MESSAGE_ID_OSTM, sizeof (OSTMData), debug);

	// Fill contents
	OSTMData.stateValueID = VALUE_ID_OSTM_STATE_CHANGE_REQUEST;
	OSTMData.stateContentLength = sizeof (OSTMData.state);
	OSTMData.state = (uint8_t) command;

	if (debug) {
		LogPrint("OSTM message:\n\tState change request value ID: 0x%x\n\t"
				 "State change request content length: %u\n\tState change request: %u",
				 OSTMData.stateValueID, OSTMData.stateContentLength, OSTMData.state);
	}

	// Convert from host endianness to little endian
	OSTMData.stateValueID = htole16(OSTMData.stateValueID);
	OSTMData.stateContentLength = htole16(OSTMData.stateContentLength);

	// Construct footer
	OSTMData.footer = buildISOFooter(&OSTMData, sizeof (OSTMData), debug);

	memcpy(ostmDataBuffer, &OSTMData, sizeof (OSTMData));

	return sizeof (OSTMType);
}


/*!
 * \brief encodeSTRTMessage Constructs an ISO STRT message based on start time parameters
 * \param timeOfStart Time when test shall start, a value of NULL indicates that the time is not known
 * \param strtDataBuffer Data buffer in which to place encoded STRT message
 * \param bufferLength Size of data buffer in which to place encoded STRT message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeSTRTMessage(const struct timeval *timeOfStart, char *strtDataBuffer,
						  const size_t bufferLength, const char debug) {

	STRTType STRTData;

	memset(strtDataBuffer, 0, bufferLength);

	// If buffer too small to hold STRT data, generate an error
	if (bufferLength < sizeof (STRTType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary STRT data");
		return -1;
	}

	STRTData.header = buildISOHeader(MESSAGE_ID_STRT, sizeof (STRTType), debug);

	// Fill contents
	STRTData.StartTimeValueIdU16 = VALUE_ID_STRT_GPS_QMS_OF_WEEK;
	STRTData.StartTimeContentLengthU16 = sizeof (STRTData.StartTimeU32);
	STRTData.StartTimeU32 =
		timeOfStart == NULL ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : TimeGetAsGPSqmsOfWeek(timeOfStart);
	STRTData.GPSWeekValueID = VALUE_ID_STRT_GPS_WEEK;
	STRTData.GPSWeekContentLength = sizeof (STRTData.GPSWeek);
	STRTData.GPSWeek = timeOfStart == NULL ? GPS_WEEK_UNAVAILABLE_VALUE : TimeGetAsGPSweek(timeOfStart);

	if (debug) {
		LogPrint("STRT message:\n\tGPS second of week value ID: 0x%x\n\t"
				 "GPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
				 "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\t"
				 "GPS week: %u", STRTData.StartTimeValueIdU16, STRTData.StartTimeContentLengthU16,
				 STRTData.StartTimeU32, STRTData.GPSWeekValueID, STRTData.GPSWeekContentLength,
				 STRTData.GPSWeek);
	}

	// Swap from host endianness to little endian
	STRTData.StartTimeValueIdU16 = htole16(STRTData.StartTimeValueIdU16);
	STRTData.StartTimeContentLengthU16 = htole16(STRTData.StartTimeContentLengthU16);
	STRTData.StartTimeU32 = htole32(STRTData.StartTimeU32);
	STRTData.GPSWeekValueID = htole16(STRTData.GPSWeekValueID);
	STRTData.GPSWeekContentLength = htole16(STRTData.GPSWeekContentLength);
	STRTData.GPSWeek = htole16(STRTData.GPSWeek);

	// Construct footer
	STRTData.footer = buildISOFooter(&STRTData, sizeof (STRTType), debug);

	memcpy(strtDataBuffer, &STRTData, sizeof (STRTType));

	return sizeof (STRTType);
}

/*!
 * \brief encodeHEABMessage Constructs an ISO HEAB message based on current control center status and system time
 * \param status Current control center status according to ::ControlCenterStatusType. Entering an unaccepable value
 *	makes this parameter default to ABORT
 * \param heabDataBuffer Buffer to which HEAB message is to be written
 * \param bufferLength Size of buffer to which HEAB message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeHEABMessage(const ControlCenterStatusType status, char *heabDataBuffer,
						  const size_t bufferLength, const char debug) {

	HEABType HEABData;
	struct timeval currentTime;

	TimeSetToCurrentSystemTime(&currentTime);

	memset(heabDataBuffer, 0, bufferLength);

	// If buffer too small to hold HEAB data, generate an error
	if (bufferLength < sizeof (HEABType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary HEAB data");
		return -1;
	}

	// Construct header
	HEABData.header = buildISOHeader(MESSAGE_ID_HEAB, sizeof (HEABData), debug);

	// Fill contents
	HEABData.HEABStructValueID = VALUE_ID_HEAB_STRUCT;
	HEABData.HEABStructContentLength = sizeof (HEABType) - sizeof (HeaderType) - sizeof (FooterType)
		- sizeof (HEABData.HEABStructValueID) - sizeof (HEABData.HEABStructContentLength);
	HEABData.GPSQmsOfWeek = TimeGetAsGPSqmsOfWeek(&currentTime);
	if (!(status == CONTROL_CENTER_STATUS_INIT || status == CONTROL_CENTER_STATUS_READY
		  || status == CONTROL_CENTER_STATUS_ABORT || status == CONTROL_CENTER_STATUS_RUNNING
		  || status == CONTROL_CENTER_STATUS_TEST_DONE || status == CONTROL_CENTER_STATUS_NORMAL_STOP)) {
		LogMessage(LOG_LEVEL_ERROR, "HEAB does not support status ID %u - defaulting to ABORT",
				   (uint8_t) status);
		HEABData.controlCenterStatus = (uint8_t) CONTROL_CENTER_STATUS_ABORT;
	}
	else {
		HEABData.controlCenterStatus = (uint8_t) status;
	}

	if (debug) {
		LogPrint("HEAB message:\n\tHEAB struct value ID: 0x%x\n\t"
				 "HEAB struct content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
				 "Control center status: 0x%x", HEABData.HEABStructValueID, HEABData.HEABStructContentLength,
				 HEABData.GPSQmsOfWeek, HEABData.controlCenterStatus);
	}

	// Switch from host endianness to little endian
	HEABData.HEABStructValueID = htole16(HEABData.HEABStructValueID);
	HEABData.HEABStructContentLength = htole16(HEABData.HEABStructContentLength);
	HEABData.GPSQmsOfWeek = htole32(HEABData.GPSQmsOfWeek);

	HEABData.footer = buildISOFooter(&HEABData, sizeof (HEABData), debug);

	memcpy(heabDataBuffer, &HEABData, sizeof (HEABData));

	return sizeof (HEABType);

}


/*!
 * \brief encodeRCMMMessage 
 * \param 
 * \param 
 * \param 
 * \param 
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeRCMMMessage(const uint8_t rcmmControlCommand, char *rcmmDataBuffer,
						  const size_t bufferLength, const char debug) {

	RCMMType RCMMData;

	memset(rcmmDataBuffer, 0, bufferLength);

	// If buffer too small to hold RCMM data, generate an error
	if (bufferLength < sizeof (RCMMType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary RCMM data");
		return -1;
	}

	// Construct header
	RCMMData.header = buildISOHeader(MESSAGE_ID_RCMM, sizeof (RCMMData), debug);

	// Fill contents
	RCMMData.RCMMControlValueID = VALUE_ID_RCMM_CONTROL;
	RCMMData.RCMMControlContentLength = 1;
	RCMMData.rcmmControlU8 = rcmmControlCommand;


	if (debug) {
		LogPrint("RCMM message:\n\tRCMM Control value ID: 0x%x\n\t"
				 "RCMM Control content length: %u\n\t"
				 "Control center status: %d", RCMMData.RCMMControlValueID, RCMMData.RCMMControlContentLength,
				 RCMMData.rcmmControlU8);
	}

	// Switch from host endianness to little endian
	RCMMData.RCMMControlValueID = htole16(RCMMData.RCMMControlValueID);
	RCMMData.RCMMControlContentLength = htole16(RCMMData.RCMMControlContentLength);

	RCMMData.footer = buildISOFooter(&RCMMData, sizeof (RCMMData), debug);

	memcpy(rcmmDataBuffer, &RCMMData, sizeof (RCMMData));

	return sizeof (RCMMType);

}


/*!
 * \brief buildMONRMessage Fills a MONRType struct from a buffer of raw data
 * \param MonrData Raw data to be decoded
 * \param MONRData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeMONRMessage(const char *monrDataBuffer, const size_t bufferLength,
										uint32_t * objectID, ObjectMonitorType * monitorData,
										const char debug) {

	MONRType MONRData;
	const char *p = monrDataBuffer;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (MONRData) - sizeof (MONRData.header)
														- sizeof (MONRData.footer.Crc) -
														sizeof (MONRData.monrStructValueID)
														- sizeof (MONRData.monrStructContentLength));
	ISOMessageReturnValue retval = MESSAGE_OK;

	memset(monitorData, 0, sizeof (*monitorData));
	*objectID = 0;

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &MONRData.header, debug)) != MESSAGE_OK) {
		memset(monitorData, 0, sizeof (*monitorData));
		return retval;
	}
	p += sizeof (MONRData.header);
	*objectID = MONRData.header.TransmitterIdU8;

	// If message is not a MONR message, generate an error
	if (MONRData.header.MessageIdU16 != MESSAGE_ID_MONR) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR message into MONR parsing function");
		return MESSAGE_TYPE_ERROR;
	}

	// Decode content header
	memcpy(&MONRData.monrStructValueID, p, sizeof (MONRData.monrStructValueID));
	p += sizeof (MONRData.monrStructValueID);
	MONRData.monrStructValueID = le16toh(MONRData.monrStructValueID);

	// If content is not a MONR struct or an unexpected size, generate an error
	if (MONRData.monrStructValueID != VALUE_ID_MONR_STRUCT) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&MONRData.monrStructContentLength, p, sizeof (MONRData.monrStructContentLength));
	p += sizeof (MONRData.monrStructContentLength);
	MONRData.monrStructContentLength = le16toh(MONRData.monrStructContentLength);

	if (MONRData.monrStructContentLength != ExpectedMONRStructSize) {
		LogMessage(LOG_LEVEL_ERROR, "MONR content length %u differs from the expected length %u",
				   MONRData.monrStructContentLength, ExpectedMONRStructSize);
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode content
	memcpy(&MONRData.gpsQmsOfWeek, p, sizeof (MONRData.gpsQmsOfWeek));
	p += sizeof (MONRData.gpsQmsOfWeek);
	MONRData.gpsQmsOfWeek = le32toh(MONRData.gpsQmsOfWeek);

	memcpy(&MONRData.xPosition, p, sizeof (MONRData.xPosition));
	p += sizeof (MONRData.xPosition);
	MONRData.xPosition = (int32_t) le32toh(MONRData.xPosition);

	memcpy(&MONRData.yPosition, p, sizeof (MONRData.yPosition));
	p += sizeof (MONRData.yPosition);
	MONRData.yPosition = (int32_t) le32toh(MONRData.yPosition);

	memcpy(&MONRData.zPosition, p, sizeof (MONRData.zPosition));
	p += sizeof (MONRData.zPosition);
	MONRData.zPosition = (int32_t) le32toh(MONRData.zPosition);

	memcpy(&MONRData.heading, p, sizeof (MONRData.heading));
	p += sizeof (MONRData.heading);
	MONRData.heading = le16toh(MONRData.heading);

	memcpy(&MONRData.longitudinalSpeed, p, sizeof (MONRData.longitudinalSpeed));
	p += sizeof (MONRData.longitudinalSpeed);
	MONRData.longitudinalSpeed = (int16_t) le16toh(MONRData.longitudinalSpeed);

	memcpy(&MONRData.lateralSpeed, p, sizeof (MONRData.lateralSpeed));
	p += sizeof (MONRData.lateralSpeed);
	MONRData.lateralSpeed = (int16_t) le16toh(MONRData.lateralSpeed);

	memcpy(&MONRData.longitudinalAcc, p, sizeof (MONRData.longitudinalAcc));
	p += sizeof (MONRData.longitudinalAcc);
	MONRData.longitudinalAcc = (int16_t) le16toh(MONRData.longitudinalAcc);

	memcpy(&MONRData.lateralAcc, p, sizeof (MONRData.lateralAcc));
	p += sizeof (MONRData.lateralAcc);
	MONRData.lateralAcc = (int16_t) le16toh(MONRData.lateralAcc);

	memcpy(&MONRData.driveDirection, p, sizeof (MONRData.driveDirection));
	p += sizeof (MONRData.driveDirection);

	memcpy(&MONRData.state, p, sizeof (MONRData.state));
	p += sizeof (MONRData.state);

	memcpy(&MONRData.readyToArm, p, sizeof (MONRData.readyToArm));
	p += sizeof (MONRData.readyToArm);

	memcpy(&MONRData.errorStatus, p, sizeof (MONRData.errorStatus));
	p += sizeof (MONRData.errorStatus);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - monrDataBuffer), &MONRData.footer,
						 debug)) != MESSAGE_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Error decoding MONR footer");
		return retval;
	}

	if ((retval = verifyChecksum(&MONRData, sizeof (MONRData) - sizeof (MONRData.footer),
								 MONRData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		LogMessage(LOG_LEVEL_WARNING, "MONR checksum error");
		return retval;
	}

	if (debug) {
		LogPrint("MONR:");
		LogPrint("SyncWord = %x", MONRData.header.SyncWordU16);
		LogPrint("TransmitterId = %d", MONRData.header.TransmitterIdU8);
		LogPrint("PackageCounter = %d", MONRData.header.MessageCounterU8);
		LogPrint("AckReq = %d", MONRData.header.AckReqProtVerU8);
		LogPrint("MessageId = %d", MONRData.header.MessageIdU16);
		LogPrint("MessageLength = %d", MONRData.header.MessageLengthU32);
		LogPrint("ValueId = %d", MONRData.monrStructValueID);
		LogPrint("ContentLength = %d", MONRData.monrStructContentLength);
		LogPrint("GPSSOW = %d", MONRData.gpsQmsOfWeek);
		LogPrint("XPosition = %d", MONRData.xPosition);
		LogPrint("YPosition = %d", MONRData.yPosition);
		LogPrint("ZPosition = %d", MONRData.zPosition);
		LogPrint("Heading = %d", MONRData.heading);
		LogPrint("LongitudinalSpeed = %d", MONRData.longitudinalSpeed);
		LogPrint("LateralSpeed = %d", MONRData.lateralSpeed);
		LogPrint("LongitudinalAcc = %d", MONRData.longitudinalAcc);
		LogPrint("LateralAcc = %d", MONRData.lateralAcc);
		LogPrint("DriveDirection = %d", MONRData.driveDirection);
		LogPrint("State = %d", MONRData.state);
		LogPrint("ReadyToArm = %d", MONRData.readyToArm);
		LogPrint("ErrorStatus = %d", MONRData.errorStatus);
	}

	// Fill output struct with parsed data
	convertMONRToHostRepresentation(&MONRData, monitorData);

	return retval;
}


/*!
 * \brief convertMONRToHostRepresentation Converts a MONR message to the internal representation for
 * object monitoring data
 * \param MONRData MONR message to be converted
 * \param monitorData Monitor data in which result is to be placed
 */
void convertMONRToHostRepresentation(const MONRType * MONRData, ObjectMonitorType * monitorData) {

	// Timestamp
	monitorData->isTimestampValid = MONRData->gpsQmsOfWeek != GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (monitorData->isTimestampValid) {
		struct timeval currentTime;

		TimeSetToCurrentSystemTime(&currentTime);
		TimeSetToGPStime(&monitorData->timestamp, TimeGetAsGPSweek(&currentTime), MONRData->gpsQmsOfWeek);
	}

	// Position / heading
	monitorData->position.xCoord_m = (double)(MONRData->xPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.yCoord_m = (double)(MONRData->yPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.zCoord_m = (double)(MONRData->zPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.isPositionValid = true;
	monitorData->position.isHeadingValid = MONRData->heading != HEADING_UNAVAILABLE_VALUE;
	if (monitorData->position.isHeadingValid) {
		monitorData->position.heading_rad =
			mapISOHeadingToHostHeading(MONRData->heading / 100.0 * M_PI / 180.0);
	}

	// Velocity
	monitorData->speed.isLongitudinalValid = MONRData->longitudinalSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.longitudinal_m_s = monitorData->speed.isLongitudinalValid ?
		(double)(MONRData->longitudinalSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;
	monitorData->speed.isLateralValid = MONRData->lateralSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.lateral_m_s = monitorData->speed.isLateralValid ?
		(double)(MONRData->lateralSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;

	// Acceleration
	monitorData->acceleration.isLongitudinalValid =
		MONRData->longitudinalAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.longitudinal_m_s2 =
		monitorData->acceleration.isLongitudinalValid ? (double)(MONRData->longitudinalAcc) /
		ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;
	monitorData->acceleration.isLateralValid = MONRData->lateralAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.lateral_m_s2 = monitorData->acceleration.isLateralValid ?
		(double)(MONRData->lateralAcc) / ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;

	// Drive direction
	switch (MONRData->driveDirection) {
	case ISO_DRIVE_DIRECTION_FORWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_FORWARD;
		break;
	case ISO_DRIVE_DIRECTION_BACKWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_BACKWARD;
		break;
	case ISO_DRIVE_DIRECTION_UNAVAILABLE:
	default:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
	}

	// State
	switch (MONRData->state) {
	case ISO_OBJECT_STATE_DISARMED:
		monitorData->state = OBJECT_STATE_DISARMED;
		break;
	case ISO_OBJECT_STATE_ARMED:
		monitorData->state = OBJECT_STATE_ARMED;
		break;
	case ISO_OBJECT_STATE_RUNNING:
		monitorData->state = OBJECT_STATE_RUNNING;
		break;
	case ISO_OBJECT_STATE_POSTRUN:
		monitorData->state = OBJECT_STATE_POSTRUN;
		break;
	case ISO_OBJECT_STATE_ABORTING:
		monitorData->state = OBJECT_STATE_ABORTING;
		break;
	case ISO_OBJECT_STATE_REMOTE_CONTROLLED:
		monitorData->state = OBJECT_STATE_REMOTE_CONTROL;
		break;
	case ISO_OBJECT_STATE_OFF:
	case ISO_OBJECT_STATE_INIT:
	default:
		monitorData->state = OBJECT_STATE_UNKNOWN;
		break;
	}

	// Ready to arm
	switch (MONRData->readyToArm) {
	case ISO_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_READY_TO_ARM;
		break;
	case ISO_NOT_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_NOT_READY_TO_ARM;
		break;
	case ISO_READY_TO_ARM_UNAVAILABLE:
	default:
		monitorData->armReadiness = OBJECT_READY_TO_ARM_UNAVAILABLE;
	}

	// Error status
	monitorData->error.engineFault = MONRData->errorStatus & BITMASK_ERROR_ENGINE_FAULT;
	monitorData->error.abortRequest = MONRData->errorStatus & BITMASK_ERROR_ABORT_REQUEST;
	monitorData->error.batteryFault = MONRData->errorStatus & BITMASK_ERROR_BATTERY_FAULT;
	monitorData->error.unknownError = MONRData->errorStatus & BITMASK_ERROR_OTHER
		|| MONRData->errorStatus & BITMASK_ERROR_VENDOR_SPECIFIC;
	monitorData->error.syncPointEnded = MONRData->errorStatus & BITMASK_ERROR_SYNC_POINT_ENDED;
	monitorData->error.outsideGeofence = MONRData->errorStatus & BITMASK_ERROR_OUTSIDE_GEOFENCE;
	monitorData->error.badPositioningAccuracy =
		MONRData->errorStatus & BITMASK_ERROR_BAD_POSITIONING_ACCURACY;

	return;
}


/*!
 * \brief encodeSYPMMessage Fills an ISO SYPM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param synchronizationTime Time along trajectory at which objects are to be synchronized
 * \param freezeTime Time along trajectory after which no further adaptation to the master is allowed
 * \param mtspDataBuffer Buffer to which SYPM message is to be written
 * \param bufferLength Size of buffer to which SYPM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeSYPMMessage(const struct timeval synchronizationTime, const struct timeval freezeTime,
						  char *sypmDataBuffer, const size_t bufferLength, const char debug) {

	SYPMType SYPMData;

	// If buffer too small to hold SYPM data, generate an error
	if (bufferLength < sizeof (SYPMType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary SYPM data");
		return -1;
	}

	// Construct header
	SYPMData.header = buildISOHeader(MESSAGE_ID_SYPM, sizeof (SYPMData), debug);

	// Fill contents
	SYPMData.syncPointTimeValueID = VALUE_ID_SYPM_SYNC_POINT_TIME;
	SYPMData.syncPointTimeContentLength = sizeof (SYPMData.syncPointTime);
	SYPMData.syncPointTime = (uint32_t) TimeGetAsUTCms(&synchronizationTime);

	SYPMData.freezeTimeValueID = VALUE_ID_SYPM_FREEZE_TIME;
	SYPMData.freezeTimeContentLength = sizeof (SYPMData.freezeTime);
	SYPMData.freezeTime = (uint32_t) TimeGetAsUTCms(&freezeTime);

	if (debug) {
		LogPrint("SYPM message:\n\tSynchronization point time value ID: 0x%x\n\t"
				 "Synchronization point time content length: %u\n\t"
				 "Synchronization point time: %u [ms]\n\t"
				 "Freeze time value ID: 0x%x\n\t"
				 "Freeze time content length: %u\n\t"
				 "Freeze time: %u [ms]", SYPMData.syncPointTimeValueID,
				 SYPMData.syncPointTimeContentLength, SYPMData.syncPointTime,
				 SYPMData.freezeTimeValueID, SYPMData.freezeTimeContentLength, SYPMData.freezeTime);
	}

	// Switch from host endianness to little endian
	SYPMData.syncPointTimeValueID = htole16(SYPMData.syncPointTimeValueID);
	SYPMData.syncPointTimeContentLength = htole16(SYPMData.syncPointTimeContentLength);
	SYPMData.syncPointTime = htole16(SYPMData.syncPointTime);
	SYPMData.freezeTimeValueID = htole16(SYPMData.freezeTimeValueID);
	SYPMData.freezeTimeContentLength = htole16(SYPMData.freezeTimeContentLength);
	SYPMData.freezeTime = htobe16(SYPMData.freezeTime);

	// Construct footer
	SYPMData.footer = buildISOFooter(&SYPMData, sizeof (SYPMData), debug);

	memcpy(sypmDataBuffer, &SYPMData, sizeof (SYPMData));

	return sizeof (SYPMType);
}

/*!
 * \brief encodeMTSPMessage Fills an ISO MTSP struct with relevant data fields, and corresponding value IDs and content lengths
 * \param estSyncPointTime Estimated time when the master object will reach the synchronization point
 * \param mtspDataBuffer Buffer to which MTSP message is to be written
 * \param bufferLength Size of buffer to which MTSP message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeMTSPMessage(const struct timeval *estSyncPointTime, char *mtspDataBuffer,
						  const size_t bufferLength, const char debug) {

	MTSPType MTSPData;

	memset(mtspDataBuffer, 0, bufferLength);

	// If buffer too small to hold MTSP data, generate an error
	if (bufferLength < sizeof (MTSPType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary MTSP data");
		return -1;
	}

	// Construct header
	MTSPData.header = buildISOHeader(MESSAGE_ID_MTSP, sizeof (MTSPData), debug);

	// Fill contents
	MTSPData.estSyncPointTimeValueID = VALUE_ID_MTSP_EST_SYNC_POINT_TIME;
	MTSPData.estSyncPointTimeContentLength = sizeof (MTSPData.estSyncPointTime);
	MTSPData.estSyncPointTime =
		estSyncPointTime ==
		NULL ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : TimeGetAsGPSqmsOfWeek(estSyncPointTime);

	if (debug) {
		LogPrint("MTSP message:\n\t"
				 "Estimated sync point time value ID: 0x%x\n\t"
				 "Estimated sync point time content length: %u\n\t"
				 "Estimated sync point time: %u [¼ ms]",
				 MTSPData.estSyncPointTimeValueID, MTSPData.estSyncPointTimeContentLength,
				 MTSPData.estSyncPointTime);
	}

	// Switch from host endianness to little endian
	MTSPData.estSyncPointTimeValueID = htole16(MTSPData.estSyncPointTimeValueID);
	MTSPData.estSyncPointTimeContentLength = htole16(MTSPData.estSyncPointTimeContentLength);
	MTSPData.estSyncPointTime = htole32(MTSPData.estSyncPointTime);

	// Construct footer
	MTSPData.footer = buildISOFooter(&MTSPData, sizeof (MTSPData), debug);

	memcpy(mtspDataBuffer, &MTSPData, sizeof (MTSPData));

	return sizeof (MTSPType);
}


/*!
 * \brief encodeTRCMMessage Fills an ISO TRCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param triggerID ID of the trigger to be configured
 * \param triggerType Type of the trigger to be configured according to ::TriggerType_t
 * \param param1 First parameter of the trigger to be configured according to ::TriggerTypeParameter_t
 * \param param2 Second parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param param3 Third parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param trcmDataBuffer Buffer to which TRCM message is to be written
 * \param bufferLength Size of buffer to which TRCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeTRCMMessage(const uint16_t * triggerID, const TriggerType_t * triggerType,
						  const TriggerTypeParameter_t * param1, const TriggerTypeParameter_t * param2,
						  const TriggerTypeParameter_t * param3, char *trcmDataBuffer,
						  const size_t bufferLength, const char debug) {

	TRCMType TRCMData;

	memset(trcmDataBuffer, 0, bufferLength);

	// If buffer too small to hold TRCM data, generate an error
	if (bufferLength < sizeof (TRCMType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary TRCM data");
		return -1;
	}

	// Construct header
	TRCMData.header = buildISOHeader(MESSAGE_ID_TRCM, sizeof (TRCMData), debug);

	// Fill contents
	TRCMData.triggerIDValueID = VALUE_ID_TRCM_TRIGGER_ID;
	TRCMData.triggerIDContentLength = sizeof (TRCMData.triggerID);
	TRCMData.triggerID = triggerID == NULL ? TRIGGER_ID_UNAVAILABLE : *triggerID;

	TRCMData.triggerTypeValueID = VALUE_ID_TRCM_TRIGGER_TYPE;
	TRCMData.triggerTypeContentLength = sizeof (TRCMData.triggerType);
	TRCMData.triggerType = triggerType == NULL ? TRIGGER_TYPE_UNAVAILABLE : (uint16_t) (*triggerType);

	TRCMData.triggerTypeParameter1ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1;
	TRCMData.triggerTypeParameter2ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2;
	TRCMData.triggerTypeParameter3ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3;

	TRCMData.triggerTypeParameter1ContentLength = sizeof (TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2ContentLength = sizeof (TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3ContentLength = sizeof (TRCMData.triggerTypeParameter3);

	TRCMData.triggerTypeParameter1 =
		param1 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	TRCMData.triggerTypeParameter2 =
		param2 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	TRCMData.triggerTypeParameter3 =
		param3 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		LogPrint("TRCM message:\n\tTrigger ID value ID: 0x%x\n\tTrigger ID content length: %u\n\t"
				 "Trigger ID: %u\n\tTrigger type value ID: 0x%x\n\tTrigger type content length: %u\n\t"
				 "Trigger type: %u\n\tTrigger type parameter 1 value ID: 0x%x\n\tTrigger type parameter 1 content length: %u\n\t"
				 "Trigger type parameter 1: %u\n\tTrigger type parameter 2 value ID: 0x%x\n\tTrigger type parameter 2 content length: %u\n\t"
				 "Trigger type parameter 2: %u\n\tTrigger type parameter 3 value ID: 0x%x\n\tTrigger type parameter 3 content length: %u"
				 "Trigger type parameter 3: %u\n\t", TRCMData.triggerIDValueID,
				 TRCMData.triggerIDContentLength, TRCMData.triggerID, TRCMData.triggerTypeValueID,
				 TRCMData.triggerTypeContentLength, TRCMData.triggerType,
				 TRCMData.triggerTypeParameter1ValueID, TRCMData.triggerTypeParameter1ContentLength,
				 TRCMData.triggerTypeParameter1, TRCMData.triggerTypeParameter2ValueID,
				 TRCMData.triggerTypeParameter2ContentLength, TRCMData.triggerTypeParameter2,
				 TRCMData.triggerTypeParameter3ValueID, TRCMData.triggerTypeParameter3ContentLength,
				 TRCMData.triggerTypeParameter3);
	}

	// Switch from host endianness to little endian
	TRCMData.triggerIDValueID = htole16(TRCMData.triggerIDValueID);
	TRCMData.triggerIDContentLength = htole16(TRCMData.triggerIDContentLength);
	TRCMData.triggerID = htole16(TRCMData.triggerID);

	TRCMData.triggerTypeValueID = htole16(TRCMData.triggerTypeValueID);
	TRCMData.triggerTypeContentLength = htole16(TRCMData.triggerTypeContentLength);
	TRCMData.triggerType = htole16(TRCMData.triggerType);

	TRCMData.triggerTypeParameter1ValueID = htole16(TRCMData.triggerTypeParameter1ValueID);
	TRCMData.triggerTypeParameter2ValueID = htole16(TRCMData.triggerTypeParameter2ValueID);
	TRCMData.triggerTypeParameter3ValueID = htole16(TRCMData.triggerTypeParameter3ValueID);

	TRCMData.triggerTypeParameter1ContentLength = htole16(TRCMData.triggerTypeParameter1ContentLength);
	TRCMData.triggerTypeParameter2ContentLength = htole16(TRCMData.triggerTypeParameter2ContentLength);
	TRCMData.triggerTypeParameter3ContentLength = htole16(TRCMData.triggerTypeParameter3ContentLength);

	TRCMData.triggerTypeParameter1 = htole32(TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2 = htole32(TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3 = htole32(TRCMData.triggerTypeParameter3);

	// Construct footer
	TRCMData.footer = buildISOFooter(&TRCMData, sizeof (TRCMData), debug);

	memcpy(trcmDataBuffer, &TRCMData, sizeof (TRCMData));

	return sizeof (TRCMData);
}





/*!
 * \brief encodeACCMMessage Fills an ISO ACCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param actionID ID of the action to be configured
 * \param actionType Type of the action to be configured according to ::ActionType_t
 * \param param1 First parameter of the action to be configured according to ::ActionTypeParameter_t
 * \param param2 Second parameter of the action to be configured ::ActionTypeParameter_t
 * \param param3 Third parameter of the action to be configured ::ActionTypeParameter_t
 * \param trcmDataBuffer Buffer to which ACCM message is to be written
 * \param bufferLength Size of buffer to which ACCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeACCMMessage(const uint16_t * actionID, const ActionType_t * actionType,
						  const ActionTypeParameter_t * param1, const ActionTypeParameter_t * param2,
						  const ActionTypeParameter_t * param3, char *accmDataBuffer,
						  const size_t bufferLength, const char debug) {

	ACCMType ACCMData;

	memset(accmDataBuffer, 0, bufferLength);

	// If buffer too small to hold ACCM data, generate an error
	if (bufferLength < sizeof (ACCMType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary ACCM data");
		return -1;
	}

	// Construct header
	ACCMData.header = buildISOHeader(MESSAGE_ID_ACCM, sizeof (ACCMData), debug);

	// Fill contents
	ACCMData.actionIDValueID = VALUE_ID_ACCM_ACTION_ID;
	ACCMData.actionIDContentLength = sizeof (ACCMData.actionID);
	ACCMData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	ACCMData.actionTypeValueID = VALUE_ID_ACCM_ACTION_TYPE;
	ACCMData.actionTypeContentLength = sizeof (ACCMData.actionType);
	ACCMData.actionType = actionType == NULL ? ACTION_TYPE_UNAVAILABLE : (uint16_t) (*actionType);

	ACCMData.actionTypeParameter1ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM1;
	ACCMData.actionTypeParameter2ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM2;
	ACCMData.actionTypeParameter3ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM3;

	ACCMData.actionTypeParameter1ContentLength = sizeof (ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2ContentLength = sizeof (ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3ContentLength = sizeof (ACCMData.actionTypeParameter3);

	ACCMData.actionTypeParameter1 = param1 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	ACCMData.actionTypeParameter2 = param2 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	ACCMData.actionTypeParameter3 = param3 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		LogPrint("ACCM message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\t"
				 "Action ID: %u\n\tAction type value ID: 0x%x\n\tAction type content length: %u\n\t"
				 "Action type: %u\n\tAction type parameter 1 value ID: 0x%x\n\tAction type parameter 1 content length: %u\n\t"
				 "Action type parameter 1: %u\n\tAction type parameter 2 value ID: 0x%x\n\tAction type parameter 2 content length: %u\n\t"
				 "Action type parameter 2: %u\n\tAction type parameter 3 value ID: 0x%x\n\tAction type parameter 3 content length: %u"
				 "Action type parameter 3: %u\n\t", ACCMData.actionIDValueID, ACCMData.actionIDContentLength,
				 ACCMData.actionID, ACCMData.actionTypeValueID, ACCMData.actionTypeContentLength,
				 ACCMData.actionType, ACCMData.actionTypeParameter1ValueID,
				 ACCMData.actionTypeParameter1ContentLength, ACCMData.actionTypeParameter1,
				 ACCMData.actionTypeParameter2ValueID, ACCMData.actionTypeParameter2ContentLength,
				 ACCMData.actionTypeParameter2, ACCMData.actionTypeParameter3ValueID,
				 ACCMData.actionTypeParameter3ContentLength, ACCMData.actionTypeParameter3);
	}

	// Switch from host endianness to little endian
	ACCMData.actionIDValueID = htole16(ACCMData.actionIDValueID);
	ACCMData.actionIDContentLength = htole16(ACCMData.actionIDContentLength);
	ACCMData.actionID = htole16(ACCMData.actionID);

	ACCMData.actionTypeValueID = htole16(ACCMData.actionTypeValueID);
	ACCMData.actionTypeContentLength = htole16(ACCMData.actionTypeContentLength);
	ACCMData.actionType = htole16(ACCMData.actionType);

	ACCMData.actionTypeParameter1ValueID = htole16(ACCMData.actionTypeParameter1ValueID);
	ACCMData.actionTypeParameter2ValueID = htole16(ACCMData.actionTypeParameter2ValueID);
	ACCMData.actionTypeParameter3ValueID = htole16(ACCMData.actionTypeParameter3ValueID);

	ACCMData.actionTypeParameter1ContentLength = htole16(ACCMData.actionTypeParameter1ContentLength);
	ACCMData.actionTypeParameter2ContentLength = htole16(ACCMData.actionTypeParameter2ContentLength);
	ACCMData.actionTypeParameter3ContentLength = htole16(ACCMData.actionTypeParameter3ContentLength);

	ACCMData.actionTypeParameter1 = htole32(ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2 = htole32(ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3 = htole32(ACCMData.actionTypeParameter3);

	// Construct footer
	ACCMData.footer = buildISOFooter(&ACCMData, sizeof (ACCMData), debug);

	memcpy(accmDataBuffer, &ACCMData, sizeof (ACCMData));

	return sizeof (ACCMData);
}



/*!
 * \brief encodeEXACMessage Fills an ISO EXAC struct with relevant data fields, and corresponding value IDs and content lengths
 * \param actionID ID of the action to be executed
 * \param executionTime Time when the action is to be executed
 * \param exacDataBuffer Buffer to which EXAC message is to be written
 * \param bufferLength Size of buffer to which EXAC message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeEXACMessage(const uint16_t * actionID, const struct timeval *executionTime,
						  char *exacDataBuffer, const size_t bufferLength, const char debug) {

	EXACType EXACData;

	memset(exacDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (EXACType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary EXAC data");
		return -1;
	}

	// Construct header
	EXACData.header = buildISOHeader(MESSAGE_ID_EXAC, sizeof (EXACData), debug);

	// Fill contents
	EXACData.actionIDValueID = VALUE_ID_EXAC_ACTION_ID;
	EXACData.actionIDContentLength = sizeof (EXACData.actionID);
	EXACData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	EXACData.executionTime_qmsoWValueID = VALUE_ID_EXAC_ACTION_EXECUTE_TIME;
	EXACData.executionTime_qmsoWContentLength = sizeof (EXACData.executionTime_qmsoW);
	EXACData.executionTime_qmsoW =
		executionTime == NULL ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : TimeGetAsGPSqmsOfWeek(executionTime);

	if (debug) {
		LogPrint
			("EXAC message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\tAction ID: %u\n\t"
			 "Action execute time value ID: 0x%x\n\tAction execute time content length: %u\n\tAction execute time: %u [¼ ms]",
			 EXACData.actionIDValueID, EXACData.actionIDContentLength, EXACData.actionID,
			 EXACData.executionTime_qmsoWValueID, EXACData.executionTime_qmsoWContentLength,
			 EXACData.executionTime_qmsoW);
	}

	// Switch from host endianness to little endian
	EXACData.actionIDValueID = htole16(EXACData.actionIDValueID);
	EXACData.actionIDContentLength = htole16(EXACData.actionIDContentLength);
	EXACData.actionID = htole16(EXACData.actionID);
	EXACData.executionTime_qmsoWValueID = htole16(EXACData.executionTime_qmsoWValueID);
	EXACData.executionTime_qmsoWContentLength = htole16(EXACData.executionTime_qmsoWContentLength);
	EXACData.executionTime_qmsoW = htole32(EXACData.executionTime_qmsoW);

	// Construct footer
	EXACData.footer = buildISOFooter(&EXACData, sizeof (EXACData), debug);

	memcpy(exacDataBuffer, &EXACData, sizeof (EXACData));

	return sizeof (EXACType);
}

/*!
 * \brief encodeINSUPMessage Fills an ISO vendor specific (RISE) INSUP struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param command Command to send to supervisor
 * \param insupDataBuffer Data buffer to which INSUP is to be written
 * \param bufferLength Length of data buffer to which INSUP is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeINSUPMessage(const SupervisorCommandType command, char *insupDataBuffer,
						   const size_t bufferLength, const char debug) {
	INSUPType INSUPData;

	memset(insupDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (INSUPType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary INSUP data");
		return -1;
	}

	// Construct header
	INSUPData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_RISE_INSUP, sizeof (INSUPData), debug);

	// Fill contents
	INSUPData.modeValueID = VALUE_ID_INSUP_MODE;
	INSUPData.modeContentLength = sizeof (INSUPData.mode);
	INSUPData.mode = (uint8_t) command;

	if (debug) {
		LogPrint("INSUP message:\n\tMode value ID: 0x%x\n\t"
				 "Mode content length: %u\n\tMode: %u", INSUPData.modeValueID,
				 INSUPData.modeContentLength, INSUPData.mode);
	}

	// Switch from host endianness to little endian
	INSUPData.modeValueID = htole16(INSUPData.modeValueID);
	INSUPData.modeContentLength = htole16(INSUPData.modeContentLength);

	// Construct footer
	INSUPData.footer = buildISOFooter(&INSUPData, sizeof (INSUPData), debug);

	memcpy(insupDataBuffer, &INSUPData, sizeof (INSUPData));

	return sizeof (INSUPData);
}


/*!
 * \brief mapISOHeadingToHostHeading Converts between ISO NED heading to internal heading measured from the test x axis
 * \param isoHeading_rad Heading measured according to ISO specification, in radians
 * \return Heading, in radians, measured from the test x axis
 */
double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = isoHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}

/*!
 * \brief mapHostHeadingToISOHeading Converts between internal heading measured from the test x axis to ISO NED heading
 * \param isoHeading_rad Heading measured form test x axis, in radians
 * \return Heading, in radians, measured as specified by ISO 22133
 */
double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = hostHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}
