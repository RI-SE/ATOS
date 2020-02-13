#include "iso22133.h"
#include "logging.h"
#include "maestroTime.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <endian.h>
#include <math.h>

// ************************* ISO protocol versions supported by functions in this file ***************************
static const uint8_t SupportedProtocolVersions[] = { 2 };



// ************************* Type definitions according ISO protocol specification *******************************
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
#define MIN_POSITIONING_ACCURACY_ONE_METER_VALUE 1000	// ISO specification unclear on this value
#define TRIGGER_ID_UNAVAILABLE 65535
#define TRIGGER_TYPE_UNAVAILABLE 65535
#define TRIGGER_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define ACTION_ID_UNAVAILABLE 65535
#define ACTION_TYPE_UNAVAILABLE 65535
#define ACTION_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define POSITION_ONE_METER_VALUE 1000
#define HEADING_UNAVAILABLE_VALUE 36001
#define SPEED_UNAVAILABLE_VALUE (-32768)
#define SPEED_ONE_METER_PER_SECOND_VALUE 100
#define ACCELERATION_UNAVAILABLE_VALUE 32001
#define ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE 1000
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


#pragma pack(pop)


// ************************* Non-ISO type definitions and defines ************************************************
// Byte swapper definitions for 6 byte values
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define le48toh(x) (x)
#define htole48(x) (x)
#else
#define le48toh(x) (le64toh(x) >> 16)
#define htole48(x) (htole64(x) >> 16)
#endif

// ************************** static function declarations ********************************************************
static ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);
static ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length,
											 FooterType * HeaderData, const char debug);
static HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug);
static FooterType buildISOFooter(const void *message, const size_t sizeExclFooter, const char debug);
static char isValidMessageID(const uint16_t id);
static double mapISOHeadingToHostHeading(const double isoHeading_rad);
static void convertMONRToHostRepresentation(const MONRType * MONRData, ObjectMonitorType * monitorData);

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

	// TODO: check on CRC
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

	// TODO: Calculate CRC - remembering that message begins with header and messageSize will include header and footer
	footer.Crc = 0;
	footer.Crc = htole16(footer.Crc);

	if (debug) {
		LogPrint("Encoded ISO footer:\n\tCRC: 0x%x", footer.Crc);
	}

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
		|| id == MESSAGE_ID_CATA || id == MESSAGE_ID_RCCM || id == MESSAGE_ID_RCRT || id == MESSAGE_ID_PIME
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
ssize_t encodeOSEMMessage(const double *latitude_deg, const double *longitude_deg, const float *altitude_m,
						  const float *maxWayDeviation_m, const float *maxLateralDeviation_m,
						  const float *minimumPositioningAccuracy_m, char *osemDataBuffer,
						  const size_t bufferLength, const char debug) {

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
	monitorData->speed.isValid = true;
	if (MONRData->longitudinalSpeed == SPEED_UNAVAILABLE_VALUE)
		monitorData->speed.isValid = false;
	else
		monitorData->speed.longitudinal_m_s =
			(double)(MONRData->longitudinalSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE;
	if (MONRData->lateralSpeed == SPEED_UNAVAILABLE_VALUE)
		monitorData->speed.isValid = false;
	else
		monitorData->speed.lateral_m_s = (double)(MONRData->lateralSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE;

	// Acceleration
	monitorData->acceleration.isValid = true;
	if (MONRData->longitudinalAcc == ACCELERATION_UNAVAILABLE_VALUE)
		monitorData->acceleration.isValid = false;
	else
		monitorData->acceleration.longitudinal_m_s2 =
			(double)(MONRData->longitudinalAcc) / ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE;
	if (MONRData->lateralAcc == ACCELERATION_UNAVAILABLE_VALUE)
		monitorData->acceleration.isValid = false;
	else
		monitorData->acceleration.lateral_m_s2 =
			(double)(MONRData->lateralAcc) / ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE;

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
double mapISOHeadingToHostHeading(const double isoHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double retval = isoHeading_rad;

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
