#include "iso22133.h"
#include "logging.h"
#include "maestroTime.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <endian.h>

static const uint8_t SupportedProtocolVersions[] = { 2 };

// ************************* Byte swapper definitions for 6 byte values
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define le48toh(x) __uint64_identity (x)
#define htole48(x) __uint64_identity (x)
#else
#define le48toh(x) (__bswap_64(x) >> 16)
#define htole48(x) (__bswap_64(x) >> 16)
#endif

// ************************** static function declarations
static ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);
static ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length,
											 FooterType * HeaderData, const char debug);
static HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug);
static FooterType buildISOFooter(const void *message, const size_t sizeExclFooter, const char debug);
static char isValidMessageID(const uint16_t id);

// ************************** function definitions

/*!
 * \brief decodeISOHeader Convert data in a buffer to an ISO header
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
 * \param messageLength Length of the message excluding header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO header data
 */
HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug) {
	HeaderType header;

	header.SyncWordU16 = ISO_SYNC_WORD;
	header.TransmitterIdU8 = 0;
	header.MessageCounterU8 = 0;
	header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	header.MessageIdU16 = (uint16_t) id;
	header.MessageLengthU32 = messageLength;

	if (debug) {
		LogPrint("Encoded ISO header:\n\tSync word: 0x%x\n\tTransmitter ID: %u\n\tMessage counter: %u\n\t"
				 "Ack request | Protocol version: 0x%x\n\tMessage ID: 0x%x\n\tMessage length: %u",
				 header.SyncWordU16, header.TransmitterIdU8, header.MessageCounterU8, header.AckReqProtVerU8,
				 header.MessageIdU16, header.MessageLengthU32);
	}

	return header;
}

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

	// Create header
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


ssize_t encodeOSEMMessage(const double * latitude_deg, const double * longitude_deg, const float * altitude_m, const float * maxWayDeviation_m, const float * maxLateralDeviation_m, const float * minimumPositioningAccuracy_m, char * osemDataBuffer, const size_t bufferLength, const char debug) {

	const char SizeDifference64bitTo48bit = 2;
	OSEMType OSEMData;
	struct timeval currentTime;
	struct tm * printableTime;
	char *p = osemDataBuffer;
	TimeSetToCurrentSystemTime(&currentTime);
	printableTime = localtime(&currentTime.tv_sec);

	memset(osemDataBuffer, 0, bufferLength);

	// If buffer too small to hold OSEM data, generate an error
	if (bufferLength < sizeof (OSEMData) - 2* SizeDifference64bitTo48bit) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary OSEM data");
		return -1;
	}

	// Build header, and account for the two values which are 48 bit in the message
	OSEMData.header = buildISOHeader(MESSAGE_ID_OSEM, sizeof (OSEMData)
									 - sizeof (OSEMData.header) - sizeof (OSEMData.footer) - 2*SizeDifference64bitTo48bit, debug);

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
	OSEMData.dateContentLength = sizeof (OSEMData.dateContentLength);
	OSEMData.date = (uint32_t) ((printableTime->tm_year + 1900) * 10000 + (printableTime->tm_mon + 1) * 100
			+ (printableTime->tm_mday));

	OSEMData.GPSWeekValueID = VALUE_ID_OSEM_GPS_WEEK;
	OSEMData.GPSWeekContentLength = sizeof (OSEMData.GPSWeekContentLength);
	OSEMData.GPSWeek = TimeGetAsGPSweek(&currentTime);

	OSEMData.GPSQmsOfWeekValueID = VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK;
	OSEMData.GPSQmsOfWeekContentLength = sizeof (OSEMData.GPSQmsOfWeek);
	OSEMData.GPSQmsOfWeek = TimeGetAsGPSqmsOfWeek(&currentTime);

	OSEMData.maxWayDeviationValueID = VALUE_ID_OSEM_MAX_WAY_DEVIATION;
	OSEMData.maxWayDeviationContentLength = sizeof (OSEMData.maxWayDeviation);
	OSEMData.maxWayDeviation = (maxWayDeviation_m == NULL) ?
				MAX_WAY_DEVIATION_UNAVAILABLE_VALUE : (uint16_t) (*maxWayDeviation_m * MAX_WAY_DEVIATION_ONE_METER_VALUE);

	OSEMData.maxLateralDeviationValueID = VALUE_ID_OSEM_MAX_LATERAL_DEVIATION;
	OSEMData.maxLateralDeviationContentLength = sizeof (OSEMData.maxLateralDeviation);
	OSEMData.maxLateralDeviation = (maxLateralDeviation_m == NULL) ?
				MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE : (uint16_t) (*maxLateralDeviation_m * MAX_LATERAL_DEVIATION_ONE_METER_VALUE);

	OSEMData.minPosAccuracyValueID = VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY;
	OSEMData.minPosAccuracyContentLength = sizeof (OSEMData.minPosAccuracy);
	OSEMData.minPosAccuracy = (minimumPositioningAccuracy_m == NULL) ?
				MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE : (uint16_t) (*minimumPositioningAccuracy_m * MIN_POSITIONING_ACCURACY_ONE_METER_VALUE);

	if (debug) {
		LogPrint("OSEM message:\n\tLatitude value ID: 0x%x\n\tLatitude content length: %u\n\tLatitude: %ld [100 nanodegrees]\n\t"
				 "Longitude value ID: 0x%x\n\tLongitude content length: %u\n\tLongitude: %ld [100 nanodegrees]\n\t"
				 "Altitude value ID: 0x%x\n\tAltitude content length: %u\n\tAltitude: %d [cm]"
				 "Date value ID: 0x%x\n\tDate content length: %u\n\tDate: %u\n\t"
				 "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\tGPS week: %u\n\t"
				 "GPS second of week value ID: 0x%x\n\tGPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
				 "Max way deviation value ID: 0x%x\n\tMax way deviation content length: %u\n\tMax way deviation: %u\n\t"
				 "Max lateral deviation value ID: 0x%x\n\tMax lateral deviation content length: %u\n\t"
				 "Min positioning accuracy value ID: 0x%x\n\tMin positioning accuracy content length: %u\n\tMin positioning accuracy: %u",
				 OSEMData.latitudeValueID, OSEMData.latitudeContentLength, OSEMData.latitude,
				 OSEMData.longitudeValueID, OSEMData.longitudeContentLength, OSEMData.longitude,
				 OSEMData.altitudeValueID, OSEMData.altitudeContentLength, OSEMData.altitude,
				 OSEMData.dateValueID, OSEMData.dateContentLength, OSEMData.date,
				 OSEMData.GPSWeekValueID, OSEMData.GPSWeekContentLength, OSEMData.GPSWeek,
				 OSEMData.GPSQmsOfWeekValueID, OSEMData.GPSQmsOfWeekContentLength, OSEMData.GPSQmsOfWeek,
				 OSEMData.maxWayDeviationValueID, OSEMData.maxWayDeviationContentLength, OSEMData.maxWayDeviation,
				 OSEMData.maxLateralDeviationValueID, OSEMData.maxLateralDeviationContentLength, OSEMData.maxLateralDeviation,
				 OSEMData.minPosAccuracyValueID, OSEMData.minPosAccuracyContentLength, OSEMData.minPosAccuracy);
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
	memcpy(p, &OSEMData.latitudeValueID, sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength));
	p += sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength);
	memcpy(p, &OSEMData.latitude, sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit;

	// Special handling of 48 bit value
	memcpy(p, &OSEMData.longitudeValueID, sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength));
	p += sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength);
	memcpy(p, &OSEMData.longitude, sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit;

	// Copy rest of struct (excluding footer) directly into buffer since no more byte anomalies remain
	memcpy(p, &OSEMData.altitude, sizeof (OSEMData) - sizeof (OSEMData.footer)
		   - (size_t) (p - osemDataBuffer + 2*SizeDifference64bitTo48bit));
	p += sizeof (OSEMData) - sizeof (OSEMData.footer) - (size_t) (p - osemDataBuffer + 2*SizeDifference64bitTo48bit);

	// Build footer
	OSEMData.footer = buildISOFooter(osemDataBuffer, sizeof (OSEMType) - 2*SizeDifference64bitTo48bit, debug);
	memcpy(p, &OSEMData.footer, sizeof (OSEMData.footer));

	return sizeof (OSEMType) - 2*SizeDifference64bitTo48bit;
}



/*!
 * \brief encodeSTRTMessage Constructs an ISO STRT message based on start time parameters
 * \param startTimeGPSqmsOW Quarter milliseconds of week when recipient of message shall start
 * \param startGPSWeek GPS week when recipient shall start
 * \param strtDataBuffer Data buffer in which to place encoded STRT message
 * \param bufferLength Size of data buffer in which to place encoded STRT message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeSTRTMessage(const uint32_t startTimeGPSqmsOW, const uint16_t startGPSWeek, char *strtDataBuffer,
						  const size_t bufferLength, const char debug) {

	STRTType STRTData;

	memset(strtDataBuffer, 0, bufferLength);

	// If buffer too small to hold STRT data, generate an error
	if (bufferLength < sizeof (STRTType)) {
		LogMessage(LOG_LEVEL_ERROR, "Buffer too small to hold necessary STRT data");
		return -1;
	}

	STRTData.header =
		buildISOHeader(MESSAGE_ID_STRT, sizeof (STRTType) - sizeof (HeaderType) - sizeof (FooterType), debug);

	// Fill contents, ensuring to swap from host endianness to little endian
	STRTData.StartTimeValueIdU16 = VALUE_ID_STRT_GPS_QMS_OF_WEEK;
	STRTData.StartTimeValueIdU16 = htole16(STRTData.StartTimeValueIdU16);
	STRTData.StartTimeContentLengthU16 = sizeof (STRTData.StartTimeU32);
	STRTData.StartTimeContentLengthU16 = htole16(STRTData.StartTimeContentLengthU16);
	STRTData.StartTimeU32 = startTimeGPSqmsOW;
	STRTData.StartTimeU32 = htole32(STRTData.StartTimeU32);
	STRTData.GPSWeekValueID = VALUE_ID_STRT_GPS_WEEK;
	STRTData.GPSWeekValueID = htole16(STRTData.GPSWeekValueID);
	STRTData.GPSWeekContentLength = sizeof (STRTData.GPSWeek);
	STRTData.GPSWeekContentLength = htole16(STRTData.GPSWeekContentLength);
	STRTData.GPSWeek = startGPSWeek;
	STRTData.GPSWeek = htole16(STRTData.GPSWeek);

	if (debug) {
		LogPrint("STRT message:\n\tGPS second of week value ID: 0x%x\n\t"
				 "GPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
				 "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\t"
				 "GPS week: %u", STRTData.StartTimeValueIdU16, STRTData.StartTimeContentLengthU16,
				 STRTData.StartTimeU32, STRTData.GPSWeekValueID, STRTData.GPSWeekContentLength,
				 STRTData.GPSWeek);
	}

	STRTData.footer = buildISOFooter(&STRTData, sizeof (STRTType), debug);

	memcpy(strtDataBuffer, &STRTData, sizeof (STRTType));

	return sizeof (STRTType);
}



/*!
 * \brief buildMONRMessage Fills a MONRType struct from a buffer of raw data
 * \param MonrData Raw data to be decoded
 * \param MONRData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeMONRMessage(const char *MonrData, const size_t length, MONRType * MONRData,
										const char debug) {

	const char *p = MonrData;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (*MONRData) - sizeof (MONRData->header)
														- sizeof (MONRData->footer.Crc) -
														sizeof (MONRData->monrStructValueID)
														- sizeof (MONRData->monrStructContentLength));
	ISOMessageReturnValue retval = MESSAGE_OK;

	// Decode ISO header
	if ((retval = decodeISOHeader(p, length, &MONRData->header, debug)) != MESSAGE_OK) {
		memset(MONRData, 0, sizeof (*MONRData));
		return retval;
	}
	p += sizeof (MONRData->header);

	// If message is not a MONR message, generate an error
	if (MONRData->header.MessageIdU16 != MESSAGE_ID_MONR) {
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_TYPE_ERROR;
	}

	// Decode content header
	memcpy(&MONRData->monrStructValueID, p, sizeof (MONRData->monrStructValueID));
	p += sizeof (MONRData->monrStructValueID);
	MONRData->monrStructValueID = le16toh(MONRData->monrStructValueID);

	// If content is not a MONR struct or an unexpected size, generate an error
	if (MONRData->monrStructValueID != VALUE_ID_MONR_STRUCT) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&MONRData->monrStructContentLength, p, sizeof (MONRData->monrStructContentLength));
	p += sizeof (MONRData->monrStructContentLength);
	MONRData->monrStructContentLength = le16toh(MONRData->monrStructContentLength);

	if (MONRData->monrStructContentLength != ExpectedMONRStructSize) {
		LogMessage(LOG_LEVEL_ERROR, "MONR content length %u differs from the expected length %u",
				   MONRData->monrStructContentLength, ExpectedMONRStructSize);
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode content
	memcpy(&MONRData->gpsQmsOfWeek, p, sizeof (MONRData->gpsQmsOfWeek));
	p += sizeof (MONRData->gpsQmsOfWeek);
	MONRData->gpsQmsOfWeek = le32toh(MONRData->gpsQmsOfWeek);

	memcpy(&MONRData->xPosition, p, sizeof (MONRData->xPosition));
	p += sizeof (MONRData->xPosition);
	MONRData->xPosition = (int32_t) le32toh(MONRData->xPosition);

	memcpy(&MONRData->yPosition, p, sizeof (MONRData->yPosition));
	p += sizeof (MONRData->yPosition);
	MONRData->yPosition = (int32_t) le32toh(MONRData->yPosition);

	memcpy(&MONRData->zPosition, p, sizeof (MONRData->zPosition));
	p += sizeof (MONRData->zPosition);
	MONRData->zPosition = (int32_t) le32toh(MONRData->zPosition);

	memcpy(&MONRData->heading, p, sizeof (MONRData->heading));
	p += sizeof (MONRData->heading);
	MONRData->heading = le16toh(MONRData->heading);

	memcpy(&MONRData->longitudinalSpeed, p, sizeof (MONRData->longitudinalSpeed));
	p += sizeof (MONRData->longitudinalSpeed);
	MONRData->longitudinalSpeed = (int16_t) le16toh(MONRData->longitudinalSpeed);

	memcpy(&MONRData->lateralSpeed, p, sizeof (MONRData->lateralSpeed));
	p += sizeof (MONRData->lateralSpeed);
	MONRData->lateralSpeed = (int16_t) le16toh(MONRData->lateralSpeed);

	memcpy(&MONRData->longitudinalAcc, p, sizeof (MONRData->longitudinalAcc));
	p += sizeof (MONRData->longitudinalAcc);
	MONRData->longitudinalAcc = (int16_t) le16toh(MONRData->longitudinalAcc);

	memcpy(&MONRData->lateralAcc, p, sizeof (MONRData->lateralAcc));
	p += sizeof (MONRData->lateralAcc);
	MONRData->lateralAcc = (int16_t) le16toh(MONRData->lateralAcc);

	memcpy(&MONRData->driveDirection, p, sizeof (MONRData->driveDirection));
	p += sizeof (MONRData->driveDirection);

	memcpy(&MONRData->state, p, sizeof (MONRData->state));
	p += sizeof (MONRData->state);

	memcpy(&MONRData->readyToArm, p, sizeof (MONRData->readyToArm));
	p += sizeof (MONRData->readyToArm);

	memcpy(&MONRData->errorStatus, p, sizeof (MONRData->errorStatus));
	p += sizeof (MONRData->errorStatus);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, length - (size_t) (p - MonrData), &MONRData->footer, debug)) != MESSAGE_OK) {
		memset(MONRData, 0, sizeof (*MONRData));
		return retval;
	}

	if (debug == 1) {
		LogPrint("MONR:");
		LogPrint("SyncWord = %x", MONRData->header.SyncWordU16);
		LogPrint("TransmitterId = %d", MONRData->header.TransmitterIdU8);
		LogPrint("PackageCounter = %d", MONRData->header.MessageCounterU8);
		LogPrint("AckReq = %d", MONRData->header.AckReqProtVerU8);
		LogPrint("MessageId = %d", MONRData->header.MessageIdU16);
		LogPrint("MessageLength = %d", MONRData->header.MessageLengthU32);
		LogPrint("ValueId = %d", MONRData->monrStructValueID);
		LogPrint("ContentLength = %d", MONRData->monrStructContentLength);
		LogPrint("GPSSOW = %d", MONRData->gpsQmsOfWeek);
		LogPrint("XPosition = %d", MONRData->xPosition);
		LogPrint("YPosition = %d", MONRData->yPosition);
		LogPrint("ZPosition = %d", MONRData->zPosition);
		LogPrint("Heading = %d", MONRData->heading);
		LogPrint("LongitudinalSpeed = %d", MONRData->longitudinalSpeed);
		LogPrint("LateralSpeed = %d", MONRData->lateralSpeed);
		LogPrint("LongitudinalAcc = %d", MONRData->longitudinalAcc);
		LogPrint("LateralAcc = %d", MONRData->lateralAcc);
		LogPrint("DriveDirection = %d", MONRData->driveDirection);
		LogPrint("State = %d", MONRData->state);
		LogPrint("ReadyToArm = %d", MONRData->readyToArm);
		LogPrint("ErrorStatus = %d", MONRData->errorStatus);
	}

	return retval;
}


/*!
 * \brief MONRToASCII Converts a MONR struct into human readable ASCII text
 * \param MONRData Struct containing MONR data
 * \param asciiBuffer Buffer in which to print ASCII text representation
 * \param bufferLength Length of ASCII buffer
 * \param debug Flag for enabling debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue MONRToASCII(const MONRType * MONRData, char *asciiBuffer, const size_t bufferLength,
								  const char debug) {

	memset(asciiBuffer, 0, bufferLength);
	if (MONRData->header.MessageIdU16 != MESSAGE_ID_MONR) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		return MESSAGE_TYPE_ERROR;
	}

	sprintf(asciiBuffer + strlen(asciiBuffer), "%u;", MONRData->gpsQmsOfWeek);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%d;%d;%d;%u;",
			MONRData->xPosition, MONRData->yPosition, MONRData->zPosition, MONRData->heading);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%d;%d;%d;%d;", MONRData->longitudinalSpeed,
			MONRData->lateralSpeed, MONRData->longitudinalAcc, MONRData->lateralAcc);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%u;%u;%u;%u;", MONRData->driveDirection,
			MONRData->state, MONRData->readyToArm, MONRData->errorStatus);

	if (debug) {
		LogPrint("Converted MONR to string:\n\tMONR:\n\t\t"
				 "Value ID = 0x%x\n\t\t"
				 "Content length = %u\n\t\t"
				 "GPS quarter millisecond of week = %u\n\t\t"
				 "X position = %d\n\t\t"
				 "Y position = %d\n\t\t"
				 "Z position = %d\n\t\t"
				 "Heading = %d\n\t\t"
				 "Longitudinal speed = %d\n\t\t"
				 "Lateral speed = %d\n\t\t"
				 "Longitudinal acceleration = %d\n\t\t"
				 "Lateral acceleration = %d\n\t\t"
				 "Drive direction = %d\n\t\t"
				 "State = %d\n\t\t"
				 "Ready to arm = %d\n\t\t"
				 "Error status = %d\n\t"
				 "String:\n\t\t<%s>", MONRData->monrStructValueID, MONRData->monrStructContentLength,
				 MONRData->gpsQmsOfWeek, MONRData->xPosition, MONRData->yPosition, MONRData->zPosition,
				 MONRData->heading, MONRData->longitudinalSpeed, MONRData->lateralSpeed, MONRData->longitudinalAcc,
				 MONRData->lateralAcc, MONRData->driveDirection, MONRData->state, MONRData->readyToArm,
				 MONRData->errorStatus, asciiBuffer);
	}

	return MESSAGE_OK;
}


/*!
 * \brief MONRToASCII Converts an ASCII string into a MONR struct
 * \param asciiBuffer Buffer containing ASCII text representation
 * \param MONRData Struct containing MONR data
 * \param debug Flag for enabling debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue ASCIIToMONR(const char *asciiBuffer, MONRType * MONRData, const char debug) {

	const char *token;
	const char delim[] = ";";
	const int NumberBaseDecimal = 10;
	char *copy = strdup(asciiBuffer);

	memset(MONRData, 0, sizeof (*MONRData));

	token = strtok(copy, delim);

	token = strtok(NULL, delim);
	MONRData->gpsQmsOfWeek = (uint32_t) strtoul(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->xPosition = (int32_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->yPosition = (int32_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->zPosition = (int32_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->heading = (uint16_t) strtoul(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->longitudinalSpeed = (int16_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->lateralSpeed = (int16_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->longitudinalAcc = (int16_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->lateralAcc = (int16_t) strtol(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->driveDirection = (uint8_t) strtoul(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->state = (uint8_t) strtoul(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->readyToArm = (uint8_t) strtoul(token, NULL, NumberBaseDecimal);

	token = strtok(NULL, delim);
	MONRData->errorStatus = (uint8_t) strtoul(token, NULL, NumberBaseDecimal);

	if (debug) {
		LogPrint("Converted string to MONR:\n\tString:\n\t\t<%s>\n\tMONR:\n\t\t"
				 "Value ID = 0x%x\n\t\t"
				 "Content length = %u\n\t\t"
				 "GPS quarter millisecond of week = %u\n\t\t"
				 "X position = %d\n\t\t"
				 "Y position = %d\n\t\t"
				 "Z position = %d\n\t\t"
				 "Heading = %d\n\t\t"
				 "Longitudinal speed = %d\n\t\t"
				 "Lateral speed = %d\n\t\t"
				 "Longitudinal acceleration = %d\n\t\t"
				 "Lateral acceleration = %d\n\t\t"
				 "Drive direction = %d\n\t\t"
				 "State = %d\n\t\t"
				 "Ready to arm = %d\n\t\t"
				 "Error status = %d", asciiBuffer, MONRData->monrStructValueID, MONRData->monrStructContentLength,
				 MONRData->gpsQmsOfWeek, MONRData->xPosition, MONRData->yPosition, MONRData->zPosition,
				 MONRData->heading, MONRData->longitudinalSpeed, MONRData->lateralSpeed, MONRData->longitudinalAcc,
				 MONRData->lateralAcc, MONRData->driveDirection, MONRData->state, MONRData->readyToArm,
				 MONRData->errorStatus);
	}

	return MESSAGE_OK;
}
