#include "iso22133.h"
#include "string.h"
#include "logging.h"
#include "errno.h"
#include "stdlib.h"

static const uint8_t SupportedProtocolVersions[] = { 2 };

// ************************** static functions
static ISOMessageReturnValue buildISOHeader(const char * MessageBuffer, const size_t length, HeaderType * HeaderData, const char debug);
static ISOMessageReturnValue buildISOFooter(const char * MessageBuffer, const size_t length, FooterType * HeaderData, const char debug);

// ************************** function definitions

/*!
 * \brief buildISOHeader Convert data in a buffer to an ISO header
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue buildISOHeader(const char* MessageBuffer, const size_t length, HeaderType * HeaderData, const char debug) {
	const char *p = MessageBuffer;
	ISOMessageReturnValue retval = MESSAGE_OK;
	const char ProtocolVersionBitmask = 0x7F;
	char messageProtocolVersion = 0;
	char isProtocolVersionSupported = 0;

	if (length < sizeof (HeaderData)) {
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO header");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode ISO header
	memcpy(&HeaderData->SyncWordU16, p, sizeof (HeaderData->SyncWordU16));
	p += sizeof (HeaderData->SyncWordU16);

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

	// Loop over permitted protocol versions
	messageProtocolVersion = HeaderData->AckReqProtVerU8 & ProtocolVersionBitmask;
	for (size_t i = 0; i < sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]); ++i) {
		if (SupportedProtocolVersions[i] == messageProtocolVersion) {
			isProtocolVersionSupported = 1;
			break;
		}
	}

	if (!isProtocolVersionSupported) {
		LogMessage(LOG_LEVEL_WARNING, "Protocol version %u not supported", messageProtocolVersion);
		retval = MESSAGE_VERSION_ERROR;
		memset(HeaderData, 0, sizeof (*HeaderData));
		return retval;
	}

	memcpy(&HeaderData->MessageIdU16, p, sizeof (HeaderData->MessageIdU16));
	p += sizeof (HeaderData->MessageIdU16);

	memcpy(&HeaderData->MessageLengthU32, p, sizeof (HeaderData->MessageLengthU32));
	p += sizeof (HeaderData->MessageLengthU32);

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
 * \brief buildISOFooter Convert data in a buffer to an ISO footer
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue buildISOFooter(const char* MessageBuffer, const size_t length, FooterType * FooterData, const char debug) {

	if (length < sizeof (FooterData->Crc)) {
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO footer");
		memset(FooterData, 0, sizeof (*FooterData));
		return MESSAGE_LENGTH_ERROR;
	}
	memcpy(&FooterData->Crc, MessageBuffer, sizeof (FooterData->Crc));

	// TODO: check on CRC
	return MESSAGE_OK;
}


/*!
 * \brief buildMONRMessage Fills a MONRType struct from a buffer of raw data
 * \param MonrData Raw data to be decoded
 * \param MONRData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue buildMONRMessage(const char * MonrData, const size_t length, MONRType * MONRData, const char debug) {

	const char *p = MonrData;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (*MONRData) - sizeof (MONRData->header)
											  - sizeof (MONRData->footer.Crc) -
											  sizeof (MONRData->monrStructValueID)
											  - sizeof (MONRData->monrStructContentLength));
	ISOMessageReturnValue retval = MESSAGE_OK;

	// Decode ISO header
	if ((retval = buildISOHeader(p, length, &MONRData->header, debug)) != MESSAGE_OK) {
		memset(MONRData, 0, sizeof (*MONRData));
		return retval;
	}
	p += sizeof (MONRData->header);

	if (MONRData->header.MessageIdU16 != MESSAGE_ID_MONR) {
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_TYPE_ERROR;
	}

	// Decode content header
	memcpy(&MONRData->monrStructValueID, p, sizeof (MONRData->monrStructValueID));
	p += sizeof (MONRData->monrStructValueID);

	if (MONRData->monrStructValueID != VALUE_ID_MONR_STRUCT) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&MONRData->monrStructContentLength, p, sizeof (MONRData->monrStructContentLength));
	p += sizeof (MONRData->monrStructContentLength);

	if (MONRData->monrStructContentLength != ExpectedMONRStructSize) {
		LogMessage(LOG_LEVEL_ERROR, "MONR content length %u differs from the expected length %u",
				   MONRData->monrStructContentLength, ExpectedMONRStructSize);
		memset(MONRData, 0, sizeof (*MONRData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode content
	memcpy(&MONRData->gpsQmsOfWeek, p, sizeof (MONRData->gpsQmsOfWeek));
	p += sizeof (MONRData->gpsQmsOfWeek);

	memcpy(&MONRData->xPosition, p, sizeof (MONRData->xPosition));
	p += sizeof (MONRData->xPosition);

	memcpy(&MONRData->yPosition, p, sizeof (MONRData->yPosition));
	p += sizeof (MONRData->yPosition);

	memcpy(&MONRData->zPosition, p, sizeof (MONRData->zPosition));
	p += sizeof (MONRData->zPosition);

	memcpy(&MONRData->heading, p, sizeof (MONRData->heading));
	p += sizeof (MONRData->heading);

	memcpy(&MONRData->longitudinalSpeed, p, sizeof (MONRData->longitudinalSpeed));
	p += sizeof (MONRData->longitudinalSpeed);

	memcpy(&MONRData->lateralSpeed, p, sizeof (MONRData->lateralSpeed));
	p += sizeof (MONRData->lateralSpeed);

	memcpy(&MONRData->longitudinalAcc, p, sizeof (MONRData->longitudinalAcc));
	p += sizeof (MONRData->longitudinalAcc);

	memcpy(&MONRData->lateralAcc, p, sizeof (MONRData->lateralAcc));
	p += sizeof (MONRData->lateralAcc);

	memcpy(&MONRData->driveDirection, p, sizeof (MONRData->driveDirection));
	p += sizeof (MONRData->driveDirection);

	memcpy(&MONRData->state, p, sizeof (MONRData->state));
	p += sizeof (MONRData->state);

	memcpy(&MONRData->readyToArm, p, sizeof (MONRData->readyToArm));
	p += sizeof (MONRData->readyToArm);

	memcpy(&MONRData->errorStatus, p, sizeof (MONRData->errorStatus));
	p += sizeof (MONRData->errorStatus);

	// Footer
	if ((retval = buildISOFooter(p, length-(size_t)(p-MonrData), &MONRData->footer, debug)) != MESSAGE_OK) {
		memset(MONRData, 0, sizeof (*MONRData));
		return retval;
	}
	p += sizeof (MONRData->footer);

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
ISOMessageReturnValue MONRToASCII(const MONRType * MONRData, char * asciiBuffer, const size_t bufferLength, const char debug) {

	memset(asciiBuffer, 0, bufferLength);
	if (MONRData->header.MessageIdU16 != MESSAGE_ID_MONR) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		return MESSAGE_TYPE_ERROR;
	}

	sprintf(asciiBuffer + strlen(asciiBuffer), "%u;", MONRData->gpsQmsOfWeek);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%d;%d;%d;%u;",
			MONRData->xPosition, MONRData->yPosition, MONRData->zPosition,
			MONRData->heading);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%d;%d;%d;%d;", MONRData->longitudinalSpeed,
			MONRData->lateralSpeed, MONRData->longitudinalAcc, MONRData->lateralAcc);
	sprintf(asciiBuffer + strlen(asciiBuffer), "%u;%u;%u;%u;", MONRData->driveDirection,
			MONRData->state, MONRData->readyToArm, MONRData->errorStatus);

	return MESSAGE_OK;
}


/*!
 * \brief MONRToASCII Converts an ASCII string into a MONR struct
 * \param asciiBuffer Buffer containing ASCII text representation
 * \param MONRData Struct containing MONR data
 * \param debug Flag for enabling debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue ASCIIToMONR(const char * asciiBuffer, MONRType * MONRData, const char debug) {

	const size_t bufferLength = strlen(asciiBuffer);
	const char *token;
	const char delim[] = ";";
	const int NumberBaseDecimal = 10;
	char * copy = strdup(asciiBuffer);

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

	return MESSAGE_OK;
}
