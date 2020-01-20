#include "iso22133.h"
#include "string.h"
#include "logging.h"
#include "errno.h"

static const uint8_t SupportedProtocolVersions[] = { 2 };

// ************************** static functions
static int32_t buildISOHeader(const char * MessageBuffer, const size_t length, HeaderType * HeaderData, char Debug);
static int32_t buildISOFooter(const char * MessageBuffer, const size_t length, FooterType * HeaderData, char Debug);


// ************************** function definitions
void getSupportedISOProtocolVersions(const uint8_t ** supportedProtocolVersions, size_t * nProtocols) {
	*supportedProtocolVersions = SupportedProtocolVersions;
	*nProtocols = sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]);
}



int32_t buildMONRMessage(const char * MonrData, const size_t length, MONRType * MONRData, char debug) {

	const char *p = MonrData;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (*MONRData) - sizeof (MONRData->header)
											  - sizeof (MONRData->footer.Crc) -
											  sizeof (MONRData->monrStructValueID)
											  - sizeof (MONRData->monrStructContentLength));

	// Decode ISO header
	if (buildISOHeader(p, length, &MONRData->header, 0) == -1) {
		memset(MONRData, 0, sizeof (*MONRData));
		return -1;
	}
	p += sizeof (MONRData->header);

	// Decode content header
	memcpy(&MONRData->monrStructValueID, p, sizeof (MONRData->monrStructValueID));
	p += sizeof (MONRData->monrStructValueID);

	if (MONRData->monrStructValueID != VALUE_ID_MONR_STRUCT) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass non-MONR struct into MONR parsing function");
		memset(MONRData, 0, sizeof (*MONRData));
		return -1;
	}

	memcpy(&MONRData->monrStructContentLength, p, sizeof (MONRData->monrStructContentLength));
	p += sizeof (MONRData->monrStructContentLength);

	if (MONRData->monrStructContentLength != ExpectedMONRStructSize) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "MONR content length %u differs from the expected length %u",
				   MONRData->monrStructContentLength, ExpectedMONRStructSize);
		memset(MONRData, 0, sizeof (*MONRData));
		return -1;
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
	if (buildISOFooter(p, length-(size_t)(p-MonrData), &MONRData->footer, 0) == -1) {
		memset(MONRData, 0, sizeof (*MONRData));
		return -1;
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

	return 0;
}


int32_t buildISOHeader(const char* MessageBuffer, const size_t length, HeaderType * HeaderData, char Debug) {
	const char *p = MessageBuffer;
	int32_t retval = 0;
	const char ProtocolVersionBitmask = 0x7F;
	char messageProtocolVersion = 0;
	char isProtocolVersionSupported = 0;
	const uint8_t *supportedProtocolVersions;
	size_t nSupportedProtocols = 0;

	if (length < sizeof (HeaderData)) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO header");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return -1;
	}

	// Decode ISO header
	memcpy(&HeaderData->SyncWordU16, p, sizeof (HeaderData->SyncWordU16));
	p += sizeof (HeaderData->SyncWordU16);

	if (HeaderData->SyncWordU16 != ISO_SYNC_WORD) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Sync word error when decoding ISO header");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return -1;
	}

	memcpy(&HeaderData->TransmitterIdU8, p, sizeof (HeaderData->TransmitterIdU8));
	p += sizeof (HeaderData->TransmitterIdU8);

	memcpy(&HeaderData->MessageCounterU8, p, sizeof (HeaderData->MessageCounterU8));
	p += sizeof (HeaderData->MessageCounterU8);

	memcpy(&HeaderData->AckReqProtVerU8, p, sizeof (HeaderData->AckReqProtVerU8));
	p += sizeof (HeaderData->AckReqProtVerU8);

	// Loop over permitted protocol versions
	messageProtocolVersion = HeaderData->AckReqProtVerU8 & ProtocolVersionBitmask;
	getSupportedISOProtocolVersions(&supportedProtocolVersions, &nSupportedProtocols);
	for (size_t i = 0; i < nSupportedProtocols; ++i) {
		if (supportedProtocolVersions[i] == messageProtocolVersion) {
			isProtocolVersionSupported = 1;
			break;
		}
	}

	if (!isProtocolVersionSupported) {
		errno = EPROTONOSUPPORT;
		LogMessage(LOG_LEVEL_WARNING, "Protocol version %u not supported", messageProtocolVersion);
		retval = -1;
	}

	memcpy(&HeaderData->MessageIdU16, p, sizeof (HeaderData->MessageIdU16));
	p += sizeof (HeaderData->MessageIdU16);

	memcpy(&HeaderData->MessageLengthU32, p, sizeof (HeaderData->MessageLengthU32));
	p += sizeof (HeaderData->MessageLengthU32);

	if (Debug) {
		LogPrint("SyncWordU16 = 0x%x", HeaderData->SyncWordU16);
		LogPrint("TransmitterIdU8 = 0x%x", HeaderData->TransmitterIdU8);
		LogPrint("MessageCounterU8 = 0x%x", HeaderData->MessageCounterU8);
		LogPrint("AckReqProtVerU8 = 0x%x", HeaderData->AckReqProtVerU8);
		LogPrint("MessageIdU16 = 0x%x", HeaderData->MessageIdU16);
		LogPrint("MessageLengthU32 = 0x%x", HeaderData->MessageLengthU32);
	}

	return retval;
}


int32_t buildISOFooter(const char* MessageBuffer, const size_t length, FooterType * HeaderData, char Debug) {

	if (length < sizeof (HeaderData->Crc)) {
		LogMessage(LOG_LEVEL_ERROR, "Too little raw data to fill ISO footer");
		return -1;
	}
	memcpy(&HeaderData->Crc, MessageBuffer, sizeof (HeaderData->Crc));

	// TODO: check on CRC
	return 0;
}
