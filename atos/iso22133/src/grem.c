#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "grem.h"
#include "iohelpers.h"


//! GREM field descriptions
static DebugStrings_t GREMSReceivedHeaderTransmitterDescription = {"Received Header Transmitter", "", &printU32};
static DebugStrings_t GREMSReceivedHeaderMessageIDDescription = {"Received Header Message ID", "", &printU16};
static DebugStrings_t GREMSReceivedHeaderMessageCounterDescription = {"Received Header Message Counter", "", &printU8};
static DebugStrings_t GREMResponseCodeDescription = {"Response Code",	"", &printU8};
static DebugStrings_t GREMPayloadLengthDescription = {"Payload Length",	"", &printU16};
static DebugStrings_t GREMPayloadDescription = {"Payload Data",	"", &printU8};


/*!
 * \brief decodeGREMMessage Fills GREM data elements from a buffer of raw data
 * \param gremDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param gremData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeGREMMessage(
		const char *gremDataBuffer,
		const size_t bufferLength,
		GeneralResponseMessageType* gremData,
		const char debug) {

	GREMType GREMdata;
	const char *p = gremDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (gremDataBuffer == NULL || gremData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to GREM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&GREMdata, 0, sizeof (GREMdata));
	memset(gremData, 0, sizeof (*gremData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &GREMdata.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (GREMdata.header);

	// If message is not a GREM message, generate an error
	if (GREMdata.header.messageID != MESSAGE_ID_GREM) {
		fprintf(stderr, "Attempted to pass non-GREM message into GREM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (GREMdata.header.messageLength > sizeof (GREMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "GREM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - gremDataBuffer < GREMdata.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_GREM_RECEIVED_HEADER_TRANSMITTER_ID:
			memcpy(&GREMdata.ReceivedHeaderTransmitterID, p, sizeof (GREMdata.ReceivedHeaderTransmitterID));
			GREMdata.ReceivedHeaderTransmitterIDValueID = valueID;
			GREMdata.ReceivedHeaderTransmitterIDContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.ReceivedHeaderTransmitterID);
			break;
		case VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_COUNTER:
			memcpy(&GREMdata.ReceivedHeaderMessageCounter, p, sizeof (GREMdata.ReceivedHeaderMessageCounter));
			GREMdata.ReceivedHeaderMessageCounterValueID = valueID;
			GREMdata.ReceivedHeaderMessageCounterContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.ReceivedHeaderMessageCounter);
			break;
		case VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_ID:
			memcpy(&GREMdata.ReceivedHeaderMessageID, p, sizeof (GREMdata.ReceivedHeaderMessageID));
			GREMdata.ReceivedHeaderMessageIDValueID = valueID;
			GREMdata.ReceivedHeaderMessageIDContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.ReceivedHeaderMessageID);
			break;
		case VALUE_ID_GREM_RESPONSE_CODE:
			memcpy(&GREMdata.ResponseCode, p, sizeof (GREMdata.ResponseCode));
			GREMdata.ResponseCodeValueID = valueID;
			GREMdata.ResponseCodeContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.ResponseCode);
			break;
		case VALUE_ID_GREM_PAYLOAD_LENGTH:
			memcpy(&GREMdata.PayloadLength, p, sizeof (GREMdata.PayloadLength));
			GREMdata.PayloadLengthValueID = valueID;
			GREMdata.PayloadLengthContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.PayloadLength);
			break;
		case VALUE_ID_GREM_PAYLOAD_DATA:
			GREMdata.PayloadDataValueID = valueID;
			GREMdata.PayloadDataContentLength = contentLength;
			expectedContentLength = contentLength;
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known GREM value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - gremDataBuffer), &GREMdata.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding GREM footer\n");
		return retval;
	}
	p += sizeof (GREMdata.footer);

	if (debug) {
		printf("GREM message:\n");
		printf("\tResponseCode transmitter ID value ID: 0x%x\n", GREMdata.ReceivedHeaderTransmitterID);
		printf("\tResponseCode: %u\n", GREMdata.ResponseCode);
	}

	// Fill output struct with parsed data
	retval = convertGREMoHostRepresentation(&GREMdata, gremData);
	return retval < 0 ? retval : p - gremDataBuffer;
}

/*!
 * \brief convertGREMToHostRepresentation Converts a GREM message to SI representation
 * \param GREMdata Data struct containing ISO formatted data
 * \param gremData Data struct to be filled with host representation
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertGREMoHostRepresentation(GREMType* GREMdata,
		GeneralResponseMessageType* gremData) {

	if (GREMdata == NULL || gremData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "GREM input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	if (!GREMdata->ResponseCodeValueID ) {
		fprintf(stderr, "Response Code Value ID not supplied in GREM message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	gremData->receivedHeaderTransmitterID = GREMdata->ReceivedHeaderTransmitterID;
	gremData->receivedHeaderMessageCounter = GREMdata->ReceivedHeaderMessageCounter;
	gremData->receivedHeaderMessageID = GREMdata->ReceivedHeaderMessageID;
	gremData->responseCode = GREMdata->ResponseCode;
	gremData->payloadLength = GREMdata->PayloadLength;
	// gremData->payloadData = GREMdata->PayloadData;

	return MESSAGE_OK;
}


/*!
 * \brief encodeGREMMessage Fills a GREM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param gremObjectData Struct containing relevant GREM data
 * \param gremDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeGREMMessage(
		const MessageHeaderType *inputHeader,
		const GeneralResponseMessageType* gremObjectData,
		char* gremDataBuffer,
		const size_t bufferLength,
		const char debug) {

	GREMType GREMData;

	memset(gremDataBuffer, 0, bufferLength);
	char* p = gremDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (gremObjectData == NULL) {
		fprintf(stderr, "GREM data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold GREM data, generate an error
	if (bufferLength < sizeof (GREMType)) {
		fprintf(stderr, "Buffer too small to hold necessary GREM data\n");
		return -1;
	}

	// Construct header
	GREMData.header = buildISOHeader(MESSAGE_ID_GREM, inputHeader, sizeof (GREMData), debug);
	memcpy(p, &GREMData.header, sizeof (GREMData.header));
	p += sizeof (GREMData.header);
	remainingBytes -= sizeof (GREMData.header);

	if (debug) {
			printf("GREM message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_GREM_RECEIVED_HEADER_TRANSMITTER_ID, &gremObjectData->receivedHeaderTransmitterID, &p,
						  sizeof (gremObjectData->receivedHeaderTransmitterID), &remainingBytes, &GREMSReceivedHeaderTransmitterDescription, debug);
	retval |= encodeContent(VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_COUNTER, &gremObjectData->receivedHeaderMessageCounter, &p,
						  sizeof (gremObjectData->receivedHeaderMessageCounter), &remainingBytes, &GREMSReceivedHeaderMessageCounterDescription, debug);
	retval |= encodeContent(VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_ID, &gremObjectData->receivedHeaderMessageID, &p,
						  sizeof (gremObjectData->receivedHeaderMessageID), &remainingBytes, &GREMSReceivedHeaderMessageIDDescription, debug);
	retval |= encodeContent(VALUE_ID_GREM_RESPONSE_CODE, &gremObjectData->responseCode, &p,
						  sizeof (gremObjectData->responseCode), &remainingBytes, &GREMResponseCodeDescription, debug);
	retval |= encodeContent(VALUE_ID_GREM_PAYLOAD_LENGTH, &gremObjectData->payloadLength, &p,
						  sizeof (gremObjectData->payloadLength), &remainingBytes, &GREMPayloadLengthDescription, debug);
	retval |= encodeContent(VALUE_ID_GREM_PAYLOAD_DATA, &gremObjectData->payload, &p,
						  sizeof (gremObjectData->payload), &remainingBytes, &GREMPayloadDescription, debug);
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary GREM data\n");
		return -1;
	}

	// Construct footer
	GREMData.footer = buildISOFooter(&GREMData, sizeof (GREMData), debug);
	memcpy(p, &GREMData.footer, sizeof (GREMData.footer));
	p += sizeof (GREMData.footer);
	remainingBytes -= sizeof (GREMData.footer);


	return p - gremDataBuffer;
}