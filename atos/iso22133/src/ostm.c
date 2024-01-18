#include "ostm.h"
#include <stdio.h>
#include <string.h>
#include <endian.h>
#include <errno.h>
#include "iso22133.h"

/*!
 * \brief encodeOSTMMessage Constructs an ISO OSTM message based on specified command
 * \param inputHeader data to create header with
 * \param command Command to send to object according to ::ObjectCommandType
 * \param ostmDataBuffer Data buffer to which OSTM is to be written
 * \param bufferLength Length of data buffer to which OSTM is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeOSTMMessage(
	const MessageHeaderType *inputHeader,
	const enum ObjectCommandType command,
	char *ostmDataBuffer,
	const size_t bufferLength,
	const char debug)
{

	OSTMType OSTMData;

	memset(ostmDataBuffer, 0, bufferLength);

	// Check so buffer can hold message
	if (bufferLength < sizeof (OSTMData)) {
		fprintf(stderr, "Buffer too small to hold necessary OSTM data\n");
		return -1;
	}

	// Check vs allowed commands
	if (!
		(command == OBJECT_COMMAND_ARM || command == OBJECT_COMMAND_DISARM
		 || command == OBJECT_COMMAND_REMOTE_CONTROL
		 || command == OBJECT_COMMAND_ALL_CLEAR)) {
		fprintf(stderr, "OSTM does not support command %u\n", (uint8_t) command);
		return -1;
	}

	// Construct header
	OSTMData.header = buildISOHeader(MESSAGE_ID_OSTM, inputHeader, sizeof (OSTMData), debug);

	// Fill contents
	OSTMData.stateValueID = VALUE_ID_OSTM_STATE_CHANGE_REQUEST;
	OSTMData.stateContentLength = sizeof (OSTMData.state);
	OSTMData.state = (uint8_t) command;

	if (debug) {
		printf("OSTM message:\n\tState change request value ID: 0x%x\n\t"
			   "State change request content length: %u\n\tState change request: %u\n",
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
 * \brief decodeOSTMMessage Decodes an ISO OSTM message.
 * \param ostmDataBuffer Buffer with data to be decoded.
 * \param bufferLength Length of OSTM data buffer.
 * \param command Decoded state change request.
 * \param debug Flag for enabling debugging.
 * \return Size of decoded message, or negative according to
 *		::ISOMessageReturnValue if an error occurred.
 */
ssize_t decodeOSTMMessage(
		const char* ostmDataBuffer,
		const size_t bufferLength,
		enum ObjectCommandType* command,
		const char debug) {

	OSTMType OSTMData;
	const char *p = ostmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (ostmDataBuffer == NULL || command == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to OSTM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&OSTMData, 0, sizeof (OSTMData));
	memset(command, 0, sizeof (*command));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &OSTMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (OSTMData.header);

	// If message is not a PODI message, generate an error
	if (OSTMData.header.messageID != MESSAGE_ID_OSTM) {
		fprintf(stderr, "Attempted to pass non-OSTM message into OSTM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (OSTMData.header.messageLength > sizeof (OSTMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "OSTM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - ostmDataBuffer < OSTMData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_OSTM_STATE_CHANGE_REQUEST:
			memcpy(&OSTMData.state, p, sizeof (OSTMData.state));
			OSTMData.stateValueID = valueID;
			OSTMData.stateContentLength = contentLength;
			expectedContentLength = sizeof (OSTMData.state);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known OSTM value IDs", valueID);
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
		 decodeISOFooter(p, bufferLength - (size_t) (p - ostmDataBuffer), &OSTMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding OSTM footer\n");
		return retval;
	}
	p += sizeof (OSTMData.footer);

	if ((retval = verifyChecksum(ostmDataBuffer, OSTMData.header.messageLength + sizeof (HeaderType),
								 OSTMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "OSTM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("OSTM message:\n");
		printf("\tRequested state value ID: 0x%x\n", OSTMData.stateValueID);
		printf("\tRequested state content length: %u\n", OSTMData.stateContentLength);
		printf("\tRequested state: %u\n", OSTMData.state);
	}

	*command = OSTMData.state;
	return retval < 0 ? retval : p - ostmDataBuffer;
}
