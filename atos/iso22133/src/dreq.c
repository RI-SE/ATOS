#include <stdio.h>
#include <string.h>
#include <endian.h>
#include <errno.h>
#include "iso22133.h"
#include "dreq.h"
#include "iohelpers.h"

/*!
 * \brief encodeDREQMessage Constructs an ISO DREQ message based on specified command (DREQ contains no message data)
 * \param inputHeader data to create header
 * \param dreqDataBuffer Data buffer to which DREQ is to be written
 * \param bufferLength Length of data buffer to which DREQ is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeDREQMessage(const MessageHeaderType *inputHeader,
	char *dreqDataBuffer,
	const size_t bufferLength,
	const char debug)
{
	DREQType DREQData;
	memset(dreqDataBuffer, 0, bufferLength);

	// Check so buffer can hold message
	if (bufferLength < sizeof (DREQData)) {
		fprintf(stderr, "Buffer too small to hold necessary DREQ data\n");
		return -1;
	}

	// Construct header
	DREQData.header = buildISOHeader(MESSAGE_ID_DREQ, inputHeader, sizeof (DREQData), debug);

	if (debug) {
        printf("DREQ data: No data in DREQ message, just header and footer\n");
    }

	// Construct footer
	DREQData.footer = buildISOFooter(&DREQData, sizeof (DREQData), debug);
	memcpy(dreqDataBuffer, &DREQData, sizeof (DREQData));

	return sizeof (DREQData);
}

/*!
 * \brief decodeDREQMessage Decodes an ISO DREQ message.
 * \param dreqDataBuffer Buffer with data to be decoded.
 * \param bufferLength Length of DRES data buffer.
 * \param command Decoded state change request.
 * \param debug Flag for enabling debugging.
 * \return Size of decoded message, or negative according to
 *		::ISOMessageReturnValue if an error occurred.
 */

ssize_t decodeDREQMessage(
		const char* dreqDataBuffer,
		const size_t bufferLength,
		TestObjectDiscoveryRequestType *testObjectDiscoveryRequestData,
		const char debug) {

	DREQType DREQData;
	const char *p = dreqDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (dreqDataBuffer == NULL || testObjectDiscoveryRequestData == NULL) {
	 	errno = EINVAL;
	 	fprintf(stderr, "Input pointers to DREQ parsing function cannot be null\n");
	 	return ISO_FUNCTION_ERROR;
	}

	memset(&DREQData, 0, sizeof (DREQData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DREQData.header, debug)) != MESSAGE_OK) {
	    return retval;
	}
	p += sizeof (DREQData.header);

	// If message is not a DREQ message, generate an error
	if (DREQData.header.messageID != MESSAGE_ID_DREQ) {
	 	fprintf(stderr, "Attempted to pass non-DREQ message into DREQ parsing function\n");
		testObjectDiscoveryRequestData->requestStatus = DREQ_NOT_RECEIVED;
	 	return MESSAGE_TYPE_ERROR;
	} else {
		testObjectDiscoveryRequestData->requestStatus = DREQ_RECEIVED;
	}

	// Decode footer
	if ((retval =
	 	 decodeISOFooter(p, bufferLength - (size_t) (p - dreqDataBuffer), &DREQData.footer,
	 					 debug)) != MESSAGE_OK) {
	 	fprintf(stderr, "Error decoding DREQ footer\n");
	 	return retval;
	 }
	 p += sizeof (DREQData.footer);

	if ((retval = verifyChecksum(dreqDataBuffer, DREQData.header.messageLength + sizeof (HeaderType),
	 							 DREQData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
	 	fprintf(stderr, "DREQ checksum error\n");
	 	return retval;
	}

    if (debug) {
        printf("DREQ data: Contains no data\n");
	 }

	return retval < 0 ? retval : p - dreqDataBuffer;
}

