#include "header.h"
#include "defines.h"
#include "footer.h"

#include <string.h>
#include <endian.h>
#include <stdio.h>

/*!
 * \brief buildISOHeader Constructs an ISO header based on the supplied message ID and content length
 * \param receiverID ID of the receiver of the message for which the header is to be used
 * \param id Message ID of the message for which the header is to be used
 * \param messageLength Length of the message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO header data
 */
HeaderType buildISOHeader(enum ISOMessageID id, const MessageHeaderType *input, uint32_t messageLength, const char debug) {
	HeaderType header;

	header.syncWord = ISO_SYNC_WORD;
	header.transmitterID = input->transmitterID;
	header.receiverID = input->receiverID;
	header.messageCounter = input->messageCounter;
	header.ackReqProtVer = ACK_REQ | ISO_PROTOCOL_VERSION;
	if (messageLength >= sizeof (HeaderType) + sizeof (FooterType)) {
		header.messageID = (uint16_t) id;
		header.messageLength = messageLength - sizeof (HeaderType) - sizeof (FooterType);
	}
	else {
		fprintf(stderr, "Supplied message length too small to hold header and footer\n");
		header.messageID = (uint16_t) MESSAGE_ID_INVALID;
		header.messageLength = 0;
	}

	if (debug) {
		printf("Encoded ISO header:\n\tSync word: 0x%x\n\tTransmitter ID: %u\n\tReceiver ID: %u\n\tMessage counter: %u\n\t"
			   "Ack request | Protocol version: 0x%x\n\tMessage ID: 0x%x\n\tMessage length: %u\n",
			   header.syncWord, header.transmitterID, header.receiverID, header.messageCounter, header.ackReqProtVer,
			   header.messageID, header.messageLength);
	}

	// Convert from host endianness to little endian
	header.syncWord = htole16(header.syncWord);
	header.messageID = htole16(header.messageID);
	header.transmitterID = htole32(header.transmitterID);
	header.receiverID = htole32(header.receiverID);
	header.messageLength = htole32(header.messageLength);

	return header;
}

/*!
 * \brief decodeISOHeader Convert data in a buffer to an ISO heade
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue decodeISOHeader(
	const char *MessageBuffer,
	const size_t length,
	HeaderType * HeaderData,
	const char debug) {

	const char *p = MessageBuffer;
	enum ISOMessageReturnValue retval = MESSAGE_OK;
	const char ProtocolVersionBitmask = 0x7F;
	char messageProtocolVersion = 0;
	char isProtocolVersionSupported = 0;

	// If not enough data to fill header, generate error
	if (length < sizeof (HeaderData)) {
		fprintf(stderr, "Too little raw data to fill ISO header\n");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode ISO header
	memcpy(&HeaderData->syncWord, p, sizeof (HeaderData->syncWord));
	HeaderData->syncWord = le16toh(HeaderData->syncWord);
	p += sizeof (HeaderData->syncWord);

	// If sync word is not correct, generate error
	if (HeaderData->syncWord != ISO_SYNC_WORD) {
		fprintf(stderr, "Sync word error when decoding ISO header (0x%04x)\n",
				HeaderData->syncWord);
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_SYNC_WORD_ERROR;
	}

	memcpy(&HeaderData->messageLength, p, sizeof (HeaderData->messageLength));
	p += sizeof (HeaderData->messageLength);
	HeaderData->messageLength = le32toh(HeaderData->messageLength);

	memcpy(&HeaderData->ackReqProtVer, p, sizeof (HeaderData->ackReqProtVer));
	p += sizeof (HeaderData->ackReqProtVer);

	// Loop over permitted protocol versions to see if current version is among them
	messageProtocolVersion = HeaderData->ackReqProtVer & ProtocolVersionBitmask;
	for (size_t i = 0; i < sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]); ++i) {
		if (SupportedProtocolVersions[i] == messageProtocolVersion) {
			isProtocolVersionSupported = 1;
			break;
		}
	}

	// Generate error if protocol version not supported
	if (!isProtocolVersionSupported) {
		fprintf(stderr, "Protocol version %u not supported\n", messageProtocolVersion);
		retval = MESSAGE_VERSION_ERROR;
		memset(HeaderData, 0, sizeof (*HeaderData));
		return retval;
	}

	memcpy(&HeaderData->transmitterID, p, sizeof (HeaderData->transmitterID));
	p += sizeof (HeaderData->transmitterID);
	HeaderData->transmitterID = le32toh(HeaderData->transmitterID);

	memcpy(&HeaderData->receiverID, p, sizeof (HeaderData->receiverID));
	p += sizeof (HeaderData->receiverID);
	HeaderData->receiverID = le32toh(HeaderData->receiverID);

	memcpy(&HeaderData->messageCounter, p, sizeof (HeaderData->messageCounter));
	p += sizeof (HeaderData->messageCounter);

	memcpy(&HeaderData->messageID, p, sizeof (HeaderData->messageID));
	p += sizeof (HeaderData->messageID);
	HeaderData->messageID = le16toh(HeaderData->messageID);

	if (debug) {
		printf("syncWord = 0x%x\n", HeaderData->syncWord);
		printf("messageLength = 0x%x\n", HeaderData->messageLength);
		printf("ackReqProtVer = 0x%x\n", HeaderData->ackReqProtVer);
		printf("transmitterID = 0x%x\n", HeaderData->transmitterID);
		printf("receiverID = 0x%x\n", HeaderData->receiverID);
		printf("messageCounter = 0x%x\n", HeaderData->messageCounter);
		printf("messageID = 0x%x\n", HeaderData->messageID);
	}

	return retval;
}