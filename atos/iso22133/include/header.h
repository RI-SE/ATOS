#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include "iso22133.h"

#pragma pack(push,1)			// Ensure sizeof() is useable for (most) network byte lengths
/*! ISO message header */
typedef struct {
	uint16_t syncWord;
	uint32_t messageLength;
	uint8_t ackReqProtVer;
	uint32_t transmitterID;
	uint32_t receiverID;
	uint8_t messageCounter;
	uint16_t messageID;
} HeaderType;;
#pragma pack(pop)

enum ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);

HeaderType buildISOHeader(enum ISOMessageID id, const MessageHeaderType *input, uint32_t messageLength, const char debug);

#ifdef __cplusplus
}
#endif