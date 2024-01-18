#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdint.h>
#include "iso22133.h"

#pragma pack(push, 1)
/*! ISO message footer */
typedef struct {
	uint16_t Crc;
} FooterType;
#pragma pack(pop)

enum ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length,
											 FooterType * HeaderData, const char debug);
FooterType buildISOFooter(const void *message, const size_t messageSize, const char debug);

uint16_t crcByte(const uint16_t crc, const uint8_t byte);
uint16_t crc16(const uint8_t * data, size_t dataLen);

enum ISOMessageReturnValue verifyChecksum(
		const void *data,
		const size_t dataLen,
		const uint16_t crc,
		const char debug);

#ifdef __cplusplus
}
#endif