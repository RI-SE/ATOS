#include "iohelpers.h"
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <endian.h>

void printContent(
		const uint16_t valueID,
		const uint16_t contentLength,
		const void* value,
		DebugStrings_t* deb) {
	if(deb && deb->printer) {
		printf("\t%s value ID: 0x%x\n\t%s content length: %u\n",
			   deb->name, valueID,
			   deb->name, contentLength);
		printf("\t%s: ", deb->name);
		deb->printer(value);
		printf(" %s\n", deb->unit);
	}
}


/*!
 * \brief encodeContent Copies ISO content into a buffer and manipulates the related pointers
 *			and size variables.
 * \param valueID ValueID of the data
 * \param src Pointer to data to be copied
 * \param dest Reference to pointer where data is to be stored (incremented after data copy)
 * \param dataSize Size of content to be copied
 * \param bufferSpace Remaining buffer space in data buffer pointed to by dest
 * \return 0 on success, -1 otherwise
 */
int encodeContent(uint16_t valueID,
		const void* src,
		char** dest,
		const size_t contentSize,
		size_t* bufferSpace,
		DebugStrings_t *debugStruct,
		const char debug) {
	uint16_t contentLength = (uint16_t) contentSize;

	union {
		uint8_t  u8Data;
		uint16_t u16Data;
		uint32_t u32Data;
		uint64_t u64Data;
	} data;

	if (*bufferSpace < contentSize + sizeof (valueID)
			+ sizeof (contentLength)) {
		errno = ENOBUFS;
		return -1;
	}

	if(debug && debugStruct->printer) {
		printContent(valueID, contentLength, src, debugStruct);
	}
	
	valueID = htole16(valueID);
	memcpy(*dest, &valueID, sizeof (valueID));
	*dest += sizeof (valueID);
	*bufferSpace -= sizeof (valueID);

	contentLength = htole16(contentLength);
	memcpy(*dest, &contentLength, sizeof (contentLength));
	*dest += sizeof (contentLength);
	*bufferSpace -= sizeof (contentLength);

	switch (contentSize) {
	case sizeof (uint8_t):
		memcpy(&data.u8Data, src, contentSize);
		break;
	case sizeof (uint16_t):
		memcpy(&data.u16Data, src, contentSize);
		data.u16Data = htole16(data.u16Data);
		break;
	case sizeof (uint32_t):
		memcpy(&data.u32Data, src, contentSize);
		data.u32Data = htole32(data.u32Data);
		break;
	case sizeof (uint64_t):
		memcpy(&data.u64Data, src, contentSize);
		data.u64Data = htole64(data.u64Data);
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	memcpy(*dest, &data, contentSize);
	*dest += contentSize;
	*bufferSpace -= contentSize;
	return 0;
}


void printU8(const void* val)  { printf("%u",  *(const uint8_t*)val); }
void printU16(const void* val) { printf("%u",  *(const uint16_t*)val); }
void printU32(const void* val) { printf("%u",  *(const uint32_t*)val); }
void printU64(const void* val) { printf("%lu", *(const uint64_t*)val); }
void printI8(const void* val)  { printf("%d",  *(const int8_t*)val); }
void printI16(const void* val) { printf("%d",  *(const int16_t*)val); }
void printI32(const void* val) { printf("%d",  *(const int32_t*)val); }
void printI64(const void* val) { printf("%ld", *(const int64_t*)val); }
void printString(const void* val) { printf("%s", (const char*)val); }
