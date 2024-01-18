#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "iso22133.h"
#include "header.h"
#include "footer.h"
#include <stdint.h>

#pragma pack(push, 1)

/*! DRES message */
typedef struct {
	HeaderType header;
	uint16_t vendorNameValueID;
	uint16_t vendorNameContentLength;
	uint8_t vendorName[64];
	uint16_t productNameValueID;
	uint16_t productNameContentLength;
	uint8_t productName[64];
	uint16_t firmwareVersionValueID;
	uint16_t firmwareVersionContentLength;
	uint8_t firmwareVersion[64];
	uint16_t testObjectNameValueID;
	uint16_t testObjectNameContentLength;
	uint8_t testObjectName[64];
	uint16_t testObjectTypeValueID;
	uint16_t testObjectTypeContentLength;
	uint8_t testObjectType;
	uint16_t subDeviceIdTypeValueID;
	uint16_t subDeviceIdTypeContentLength;
	uint32_t subDeviceId;
	FooterType footer;
} DRESType;						//Max 304 bytes

#pragma pack(pop)

//! DRES value IDs
#define VALUE_ID_VENDOR_NAME 0x0090
#define VALUE_ID_PRODUCT_NAME 0x0091
#define VALUE_ID_FIRMWARE_VERSION 0x0092
#define VALUE_ID_TEST_OBJECT_NAME 0x0093
#define VALUE_ID_TEST_OBJECT_TYPE 0x0094
#define VALUE_ID_SUB_DEVICE_ID 0x0095

void convertDRESToHostRepresentation(
		const DRESType * DRESData,
		TestObjectDiscoveryType * testObjectDiscoveryData);

#ifdef __cplusplus
}
#endif