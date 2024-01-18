#include <stdio.h>
#include <string.h>
#include <endian.h>
#include <errno.h>
#include "iso22133.h"
#include "dres.h"
#include "iohelpers.h"

/*!
 * \brief encodeDRESMessage Constructs an ISO DRES message based on specified command
 * \param inputHeader data to create header with
 * \param testObjectDiscoveryData Test object discovery data
 * \param dresDataBuffer Data buffer to which DRES is to be written
 * \param bufferLength Length of data buffer to which DRES is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeDRESMessage(
	const MessageHeaderType *inputHeader,
	const TestObjectDiscoveryType *testObjectDiscoveryData,
	char *dresDataBuffer,
	const size_t bufferLength,
	const char debug)
{

	DRESType DRESData;

	memset(dresDataBuffer, 0, bufferLength);

	// Check so buffer can hold message
	if (bufferLength < sizeof (DRESData)) {
		fprintf(stderr, "Buffer too small to hold necessary DRES data\n");
		return -1;
	}

	// Construct header
	DRESData.header = buildISOHeader(MESSAGE_ID_DRES, inputHeader, sizeof (DRESData), debug);

	// Fill contents
    DRESData.vendorNameValueID = VALUE_ID_VENDOR_NAME;
    DRESData.vendorNameContentLength = sizeof(DRESData.vendorName);
    memset(DRESData.vendorName, 0, strlen(DRESData.vendorName));
	memcpy(DRESData.vendorName, &testObjectDiscoveryData->vendor, strlen(testObjectDiscoveryData->vendor));
    DRESData.vendorNameValueID = htole16(DRESData.vendorNameValueID);
    DRESData.vendorNameContentLength = htole16(DRESData.vendorNameContentLength);

    DRESData.productNameValueID = VALUE_ID_PRODUCT_NAME;
    DRESData.productNameContentLength = sizeof(DRESData.productName);
    memset(DRESData.productName, 0, strlen(DRESData.productName));
	memcpy(DRESData.productName, &testObjectDiscoveryData->productName, strlen(testObjectDiscoveryData->productName));
    DRESData.productNameValueID = htole16(DRESData.productNameValueID);
    DRESData.productNameContentLength = htole16(DRESData.productNameContentLength);

    DRESData.firmwareVersionValueID = VALUE_ID_FIRMWARE_VERSION;
    DRESData.firmwareVersionContentLength = sizeof(DRESData.firmwareVersionContentLength);
    memset(DRESData.firmwareVersion, 0, strlen(DRESData.firmwareVersion));
	memcpy(DRESData.firmwareVersion, &testObjectDiscoveryData->firmwareVersion, strlen(testObjectDiscoveryData->firmwareVersion));
    DRESData.firmwareVersionValueID = htole16(DRESData.firmwareVersionValueID);
    DRESData.firmwareVersionContentLength = htole16(DRESData.firmwareVersionContentLength);

    DRESData.testObjectNameValueID = VALUE_ID_TEST_OBJECT_NAME;
    DRESData.testObjectNameContentLength = sizeof(DRESData.testObjectNameContentLength);
    memset(DRESData.testObjectName, 0, strlen(DRESData.testObjectName));
	memcpy(DRESData.testObjectName, &testObjectDiscoveryData->testObjectName, strlen(testObjectDiscoveryData->testObjectName));
    DRESData.testObjectNameValueID = htole16(DRESData.testObjectNameValueID);
    DRESData.testObjectNameContentLength = htole16(DRESData.testObjectNameContentLength);

    DRESData.testObjectTypeValueID = VALUE_ID_TEST_OBJECT_TYPE;
    DRESData.testObjectTypeContentLength = sizeof(DRESData.testObjectType);
    DRESData.testObjectType = testObjectDiscoveryData->testObjectTypeCode;
    DRESData.testObjectTypeValueID = htole16(DRESData.testObjectTypeValueID);
    DRESData.testObjectTypeContentLength = htole16(DRESData.testObjectTypeContentLength);

    DRESData.subDeviceIdTypeValueID = VALUE_ID_SUB_DEVICE_ID;
    DRESData.subDeviceIdTypeContentLength = sizeof(DRESData.subDeviceIdTypeContentLength);
    DRESData.subDeviceId = testObjectDiscoveryData->subDeviceId;
    DRESData.subDeviceIdTypeValueID = htole16(DRESData.subDeviceIdTypeValueID);
    DRESData.subDeviceIdTypeContentLength = htole16(DRESData.subDeviceIdTypeContentLength);

	if (debug) {
        printf("DRES data:\n");
		printf("\tVendor name value ID: 0x%x\n", DRESData.vendorNameValueID);
		printf("\tVendor name content length: %d bytes\n", DRESData.vendorNameContentLength);
		printf("\tVendor name: %s\n", DRESData.vendorName);
   		printf("\tProduct name value ID: 0x%x\n", DRESData.productNameValueID);
		printf("\tProduct name content length: %d bytes\n", DRESData.productNameContentLength);
		printf("\tProduct name: %s\n", DRESData.productName);
   		printf("\tFirmware version value ID: 0x%x\n", DRESData.firmwareVersionValueID);
		printf("\tFirmware version content length: %d bytes\n", DRESData.firmwareVersionContentLength);
		printf("\tFirmware version: %s\n", DRESData.firmwareVersion);
   		printf("\tTest object name value ID: 0x%x\n", DRESData.testObjectNameValueID);
		printf("\tTest object name content length: %d bytes\n", DRESData.testObjectNameContentLength);
		printf("\tTest object name: %s\n", DRESData.testObjectName);
   		printf("\tTest object type value ID: 0x%x\n", DRESData.testObjectTypeValueID);
		printf("\tTest object type content length: %d bytes\n", DRESData.testObjectTypeContentLength);
		printf("\tTest object type: %d\n", DRESData.testObjectType);
   		printf("\tSub device ID value ID: 0x%x\n", DRESData.subDeviceIdTypeValueID);
		printf("\tSub device ID content length: %d bytes\n", DRESData.subDeviceIdTypeContentLength);
		printf("\tSub device ID type: %d\n", DRESData.subDeviceId);
    }

	// Construct footer
	DRESData.footer = buildISOFooter(&DRESData, sizeof (DRESData), debug);
	memcpy(dresDataBuffer, &DRESData, sizeof (DRESData));

	return sizeof (DRESData);
}

/*!
 * \brief decodeDRESMessage Decodes an ISO DRES message.
 * \param dresDataBuffer Buffer with data to be decoded.
 * \param bufferLength Length of DRES data buffer.
 * \param command Decoded state change request.
 * \param debug Flag for enabling debugging.
 * \return Size of decoded message, or negative according to
 *		::ISOMessageReturnValue if an error occurred.
 */

ssize_t decodeDRESMessage(
		const char* dresDataBuffer,
		const size_t bufferLength,
		TestObjectDiscoveryType *testObjectDiscoveryData,
		const char debug) {

	DRESType DRESData;
	const char *p = dresDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (dresDataBuffer == NULL || testObjectDiscoveryData == NULL) {
	 	errno = EINVAL;
	 	fprintf(stderr, "Input pointers to OSTM parsing function cannot be null\n");
	 	return ISO_FUNCTION_ERROR;
	}

	memset(&DRESData, 0, sizeof (DRESData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DRESData.header, debug)) != MESSAGE_OK) {
	    return retval;
	}
	p += sizeof (DRESData.header);

	// If message is not a DRES message, generate an error
	if (DRESData.header.messageID != MESSAGE_ID_DRES) {
	 	fprintf(stderr, "Attempted to pass non-DRES message into DRES parsing function\n");
	 	return MESSAGE_TYPE_ERROR;
	 }

	if (DRESData.header.messageLength > sizeof (DRESType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "DRES message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - dresDataBuffer < DRESData.header.messageLength + sizeof (HeaderType)) {
		// Decode value ID and length
        memset(&valueID, 0, sizeof (valueID));
        memcpy(&valueID, p, sizeof (valueID));
	 	valueID = le16toh(valueID);
		p += sizeof (valueID);
        memset(&contentLength, 0, sizeof (contentLength));
	 	memcpy(&contentLength, p, sizeof (contentLength));
	 	contentLength = le16toh(contentLength);
		p += sizeof (contentLength);
        // Handle contents
	 	switch (valueID) {
	 	case VALUE_ID_VENDOR_NAME:
	 		DRESData.vendorNameValueID = valueID;
	 		DRESData.vendorNameContentLength = contentLength;
	 		memcpy(&DRESData.vendorName, p, sizeof (DRESData.vendorName));
	 		expectedContentLength = sizeof (DRESData.vendorName);
	 		break;
	 	case VALUE_ID_PRODUCT_NAME:
	 		DRESData.productNameValueID = valueID;
	 		DRESData.productNameContentLength = contentLength;
	 		memcpy(&DRESData.vendorName, p, sizeof (DRESData.productName));
	 		expectedContentLength = sizeof (DRESData.productName);
	 		break;
	 	case VALUE_ID_FIRMWARE_VERSION:
	 		DRESData.firmwareVersionValueID = valueID;
	 		DRESData.firmwareVersionContentLength = contentLength;
	 		memcpy(&DRESData.firmwareVersion, p, sizeof (DRESData.firmwareVersion));
	 		expectedContentLength = sizeof (DRESData.firmwareVersion);
	 		break;
	 	case VALUE_ID_TEST_OBJECT_NAME:
	 		DRESData.testObjectNameValueID = valueID;
	 		DRESData.testObjectNameContentLength = contentLength;
	 		memcpy(&DRESData.testObjectName, p, sizeof (DRESData.testObjectName));
	 		expectedContentLength = sizeof (DRESData.testObjectName);
	 		break;
	 	case VALUE_ID_TEST_OBJECT_TYPE:
	 		DRESData.testObjectTypeValueID = valueID;
	 		DRESData.testObjectTypeContentLength = contentLength;
	 		memcpy(&DRESData.testObjectType, p, sizeof (DRESData.testObjectType));
	 		expectedContentLength = sizeof (DRESData.testObjectType);
	 		break;
	 	case VALUE_ID_SUB_DEVICE_ID:
	 		DRESData.subDeviceIdTypeValueID = valueID;
	 		DRESData.subDeviceIdTypeContentLength = contentLength;
	 		memcpy(&DRESData.subDeviceId, p, sizeof (DRESData.subDeviceId));
            DRESData.subDeviceId = le32toh(DRESData.subDeviceId);
	 		expectedContentLength = sizeof (DRESData.subDeviceId);
	 		break;
	 	default:
	 		fprintf(stderr, "Value ID 0x%x does not match any known DRES value IDs", valueID);
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
	 	 decodeISOFooter(p, bufferLength - (size_t) (p - dresDataBuffer), &DRESData.footer,
	 					 debug)) != MESSAGE_OK) {
	 	fprintf(stderr, "Error decoding DRES footer\n");
	 	return retval;
	 }
	 p += sizeof (DRESData.footer);

	if ((retval = verifyChecksum(dresDataBuffer, DRESData.header.messageLength + sizeof (HeaderType),
	 							 DRESData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
	 	fprintf(stderr, "DRES checksum error\n");
	 	return retval;
	}

    if (debug) {
        printf("DRES data:\n");
		printf("\tVendor name value ID: 0x%x\n", DRESData.vendorNameValueID);
		printf("\tVendor name content length: %d bytes\n", DRESData.vendorNameContentLength);
		printf("\tVendor name: %s\n", DRESData.vendorName);
   		printf("\tProduct name value ID: 0x%x\n", DRESData.productNameValueID);
		printf("\tProduct name content length: %d bytes\n", DRESData.productNameContentLength);
		printf("\tProduct name: %s\n", DRESData.productName);
   		printf("\tFirmware version value ID: 0x%x\n", DRESData.firmwareVersionValueID);
		printf("\tFirmware version content length: %d bytes\n", DRESData.firmwareVersionContentLength);
		printf("\tFirmware version: %s\n", DRESData.firmwareVersion);
   		printf("\tTest object name value ID: 0x%x\n", DRESData.testObjectNameValueID);
		printf("\tTest object name content length: %d bytes\n", DRESData.testObjectNameContentLength);
		printf("\tTest object name: %s\n", DRESData.testObjectName);
   		printf("\tTest object type value ID: 0x%x\n", DRESData.testObjectTypeValueID);
		printf("\tTest object type content length: %d bytes\n", DRESData.testObjectTypeContentLength);
		printf("\tTest object type: %d\n", DRESData.testObjectType);
   		printf("\tSub device ID value ID: 0x%x\n", DRESData.subDeviceIdTypeValueID);
		printf("\tSub device ID content length: %d bytes\n", DRESData.subDeviceIdTypeContentLength);
		printf("\tSub device ID type: %d\n", DRESData.subDeviceId);
	 }

    // Fill output struct with parsed data
	convertDRESToHostRepresentation(&DRESData, testObjectDiscoveryData);
	return retval < 0 ? retval : p - dresDataBuffer;
}


/*!
 * \brief Maps values from DRESType struct to custom data struct TestObjectDiscoveryType where values can later be used
 * \param DRESData DRESType struct with decoded DRES values from server
 * \param testObjectDiscoveryData Custom struct TestObjectDiscoveryType output values needed later in other functions
 */
void convertDRESToHostRepresentation(
	const DRESType * DRESData,
	TestObjectDiscoveryType * testObjectDiscoveryData)
{
	memcpy(testObjectDiscoveryData->vendor, DRESData->vendorName, sizeof(DRESData->vendorName));
	memcpy(testObjectDiscoveryData->productName, DRESData->productName, sizeof(DRESData->productName));
    memcpy(testObjectDiscoveryData->firmwareVersion, DRESData->firmwareVersion, sizeof(DRESData->firmwareVersion));
    memcpy(testObjectDiscoveryData->testObjectName, DRESData->testObjectName, sizeof(DRESData->testObjectName));
    testObjectDiscoveryData->testObjectTypeCode = DRESData->testObjectType;
    testObjectDiscoveryData->subDeviceId = DRESData->subDeviceId;
}

