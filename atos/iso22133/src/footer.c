#include "footer.h"
#include "defines.h"

#include <string.h>
#include <endian.h>
#include <stdio.h>


static int8_t isCRCVerificationEnabled = DEFAULT_CRC_CHECK_ENABLED;

/*!
 * \brief buildISOFooter Constructs a footer for an ISO message
 * \param message Pointer to start of message header
 * \param messageSize Size of the entire message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO footer data
 */
FooterType buildISOFooter(const void *message, const size_t messageSize, const char debug) {
	FooterType footer;

	// Calculate CRC - remembering that message begins with header and messageSize will include header and footer
	footer.Crc = crc16(message, messageSize - sizeof (FooterType));

	if (debug) {
		printf("Encoded ISO footer:\n\tCRC: 0x%x\n", footer.Crc);
	}

	footer.Crc = htole16(footer.Crc);

	return footer;
}


/*!
 * \brief decodeISOFooter Convert data in a buffer to an ISO footer
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length, FooterType * FooterData,
									  const char debug) {

	// If too little data, generate error
	if (length < sizeof (FooterData->Crc)) {
		fprintf(stderr, "Too little raw data to fill ISO footer\n");
		memset(FooterData, 0, sizeof (*FooterData));
		return MESSAGE_LENGTH_ERROR;
	}
	memcpy(&FooterData->Crc, MessageBuffer, sizeof (FooterData->Crc));
	FooterData->Crc = le16toh(FooterData->Crc);

	if (debug) {
		printf("Decoded ISO footer:\n\tCRC: 0x%x\n", FooterData->Crc);
	}

	return MESSAGE_OK;
}


/*!
 * \brief crc16 Calculates the 16 bit CCITT checksum value for the polynomial
 *				x^16 + x^12 + x^5 + 1
 * \param data Block of data for which CRC is to be calculated
 * \param dataLen Length of the block of data
 * \return CRC checksum
 */
uint16_t crc16(const uint8_t * data, size_t dataLen) {
	uint16_t crc = DEFAULT_CRC_INIT_VALUE;

	while (dataLen-- > 0) {
		crc = crcByte(crc, *data++);
	}
	return crc;
}


/*!
 * \brief crcByte Updates the given CRC based on an input byte from data
 * \param crc CRC from previous byte
 * \param byte New data byte
 * \return New CRC value
 */
uint16_t crcByte(const uint16_t crc, const uint8_t byte) {
	static const uint16_t crcTable[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
	};

	return (uint16_t) ((crc << 8) ^ crcTable[(crc >> 8) ^ byte]);
}

/*!
 * \brief verifyChecksum Generates a checksum for specified data and checks if it matches against
 *			the specified CRC. If the specified CRC is 0, the message does not contain a CRC value
 *			and the message is assumed uncorrupted.
 * \param data Data for which checksum is to be verified
 * \param dataLen Length of the data
 * \param CRC Received CRC value for the data
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue verifyChecksum(
	const void *data,
	const size_t dataLen,
	const uint16_t CRC,
	const char debug)
{
	if (!isCRCVerificationEnabled || CRC == 0) {
		return MESSAGE_OK;
	}

	const uint16_t dataCRC = crc16(data, dataLen);

	if (debug) {
		printf("CRC given: %04x, CRC calculated: %04x\n", CRC, dataCRC);
		printf("Data, %ld bytes: ", dataLen);
		for (size_t i = 0; i < dataLen; i++) {
			printf("%02x ", ((uint8_t *) data)[i]);
		}
	}
	return dataCRC == CRC ? MESSAGE_OK : MESSAGE_CRC_ERROR;
}

/*!
 * \brief setISOCRCVerification Enables or disables checksum verification on received messages (default
 *			is to enable checksum verification)
 * \param enabled Boolean for enabling or disabling the checksum verification
 */
void setISOCRCVerification(const int8_t enabled) {
	isCRCVerificationEnabled = enabled ? 1 : 0;
	return;
}
