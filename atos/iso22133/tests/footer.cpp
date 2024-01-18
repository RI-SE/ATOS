#include <gtest/gtest.h>
extern "C" {
#include "footer.h"
}

TEST(FooterDecode, Crc) {
	char message[2];
	*reinterpret_cast<uint16_t*>(message) = htole16(0x1234);
	FooterType footer;
	auto ret = decodeISOFooter(message, 2, &footer, false);
	ASSERT_EQ(MESSAGE_OK, ret);
	EXPECT_EQ(0x1234, footer.Crc);
}

// https://crccalc.com/?crc=0102030405101112131415&method=CRC-16/XMODEM&datatype=hex&outtype=0
TEST(FooterEncode, Crc) {
	char data[] = {
		'\x01',
		'\x02',
		'\x03',
		'\x04',
		'\x05',
		'\x10',
		'\x11',
		'\x12',
		'\x13',
		'\x14',
		'\x15'
	};
	auto res = crc16(reinterpret_cast<uint8_t*>(data), sizeof(data));
	EXPECT_EQ(res, 0x881D);
}

TEST(FooterEncode, HeaderCrc) {
	char data[] = {
		'\x7f',
		 '\x7e',
		 '\xbb',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x02',
		 '\xff',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x00',
		 '\x01',
		 '\x00'
	};
	auto res = crc16(reinterpret_cast<uint8_t*>(data), sizeof(data));
	EXPECT_EQ(res, 0x5A29);
}

TEST(FooterEncode, TrajPointCRC) {
	char data[] = {
		'\x01',
		'\x00',
		'\x1e',
		'\x00',
		'\xe8',
		'\x03',
		'\x00',
		'\x00',
		'\xe8',
		'\x03',
		'\x00',
		'\x00',
		'\xd0',
		'\x07',
		'\x00',
		'\x00',
		'\xb8',
		'\x0b',
		'\x00',
		'\x00',
		'\x86',
		'\x59',
		'\x64',
		'\x00',
		'\xc8',
		'\x00',
		'\xe8',
		'\x03',
		'\xd0',
		'\x07',
		'\xa4',
		'\x70',
		'\x45',
		'\x41'
	};
	auto res = crc16(reinterpret_cast<uint8_t*>(data), sizeof(data));
	EXPECT_EQ(res, 0x7484);
}
