#include <gtest/gtest.h>
extern "C"
{
#include "grem.h"
#include "iso22133.h"
}
#include <string>
#include <sstream>
#include "testdefines.h"

class EncodeGREM : public ::testing::Test
{
protected:
	EncodeGREM()
	{
		// GREM
		grem.responseCode = GREM_GENERAL_ERROR; // GREM_GENERAL_ERROR
	}
	void SetUp() override
	{
		memset(&encodeBuffer, 0, sizeof(encodeBuffer));
		
		memset(&header, 0, sizeof(header));
		header.receiverID = 0;
		header.messageCounter = TEST_MESSAGE_COUNTER_15;
		header.transmitterID = TEST_TRANSMITTER_ID_1;
		auto res = encodeGREMMessage(&header, &grem, encodeBuffer,
									 sizeof(encodeBuffer), true);
		ASSERT_GT(res, 0);
	}
	GeneralResponseMessageType grem;
	MessageHeaderType header;
	char encodeBuffer[1024];
};

TEST_F(EncodeGREM, ResponseCode)
{
	EXPECT_EQ(encodeBuffer[41], '\x02');
}

class DecodeGREM : public ::testing::Test
{
protected:
	DecodeGREM()
	{
		// HEADER - 18 Bytes
		decodeBuffer[0] = 0x7F; // SYNC Word 0x7E7F
		decodeBuffer[1] = 0x7E;
		decodeBuffer[2] = 0x22; // Message length 0x00000022 (34) bytes
		decodeBuffer[3] = 0x00;
		decodeBuffer[4] = 0x00;
		decodeBuffer[5] = 0x00;
		decodeBuffer[6] = 0x02; // Ack Request = 0 (bit 7), Acknowledge protocol version = 2 (bit 0-6)
		decodeBuffer[7] = 0x12; // Transmitter ID 0x00003412
		decodeBuffer[8] = 0x34;
		decodeBuffer[9] = 0x00;
		decodeBuffer[10] = 0x00;
		decodeBuffer[11] = 0x00; // Receiver ID 0x00000000
		decodeBuffer[12] = 0x00;
		decodeBuffer[13] = 0x00;
		decodeBuffer[14] = 0x00;
		decodeBuffer[15] = 0x00; // Message count, 1 byte
		decodeBuffer[16] = 0x18; // Message ID, 0x0018 (GREM)
		decodeBuffer[17] = 0x00;

		// Message Content
		decodeBuffer[18] = 0x00; // ValueID 0x0200
		decodeBuffer[19] = 0x02;
		decodeBuffer[20] = 0x04; // Content length, 0x004 bytes
		decodeBuffer[21] = 0x00;
		decodeBuffer[22] = 0x34; // ReceivedHeader Transmitter 0x34120000
		decodeBuffer[23] = 0x12;
		decodeBuffer[24] = 0x00;
		decodeBuffer[25] = 0x00;
		decodeBuffer[26] = 0x01; // ValueID 0x0201
		decodeBuffer[27] = 0x02;
		decodeBuffer[28] = 0x01; // Content length, 0x001 byte
		decodeBuffer[29] = 0x00;
		decodeBuffer[30] = 0x00; // ReceivedHeader Message Counter
		decodeBuffer[31] = 0x02; // ValueID	0x0202
		decodeBuffer[32] = 0x02;
		decodeBuffer[33] = 0x02; // Content length, 0x002 bytes
		decodeBuffer[34] = 0x00;
		decodeBuffer[35] = 0x18; // ReceivedHeader Message ID 0x0018
		decodeBuffer[36] = 0x00;
		decodeBuffer[37] = 0x03; // ValueID	0x0203
		decodeBuffer[38] = 0x02;
		decodeBuffer[39] = 0x01; // Content length, 0x001 bytes
		decodeBuffer[40] = 0x00;
		decodeBuffer[41] = 0x02; // Response Code General Error 0x02
		decodeBuffer[42] = 0x04; // ValueID 0x0204
		decodeBuffer[43] = 0x02;
		decodeBuffer[44] = 0x02; // Content length, 2 bytes
		decodeBuffer[45] = 0x00;
		decodeBuffer[46] = 0x00; // Payload Length - 0x0000 bytes
		decodeBuffer[47] = 0x00;
		decodeBuffer[48] = 0x05; // ValueID 0x0205
		decodeBuffer[49] = 0x02;
		decodeBuffer[50] = 0x00; // Content length, 0 bytes
		decodeBuffer[51] = 0x00;
		// decodeBuffer[xx] = 0x00; // PayloadData  Contains no data

		// FOOTER - 2 bytes
		decodeBuffer[52] = 0x00; // CRC
		decodeBuffer[53] = 0x00;
	}
	virtual void SetUp()
	{
		memset(&grem, 0, sizeof(grem));
		decodeGREMMessage(decodeBuffer, sizeof(decodeBuffer), &grem, true);
	}
	char decodeBuffer[1024];
	GeneralResponseMessageType grem;
};

TEST_F(DecodeGREM, ResponceCode)
{
	EXPECT_EQ(grem.responseCode, GREM_GENERAL_ERROR);
}
