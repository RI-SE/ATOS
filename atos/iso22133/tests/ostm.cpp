#include "iso22133.h"
#include "header.h"

#include <gtest/gtest.h>

class EncodeOSTM : public ::testing::Test
{
protected:
	EncodeOSTM()
	{
	}

	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		MessageHeaderType inputHeader;
		inputHeader.receiverID = 0;
		inputHeader.messageCounter = 0;
		inputHeader.transmitterID = 0;
		auto res = encodeOSTMMessage(
			&inputHeader,
			OBJECT_COMMAND_ARM,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	char encodeBuffer[1024];
	char* ostm = encodeBuffer + 18;
};

TEST_F(EncodeOSTM, Command)
{
	EXPECT_EQ(ostm[0], '\x64');
	EXPECT_EQ(ostm[1], '\x00');
	EXPECT_EQ(ostm[2], '\x01');
	EXPECT_EQ(ostm[3], '\x00');
	EXPECT_EQ(ostm[4], '\x02');
}


class DecodeOSTM : public ::testing::Test
{
protected:
	DecodeOSTM()
	{
		decodeBuffer[0] = 0x7F;	// preamble
		decodeBuffer[1] = 0x7E;
		decodeBuffer[2] = 0x05;
		decodeBuffer[3] = 0x00;
		decodeBuffer[4] = 0x00;
		decodeBuffer[5] = 0x00;	 // Message length
		decodeBuffer[6] = 0x02;	 // Acknowledge protocol version
		decodeBuffer[7] = 0x34;
		decodeBuffer[8] = 0x12;
		decodeBuffer[9] = 0x00;
		decodeBuffer[10] = 0x00;	 // Transmitter ID
		decodeBuffer[11] = 0x78;
		decodeBuffer[12] = 0x56;
		decodeBuffer[13] = 0x00;
		decodeBuffer[14] = 0x00;	 // Receiver ID
		decodeBuffer[15] = 0x00;	 // Message count
		decodeBuffer[16] = 0x03;
		decodeBuffer[17] = 0x00;	 // Message ID
		decodeBuffer[18] = 0x64;
		decodeBuffer[19] = 0x00;
		decodeBuffer[20] = 0x01;
		decodeBuffer[21] = 0x00;
		decodeBuffer[22] = 0x02;
		decodeBuffer[23] = 0xB2;
		decodeBuffer[24] = 0xD7;
	}

	void SetUp() override
	{
		auto res = decodeOSTMMessage(
			decodeBuffer,
			sizeof(decodeBuffer),
			&cmd,
			false);
		ASSERT_GT(res, 0);
	}
	char decodeBuffer[1024];
	ObjectCommandType cmd;
};

TEST_F(DecodeOSTM, Command)
{
	EXPECT_EQ(cmd, OBJECT_COMMAND_ARM);
}
