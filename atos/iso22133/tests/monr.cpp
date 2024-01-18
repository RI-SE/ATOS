#include "monr.h"

#include <gtest/gtest.h>

class EncodeMONR : public ::testing::Test {
protected:
	EncodeMONR()
	{
		// Friday, April 29, 2022 2:22:22 AM
		objTime.tv_sec = 1651198942;
		objTime.tv_usec = 0;
		
		pos.xCoord_m = 1.0;
		pos.yCoord_m = -2.0;
		pos.zCoord_m = 3.0;
		pos.isXcoordValid = true;
		pos.isYcoordValid = true;
		pos.isZcoordValid = true;
		pos.isPositionValid = true;
		pos.heading_rad = 0.4;
		pos.isHeadingValid = true;

		spd.longitudinal_m_s = 1.0;
		spd.lateral_m_s = 2.0;
		spd.isLongitudinalValid = true;
		spd.isLateralValid = true;

		acc.longitudinal_m_s2 = 1.0;
		acc.lateral_m_s2 = 2.0;
		acc.isLongitudinalValid = true;
		acc.isLateralValid = true;

		driveDir = DriveDirectionType::OBJECT_DRIVE_DIRECTION_FORWARD;
		objState = ObjectStateType::OBJECT_STATE_RUNNING;
		readyToArm = ObjectArmReadinessType::OBJECT_READY_TO_ARM;

		errCode = 0xBEEF;
	}
	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		MessageHeaderType inputHeader;
		inputHeader.receiverID = 0;
		inputHeader.messageCounter = 0;
		inputHeader.transmitterID = 0;
		auto res = encodeMONRMessage(
			&inputHeader,
			&objTime,
			pos,
			spd,
			acc,
			driveDir,
			objState,
			readyToArm,
			0b01101011,
			errCode,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}
	char encodeBuffer[1024];
	struct timeval objTime;
	CartesianPosition pos;
	SpeedType spd;
	AccelerationType acc;
	DriveDirectionType driveDir;
	ObjectStateType objState;
	ObjectArmReadinessType readyToArm;
	unsigned short errCode;

	char* preamble = encodeBuffer + 18; // Skip header
	char* monrStruct = preamble + 4; // Skip preamble

};

TEST_F(EncodeMONR, MessageID)
{
	EXPECT_EQ(encodeBuffer[16], '\x06');
	EXPECT_EQ(encodeBuffer[17], '\x00');
}

TEST_F(EncodeMONR, MessageLength)
{
	EXPECT_EQ(encodeBuffer[2], '\x28');
	EXPECT_EQ(encodeBuffer[3], '\x00');
	EXPECT_EQ(encodeBuffer[4], '\x00');
	EXPECT_EQ(encodeBuffer[5], '\x00');
}

TEST_F(EncodeMONR, Preamble)
{
	EXPECT_EQ(preamble[0], '\x80');
	EXPECT_EQ(preamble[1], '\x00');
	EXPECT_EQ(preamble[2], '\x24');
	EXPECT_EQ(preamble[3], '\x00');
}

TEST_F(EncodeMONR, GPSSecondOfWeek)
{
	// GPS second of week 440560
	// GPS qmsec of week 1762240000 = 0x6909A600
	EXPECT_EQ(monrStruct[0], '\x00');
	EXPECT_EQ(monrStruct[1], '\xA6');
	EXPECT_EQ(monrStruct[2], '\x09');
	EXPECT_EQ(monrStruct[3], '\x69');
}

TEST_F(EncodeMONR, XPosition)
{
	// 1.0 m = 0x000003E8 mm
	EXPECT_EQ(monrStruct[4], '\xE8');
	EXPECT_EQ(monrStruct[5], '\x03');
	EXPECT_EQ(monrStruct[6], '\x00');
	EXPECT_EQ(monrStruct[7], '\x00');
}

TEST_F(EncodeMONR, YPosition)
{
	// -2.0 m = 0xFFFFF830 mm
	EXPECT_EQ(monrStruct[8], '\x30');
	EXPECT_EQ(monrStruct[9], '\xF8');
	EXPECT_EQ(monrStruct[10], '\xFF');
	EXPECT_EQ(monrStruct[11], '\xFF');
}

TEST_F(EncodeMONR, ZPosition)
{
	// 3.0 m = 0x00000BB8 mm
	EXPECT_EQ(monrStruct[12], '\xB8');
	EXPECT_EQ(monrStruct[13], '\x0B');
	EXPECT_EQ(monrStruct[14], '\x00');
	EXPECT_EQ(monrStruct[15], '\x00');
}

TEST_F(EncodeMONR, Yaw)
{
	// 0.4 rad = 2291 centidegrees rounded down = 0x08f3
	EXPECT_EQ(monrStruct[16], '\xF3');
	EXPECT_EQ(monrStruct[17], '\x08');
}

TEST_F(EncodeMONR, Pitch)
{
	EXPECT_EQ(monrStruct[18], '\x00');
	EXPECT_EQ(monrStruct[19], '\x00');
}

TEST_F(EncodeMONR, Roll)
{
	EXPECT_EQ(monrStruct[20], '\x00');
	EXPECT_EQ(monrStruct[21], '\x00');
}

TEST_F(EncodeMONR, LongitudinalSpeed)
{
	// 1.0 m/s = 0x00000064 cm/s
	EXPECT_EQ(monrStruct[22], '\x64');
	EXPECT_EQ(monrStruct[23], '\x00');
}

TEST_F(EncodeMONR, LateralSpeed)
{
	// 2.0 m/s = 0x000000C8 cm/s
	EXPECT_EQ(monrStruct[24], '\xC8');
	EXPECT_EQ(monrStruct[25], '\x00');
}

TEST_F(EncodeMONR, LongitudinalAcceleration)
{
	// 1.0 m/s^2 = 0x000003E8 mm/s^2
	EXPECT_EQ(monrStruct[26], '\xE8');
	EXPECT_EQ(monrStruct[27], '\x03');
}

TEST_F(EncodeMONR, LateralAcceleration)
{
	// 2.0 m/s^2 = 0x000007D0 mm/s^2
	EXPECT_EQ(monrStruct[28], '\xD0');
	EXPECT_EQ(monrStruct[29], '\x07');
}

TEST_F(EncodeMONR, DriveDirection)
{
	EXPECT_EQ(monrStruct[30], '\x00');
}

TEST_F(EncodeMONR, ObjectState)
{
	EXPECT_EQ(monrStruct[31], '\x04');
}

TEST_F(EncodeMONR, ReadyToArm)
{
	EXPECT_EQ(monrStruct[32], '\x01');
}

TEST_F(EncodeMONR, ErrorState)
{
	EXPECT_EQ(monrStruct[33], 0b01101011);
}

TEST_F(EncodeMONR, ErrorCode)
{
	EXPECT_EQ(monrStruct[34], '\xEF');
	EXPECT_EQ(monrStruct[35], '\xBE');
}


class DecodeMONR : public ::testing::Test
{
protected:
	DecodeMONR()
	{	
		decodeBuffer[0] = 0x7F;	// preamble
		decodeBuffer[1] = 0x7E;
		decodeBuffer[2] = 0x28;
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
		decodeBuffer[16] = 0x06;
		decodeBuffer[17] = 0x00;	 // Message ID
		decodeBuffer[18] = 0x80;
		decodeBuffer[19] = 0x00;
		decodeBuffer[20] = 0x24;
		decodeBuffer[21] = 0x00;
		decodeBuffer[22] = 0x00;
		decodeBuffer[23] = 0xA6;
		decodeBuffer[24] = 0x09;
		decodeBuffer[25] = 0x69;
		decodeBuffer[26] = 0xE8;
		decodeBuffer[27] = 0x03;
		decodeBuffer[28] = 0x00;
		decodeBuffer[29] = 0x00;
		decodeBuffer[30] = 0x30;
		decodeBuffer[31] = 0xF8;
		decodeBuffer[32] = 0xFF;
		decodeBuffer[33] = 0xFF;
		decodeBuffer[34] = 0xB8;
		decodeBuffer[35] = 0x0B;
		decodeBuffer[36] = 0x00;
		decodeBuffer[37] = 0x00;
		decodeBuffer[38] = 0xF3;
		decodeBuffer[39] = 0x08;
		decodeBuffer[40] = 0x00;
		decodeBuffer[41] = 0x00;
		decodeBuffer[42] = 0x00;
		decodeBuffer[43] = 0x00;
		decodeBuffer[44] = 0x64;
		decodeBuffer[45] = 0x00;
		decodeBuffer[46] = 0xC8;
		decodeBuffer[47] = 0x00;
		decodeBuffer[48] = 0xE8;
		decodeBuffer[49] = 0x03;
		decodeBuffer[50] = 0xD0;
		decodeBuffer[51] = 0x07;
		decodeBuffer[52] = 0x00;
		decodeBuffer[53] = 0x04;
		decodeBuffer[54] = 0x01;
		decodeBuffer[55] = 0b01101011;
		decodeBuffer[56] = 0xEF;
		decodeBuffer[57] = 0xBE;
		decodeBuffer[58] = 0x33;
		decodeBuffer[59] = 0x86;
	}

	void SetUp() override
	{
		memset(&monrStruct, 0, sizeof(monrStruct));
		// TODO set current time
		currTime.tv_sec = 
		// Friday, April 29, 2022 2:22:22 AM
		// minus 30000 s since it should only be
		// used for getting the GPS week
		currTime.tv_sec = 1651168942;
		currTime.tv_usec = 0;
		auto res = decodeMONRMessage(
			decodeBuffer,
			sizeof(decodeBuffer),
			currTime,
			&monrStruct,
			false);
		ASSERT_GT(res, 0);
	}

	char decodeBuffer[1024];
	ObjectMonitorType monrStruct;
	struct timeval currTime;
};

TEST_F(DecodeMONR, Timestamp)
{
	// Friday, April 29, 2022 2:22:22 AM
	EXPECT_EQ(monrStruct.timestamp.tv_sec, 1651198942);
	EXPECT_EQ(monrStruct.timestamp.tv_usec, 0);
}

TEST_F(DecodeMONR, Position)
{
	ASSERT_TRUE(monrStruct.position.isPositionValid);
	ASSERT_TRUE(monrStruct.position.isXcoordValid);
	ASSERT_TRUE(monrStruct.position.isYcoordValid);
	ASSERT_TRUE(monrStruct.position.isZcoordValid);
	EXPECT_DOUBLE_EQ(monrStruct.position.xCoord_m, 1.0);
	EXPECT_DOUBLE_EQ(monrStruct.position.yCoord_m, -2.0);
	EXPECT_DOUBLE_EQ(monrStruct.position.zCoord_m, 3.0);
}

TEST_F(DecodeMONR, Orientation)
{
	ASSERT_TRUE(monrStruct.position.isHeadingValid);
	EXPECT_FLOAT_EQ(monrStruct.position.heading_rad, 0.399854932);
}

