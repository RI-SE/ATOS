#include "traj.h"
#include <gtest/gtest.h>

class EncodeTRAJHeader : public ::testing::Test
{
protected:
	EncodeTRAJHeader() {
	}
	void SetUp() override
	{
		char name[] = "some description";
		MessageHeaderType inputHeader;
		inputHeader.receiverID = 0;
		inputHeader.messageCounter = 0;
		inputHeader.transmitterID = 0;
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto res = encodeTRAJMessageHeader(
			&inputHeader,
			0x0123,
			TRAJECTORY_INFO_RELATIVE_TO_ORIGIN,
			name,
			sizeof(name),
			21,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	ObjectSettingsType settings;
	char encodeBuffer[1024];
	char* id = encodeBuffer + 18; // skip header
	char* info = id + 6;	// skip ID
	char* name = info + 5;	// skip info
};

TEST_F(EncodeTRAJHeader, MessageID)
{
	EXPECT_EQ(encodeBuffer[16], '\x01');
	EXPECT_EQ(encodeBuffer[17], '\x00');
}

TEST_F(EncodeTRAJHeader, MessageLength)
{
	// 68+6+5+21×(34)+5
	EXPECT_EQ(encodeBuffer[2], '\x1E');
	EXPECT_EQ(encodeBuffer[3], '\x03');
	EXPECT_EQ(encodeBuffer[4], '\x00');
	EXPECT_EQ(encodeBuffer[5], '\x00');
}

TEST_F(EncodeTRAJHeader, Id)
{
	EXPECT_EQ(id[0], '\x01');
	EXPECT_EQ(id[1], '\x01');
	EXPECT_EQ(id[2], '\x02');
	EXPECT_EQ(id[3], '\x00');
	EXPECT_EQ(id[4], '\x23');
	EXPECT_EQ(id[5], '\x01');
}

TEST_F(EncodeTRAJHeader, Info)
{
	EXPECT_EQ(info[0], '\x04');
	EXPECT_EQ(info[1], '\x01');
	EXPECT_EQ(info[2], '\x01');
	EXPECT_EQ(info[3], '\x00');
	EXPECT_EQ(info[4], '\x02');
}

TEST_F(EncodeTRAJHeader, Name)
{
	EXPECT_EQ(name[0], '\x02');
	EXPECT_EQ(name[1], '\x01');
	EXPECT_EQ(name[2], '\x40');
	EXPECT_EQ(name[3], '\x00');
	EXPECT_EQ(0, memcmp(name + 4, "some description", 17));
	// Check that remaining bytes are zero
	for (int i = 21; i < 68; i++) {
		EXPECT_EQ(name[i], '\0');
	}
}

class DecodeTRAJHeader : public ::testing::Test
{
protected:
	DecodeTRAJHeader() {
		decodeBuffer[0] = 0x7F;
		decodeBuffer[1] = 0x7E;	// preamble
		decodeBuffer[2] = 0x52;
		decodeBuffer[3] = 0x00;
		decodeBuffer[4] = 0x00;
		decodeBuffer[5] = 0x00;	 // TODO Message length
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
		decodeBuffer[16] = 0x01;
		decodeBuffer[17] = 0x00;	 // Message ID

		decodeBuffer[18] = 0x01;
		decodeBuffer[19] = 0x01;
		decodeBuffer[20] = 0x02;
		decodeBuffer[21] = 0x00;	 // ID preamble
		decodeBuffer[22] = 0x23;
		decodeBuffer[23] = 0x01;

		decodeBuffer[24] = 0x04;
		decodeBuffer[25] = 0x01;
		decodeBuffer[26] = 0x01;
		decodeBuffer[27] = 0x00;	 // Info preamble
		decodeBuffer[28] = 0x02;

		decodeBuffer[29] = 0x02;
		decodeBuffer[30] = 0x01;
		decodeBuffer[31] = 0x40;
		decodeBuffer[32] = 0x00;	 // Name preamble
		decodeBuffer[33] = 's';
		decodeBuffer[34] = 'o';
		decodeBuffer[35] = 'm';
		decodeBuffer[36] = 'e';
		decodeBuffer[37] = ' ';
		decodeBuffer[38] = 'd';
		decodeBuffer[39] = 'e';
		decodeBuffer[40] = 's';
		decodeBuffer[41] = 'c';
		decodeBuffer[42] = 'r';
		decodeBuffer[43] = 'i';
		decodeBuffer[44] = 'p';
		decodeBuffer[45] = 't';
		decodeBuffer[46] = 'i';
		decodeBuffer[47] = 'o';
		decodeBuffer[48] = 'n';
		decodeBuffer[49] = '\0';
		for (int i = 50; i < 29+68; i++) {
			decodeBuffer[i] = '\0';
		}
	}

	void SetUp() override
	{
		auto res = decodeTRAJMessageHeader(
			&header,
			decodeBuffer,
			sizeof(decodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	TrajectoryHeaderType header;
	char decodeBuffer[1024];
};

TEST_F(DecodeTRAJHeader, Id)
{
	EXPECT_EQ(header.trajectoryID, 0x0123);
}

TEST_F(DecodeTRAJHeader, Info)
{
	EXPECT_EQ(header.trajectoryInfo, TRAJECTORY_INFO_RELATIVE_TO_ORIGIN);
}

TEST_F(DecodeTRAJHeader, Name)
{
	EXPECT_EQ(0, memcmp(header.trajectoryName, "some description", 17));
	for (int i = 17; i < 64; i++) {
		EXPECT_EQ(header.trajectoryName[i], '\0');
	}
}

class EncodeTRAJPoint : public ::testing::Test
{
protected:
	EncodeTRAJPoint() 
	{
		timeFromStart.tv_sec = 1;
		timeFromStart.tv_usec = 2000;
		pos.isXcoordValid = true;
		pos.isYcoordValid = true;
		pos.isZcoordValid = true;
		pos.isPositionValid = true;
		pos.xCoord_m = 1.0;
		pos.yCoord_m = -2.0;
		pos.zCoord_m = 3.0;
		pos.isHeadingValid = true;
		pos.heading_rad = 0.4;
		speed.isLongitudinalValid = true;
		speed.isLateralValid = true;
		speed.longitudinal_m_s = 2.0;
		speed.lateral_m_s = 1.0;
		acc.isLongitudinalValid = true;
		acc.isLateralValid = true;
		acc.longitudinal_m_s2 = 3.0;
		acc.lateral_m_s2 = 4.0;
		curvature = 0.5;
	}

	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto res = encodeTRAJMessagePoint(
			&timeFromStart,
			pos,
			speed,
			acc,
			curvature,
			encodeBuffer,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(res, 0);
	}

	char encodeBuffer[1024];
	struct timeval timeFromStart;
	CartesianPosition pos;
	SpeedType speed;
	AccelerationType acc;
	float curvature;
};


TEST_F(EncodeTRAJPoint, Preamble)
{
	EXPECT_EQ(encodeBuffer[0], '\x01');
	EXPECT_EQ(encodeBuffer[1], '\x00');
	EXPECT_EQ(encodeBuffer[2], '\x1E');
	EXPECT_EQ(encodeBuffer[3], '\x00');
}

TEST_F(EncodeTRAJPoint, RelativeTime)
{
	// 1.002 seconds
	EXPECT_EQ(encodeBuffer[4], '\xEA');
	EXPECT_EQ(encodeBuffer[5], '\x03');
	EXPECT_EQ(encodeBuffer[6], '\x00');
	EXPECT_EQ(encodeBuffer[7], '\x00');
}

TEST_F(EncodeTRAJPoint, XPosition)
{
	// 1.0 meters
	EXPECT_EQ(encodeBuffer[8], '\xE8');
	EXPECT_EQ(encodeBuffer[9], '\x03');
	EXPECT_EQ(encodeBuffer[10], '\x00');
	EXPECT_EQ(encodeBuffer[11], '\x00');
}

TEST_F(EncodeTRAJPoint, YPosition)
{
	// -2.0 meters
	EXPECT_EQ(encodeBuffer[12], '\x30');
	EXPECT_EQ(encodeBuffer[13], '\xF8');
	EXPECT_EQ(encodeBuffer[14], '\xFF');
	EXPECT_EQ(encodeBuffer[15], '\xFF');
}

TEST_F(EncodeTRAJPoint, ZPosition)
{
	// 3.0 meters
	EXPECT_EQ(encodeBuffer[16], '\xB8');
	EXPECT_EQ(encodeBuffer[17], '\x0B');
	EXPECT_EQ(encodeBuffer[18], '\x00');
	EXPECT_EQ(encodeBuffer[19], '\x00');
}

TEST_F(EncodeTRAJPoint, Yaw)
{
	// 0.4 radians = 2291 centidegrees rounded down
	EXPECT_EQ(encodeBuffer[20], '\xF3');
	EXPECT_EQ(encodeBuffer[21], '\x08');
}

TEST_F(EncodeTRAJPoint, LongitudinalSpeed)
{
	// 2.0 meters per second
	EXPECT_EQ(encodeBuffer[22], '\xC8');
	EXPECT_EQ(encodeBuffer[23], '\x00');
}

TEST_F(EncodeTRAJPoint, LateralSpeed)
{
	// 1.0 meters per second
	EXPECT_EQ(encodeBuffer[24], '\x64');
	EXPECT_EQ(encodeBuffer[25], '\x00');
}

TEST_F(EncodeTRAJPoint, LongitudinalAcceleration)
{
	// 3.0 meters per second squared
	EXPECT_EQ(encodeBuffer[26], '\xB8');
	EXPECT_EQ(encodeBuffer[27], '\x0B');
}

TEST_F(EncodeTRAJPoint, LateralAcceleration)
{
	// 4.0 meters per second squared
	EXPECT_EQ(encodeBuffer[28], '\xA0');
	EXPECT_EQ(encodeBuffer[29], '\x0F');
}

TEST_F(EncodeTRAJPoint, Curvature)
{
	// 0.5 meters per second squared
	union {
		float val;
		uint32_t bytes;
	} u;
	memcpy(&u.bytes, &encodeBuffer[30], sizeof(u.bytes));
	u.bytes = le32toh(u.bytes);
	EXPECT_FLOAT_EQ(u.val, 0.5);
}


class DecodeTRAJPoint : public ::testing::Test
{
protected:
	DecodeTRAJPoint()
	{
		decodeBuffer[0] = '\x01';
		decodeBuffer[1] = '\x00';
		decodeBuffer[2] = '\x1E';
		decodeBuffer[3] = '\x00';

		decodeBuffer[4] = '\xEA';
		decodeBuffer[5] = '\x03';
		decodeBuffer[6] = '\x00';
		decodeBuffer[7] = '\x00';

		decodeBuffer[8] = '\xE8';
		decodeBuffer[9] = '\x03';
		decodeBuffer[10] = '\x00';
		decodeBuffer[11] = '\x00';

		decodeBuffer[12] = '\x30';
		decodeBuffer[13] = '\xF8';
		decodeBuffer[14] = '\xFF';
		decodeBuffer[15] = '\xFF';

		decodeBuffer[16] = '\xB8';
		decodeBuffer[17] = '\x0B';
		decodeBuffer[18] = '\x00';
		decodeBuffer[19] = '\x00';

		decodeBuffer[20] = '\xF3';
		decodeBuffer[21] = '\x08';

		decodeBuffer[22] = '\xC8';
		decodeBuffer[23] = '\x00';

		decodeBuffer[24] = '\x64';
		decodeBuffer[25] = '\x00';

		decodeBuffer[26] = '\xB8';
		decodeBuffer[27] = '\x0B';

		decodeBuffer[28] = '\xA0';
		decodeBuffer[29] = '\x0F';

		union {
			float flt;
			uint32_t u32;
		} u;
		u.flt = 0.5;
		u.u32 = htole32(u.u32);
		memcpy(&decodeBuffer[30], &u.u32, sizeof(u.u32));
	}

	void SetUp() override
	{
		memset(&point, 0, sizeof(point));
		auto res = decodeTRAJMessagePoint(
			&point,
			decodeBuffer,
			false);
		ASSERT_GT(res, 0);
	}

	TrajectoryWaypointType point;
	char decodeBuffer[1024];
};

TEST_F(DecodeTRAJPoint, RelativeTime)
{
	EXPECT_EQ(point.relativeTime.tv_sec, 1);
	EXPECT_EQ(point.relativeTime.tv_usec, 2000);
}

TEST_F(DecodeTRAJPoint, XPosition)
{
	ASSERT_TRUE(point.pos.isXcoordValid);
	ASSERT_TRUE(point.pos.isPositionValid);
	EXPECT_FLOAT_EQ(point.pos.xCoord_m, 1.0);
}

TEST_F(DecodeTRAJPoint, YPosition)
{
	ASSERT_TRUE(point.pos.isYcoordValid);
	ASSERT_TRUE(point.pos.isPositionValid);
	EXPECT_FLOAT_EQ(point.pos.yCoord_m, -2.0);
}

TEST_F(DecodeTRAJPoint, ZPosition)
{
	ASSERT_TRUE(point.pos.isZcoordValid);
	ASSERT_TRUE(point.pos.isPositionValid);
	EXPECT_FLOAT_EQ(point.pos.zCoord_m, 3.0);
}

TEST_F(DecodeTRAJPoint, Yaw)
{
	ASSERT_TRUE(point.pos.isHeadingValid);
	EXPECT_FLOAT_EQ(point.pos.heading_rad, 0.399854932);
}

TEST_F(DecodeTRAJPoint, LongitudinalSpeed)
{
	ASSERT_TRUE(point.spd.isLongitudinalValid);
	EXPECT_FLOAT_EQ(point.spd.longitudinal_m_s, 2.0);
}

TEST_F(DecodeTRAJPoint, LateralSpeed)
{
	ASSERT_TRUE(point.spd.isLateralValid);
	EXPECT_FLOAT_EQ(point.spd.lateral_m_s, 1.0);
}

TEST_F(DecodeTRAJPoint, LongitudinalAcceleration)
{
	ASSERT_TRUE(point.acc.isLongitudinalValid);
	EXPECT_FLOAT_EQ(point.acc.longitudinal_m_s2, 3.0);
}

TEST_F(DecodeTRAJPoint, LateralAcceleration)
{
	ASSERT_TRUE(point.acc.isLateralValid);
	EXPECT_FLOAT_EQ(point.acc.lateral_m_s2, 4.0);
}

TEST_F(DecodeTRAJPoint, Curvature)
{
	EXPECT_FLOAT_EQ(point.curvature, 0.5);
}

class EncodeTRAJFooter : public ::testing::Test
{
protected:
	EncodeTRAJFooter()
	{

	}
	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto p = encodeBuffer;
		MessageHeaderType inputHeader;
		inputHeader.receiverID = 0;
		inputHeader.messageCounter = 0;
		inputHeader.transmitterID = 0x000000FF;
		auto offset = encodeTRAJMessageHeader(
			&inputHeader,
			0x123,
			TRAJECTORY_INFO_RELATIVE_TO_OBJECT,
			"some description",
			17,
			3,
			p,
			sizeof(encodeBuffer),
			false);
		ASSERT_GT(offset, 0);
		p += offset;
		for (int i = 0; i < 3; i++) {
			struct timeval tv = {1,2};
			CartesianPosition pos = {1,2,3,4,true,true,true,true,true};
			SpeedType spd = {1,2,true,true};
			AccelerationType acc = {1,2,true,true};
			offset = encodeTRAJMessagePoint(
				&tv,
				pos,
				spd,
				acc,
				12.34,
				p,
				sizeof(encodeBuffer) - (p-encodeBuffer),
				false);
			ASSERT_GT(offset, 0);
			p += offset;
		}
		offset = encodeTRAJMessageFooter(
			p,
			sizeof(encodeBuffer) - (p - encodeBuffer),
			false
		);
		ASSERT_GT(offset, 0);
		p += offset;
		// Run this to view raw data for which to generate CRC
		// for (int i = 0; i < p - encodeBuffer; ++i) {
		// 	printf("%02x ",(uint8_t)encodeBuffer[i]);
		// }
		// printf("\n");
	}

	char encodeBuffer[1024];
	char* lineInfo = encodeBuffer + 18 + (6 + 5 + 68) + 3*34; // Skip message header, traj header and 3 points
	char* crc = lineInfo + 5;
};

TEST_F(EncodeTRAJFooter, EndOfTransmission) {
	EXPECT_EQ(lineInfo[0], '\x53');
	EXPECT_EQ(lineInfo[1], '\x00');
	EXPECT_EQ(lineInfo[2], '\x01');
	EXPECT_EQ(lineInfo[3], '\x00');
	EXPECT_EQ(lineInfo[4], '\x04');
}

// https://crccalc.com/?crc=7f 7e ba 00 00 00 02 ff 00 00 00 00 00 00 00 00 01 00 01 01 02 00 23 01 04 01 01 00 01 02 01 40 00 73 6f 6d 65 20 64 65 73 63 72 69 70 74 69 6f 6e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 1e 00 e8 03 00 00 e8 03 00 00 d0 07 00 00 b8 0b 00 00 86 59 64 00 c8 00 e8 03 d0 07 a4 70 45 41 01 00 1e 00 e8 03 00 00 e8 03 00 00 d0 07 00 00 b8 0b 00 00 86 59 64 00 c8 00 e8 03 d0 07 a4 70 45 41 01 00 1e 00 e8 03 00 00 e8 03 00 00 d0 07 00 00 b8 0b 00 00 86 59 64 00 c8 00 e8 03 d0 07 a4 70 45 41 53 00 01 00 04&method=crc16&datatype=hex&outtype=0
TEST_F(EncodeTRAJFooter, Crc) {
	EXPECT_EQ(crc[0], '\x34');
	EXPECT_EQ(crc[1], '\x21');
}
