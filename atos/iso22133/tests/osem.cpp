#include <gtest/gtest.h>
extern "C" {
#include "osem.h"
#include "iso22133.h"
}
#include <string>
#include <sstream>
#include "testdefines.h"

class EncodeOSEM : public ::testing::Test
{
protected:
	EncodeOSEM() {
		// IDs
		settings.desiredID.transmitter = TEST_RECEIVER_ID_2;
		settings.desiredID.subTransmitter = 0x5678;
		// Origin
		settings.coordinateSystemOrigin.latitude_deg = 12.3456789012;
		settings.coordinateSystemOrigin.longitude_deg = 23.4567890123;
		settings.coordinateSystemOrigin.altitude_m = 123.45;
		settings.coordinateSystemOrigin.isLatitudeValid = true;
		settings.coordinateSystemOrigin.isLongitudeValid = true;
		settings.coordinateSystemOrigin.isAltitudeValid = true;
		settings.coordinateSystemRotation_rad = 0.45678;
		settings.coordinateSystemType = COORDINATE_SYSTEM_WGS84;
		// Friday, April 29, 2022 2:22:22 AM
		settings.currentTime.tv_sec = 1651198942;
		settings.currentTime.tv_usec = 0;
		// Requirements
		settings.maxDeviation.position_m = 0.123;
		settings.maxDeviation.lateral_m = 0.456;
		settings.maxDeviation.yaw_rad = 0.789;
		settings.minRequiredPositioningAccuracy_m = 0.12;
		settings.heabTimeout.tv_sec = 1;
		settings.heabTimeout.tv_usec = 20000;
		settings.testMode = TEST_MODE_SCENARIO;
		settings.rate.monr = 4;
		settings.rate.monr2 = 5;
		settings.rate.heab = 6;

		settings.timeServer.ip = 0x12345678;
		settings.timeServer.port = 0x9ABC;
	}
	void SetUp() override
	{
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		MessageHeaderType inputHeader;
		inputHeader.receiverID = TEST_RECEIVER_ID_2;
		inputHeader.messageCounter = 0;
		inputHeader.transmitterID = TEST_TRANSMITTER_ID_2;
		auto res = encodeOSEMMessage(
			&inputHeader,
			&settings, encodeBuffer,
			sizeof(encodeBuffer), false);
		ASSERT_GT(res, 0);
	}
	ObjectSettingsType settings;
	char encodeBuffer[1024];
	char* id = encodeBuffer + 18; // skip header
	char* origin = id + 16;	// skip ID
	char* dateTime = origin + 23; // skip origin
	char* accReq = dateTime + 15; // skip date time
	char* timeServer = accReq + 22; // skip accuracy requirements
};

TEST_F(EncodeOSEM, MessageID)
{
	EXPECT_EQ(encodeBuffer[16], '\x02');
	EXPECT_EQ(encodeBuffer[17], '\x00');
}

TEST_F(EncodeOSEM, MessageLength)
{
	EXPECT_EQ(encodeBuffer[2], '\x56');
	EXPECT_EQ(encodeBuffer[3], '\x00');
	EXPECT_EQ(encodeBuffer[4], '\x00');
	EXPECT_EQ(encodeBuffer[5], '\x00');
}

TEST_F(EncodeOSEM, IdStructPreamble)
{
	EXPECT_EQ(id[0], '\x20');
	EXPECT_EQ(id[1], '\x00');
	EXPECT_EQ(id[2], '\x0C');
	EXPECT_EQ(id[3], '\x00');
}

TEST_F(EncodeOSEM, DeviceId)
{
	EXPECT_EQ(id[4], '\x34');
	EXPECT_EQ(id[5], '\x12');
	EXPECT_EQ(id[6], '\x00');
	EXPECT_EQ(id[7], '\x00');
}

TEST_F(EncodeOSEM, SubDeviceId)
{
	EXPECT_EQ(id[8], '\x78');
	EXPECT_EQ(id[9], '\x56');
	EXPECT_EQ(id[10], '\x00');
	EXPECT_EQ(id[11], '\x00');
}

TEST_F(EncodeOSEM, ControlCentreId)
{
	EXPECT_EQ(id[12], '\xBC');
	EXPECT_EQ(id[13], '\x9A');
	EXPECT_EQ(id[14], '\x00');
	EXPECT_EQ(id[15], '\x00');
}

TEST_F(EncodeOSEM, OriginStructPreamble)
{
	EXPECT_EQ(origin[0], '\x21');
	EXPECT_EQ(origin[1], '\x00');
	EXPECT_EQ(origin[2], '\x13');
	EXPECT_EQ(origin[3], '\x00');
}

TEST_F(EncodeOSEM, Latitude)
{
	// 123456789012 nd = 0x001CBE991A14
	EXPECT_EQ(origin[4], '\x14');
	EXPECT_EQ(origin[5], '\x1A');
	EXPECT_EQ(origin[6], '\x99');
	EXPECT_EQ(origin[7], '\xBE');
	EXPECT_EQ(origin[8], '\x1C');
	EXPECT_EQ(origin[9], '\x00');
}

TEST_F(EncodeOSEM, Longitude)
{
	// 23456789012 nd = 0x00369D55F4CB
	EXPECT_EQ(origin[10], '\xCB');
	EXPECT_EQ(origin[11], '\xF4');
	EXPECT_EQ(origin[12], '\x55');
	EXPECT_EQ(origin[13], '\x9D');
	EXPECT_EQ(origin[14], '\x36');
	EXPECT_EQ(origin[15], '\x00');
}

TEST_F(EncodeOSEM, Altitude)
{
	// 12345 cm = 0x00003039
	EXPECT_EQ(origin[16], '\x39');
	EXPECT_EQ(origin[17], '\x30');
	EXPECT_EQ(origin[18], '\x00');
	EXPECT_EQ(origin[19], '\x00');
}

TEST_F(EncodeOSEM, Rotation)
{
	// 0.45678 rad = 2617 cd = 0x0A39
	EXPECT_EQ(origin[20], '\x39');
	EXPECT_EQ(origin[21], '\x0A');

	EXPECT_EQ(origin[22], '\x03');
}

TEST_F(EncodeOSEM, DateTimeStructPreamble)
{
	EXPECT_EQ(dateTime[0], '\x22');
	EXPECT_EQ(dateTime[1], '\x00');
	EXPECT_EQ(dateTime[2], '\x0B');
	EXPECT_EQ(dateTime[3], '\x00');
}

TEST_F(EncodeOSEM, Date)
{
	// 2022-04-29 02:22:22
	// 20220429 = 0x01348A0D
	EXPECT_EQ(dateTime[4], '\x0D');
	EXPECT_EQ(dateTime[5], '\x8A');
	EXPECT_EQ(dateTime[6], '\x34');
	EXPECT_EQ(dateTime[7], '\x01');
}

TEST_F(EncodeOSEM, GPSWeek)
{
	// GPS week 2207 = 0x089F
	EXPECT_EQ(dateTime[8], '\x9F');
	EXPECT_EQ(dateTime[9], '\x08');
}

TEST_F(EncodeOSEM, GPSSOW)
{
	// GPS second of week 440560
	// GPS qmsec of week 1762240000 = 0x6909A600
	EXPECT_EQ(dateTime[10], '\x00');
	EXPECT_EQ(dateTime[11], '\xA6');
	EXPECT_EQ(dateTime[12], '\x09');
	EXPECT_EQ(dateTime[13], '\x69');
}

TEST_F(EncodeOSEM, LeapSeconds)
{
	// 18 = 0x12
	EXPECT_EQ(dateTime[14], '\x12');
}

TEST_F(EncodeOSEM, AccReqStructPreamble)
{
	EXPECT_EQ(accReq[0], '\x23');
	EXPECT_EQ(accReq[1], '\x00');
	EXPECT_EQ(accReq[2], '\x12');
	EXPECT_EQ(accReq[3], '\x00');
}

TEST_F(EncodeOSEM, MaxWayDeviation)
{
	// 123 mm = 0x007B
	EXPECT_EQ(accReq[4], '\x7B');
	EXPECT_EQ(accReq[5], '\x00');
}

TEST_F(EncodeOSEM, MaxLateralDeviation)
{
	// 456 mm = 0x01C8
	EXPECT_EQ(accReq[6], '\xC8');
	EXPECT_EQ(accReq[7], '\x01');
}

TEST_F(EncodeOSEM, MaxYawDeviation)
{
	// 0.789 rad = 4520 cd rounded down = 0x11A8
	EXPECT_EQ(accReq[8], '\xA8');
	EXPECT_EQ(accReq[9], '\x11');
}

TEST_F(EncodeOSEM, MinPosAcc)
{
	// 12 cm = 0x000C
	EXPECT_EQ(accReq[10], '\x0C');
	EXPECT_EQ(accReq[11], '\x00');
}

TEST_F(EncodeOSEM, HeabTimeout)
{
	// 1.020 sec = 102 cs = 0x0066
	EXPECT_EQ(accReq[12], '\x66');
	EXPECT_EQ(accReq[13], '\x00');
}

TEST_F(EncodeOSEM, TestMode)
{
	// Test mode scenario
	EXPECT_EQ(accReq[14], '\x02');
}

TEST_F(EncodeOSEM, MessageRates)
{
	// 4 Hz = 0x04
	EXPECT_EQ(accReq[15], '\x04');
	// 5 Hz = 0x05
	EXPECT_EQ(accReq[16], '\x05');
	// 6 Hz = 0x06
	EXPECT_EQ(accReq[17], '\x06');
	// Message length not specified
}

TEST_F(EncodeOSEM, TimeServerStructPreamble)
{
	EXPECT_EQ(timeServer[0], '\x24');
	EXPECT_EQ(timeServer[1], '\x00');
	EXPECT_EQ(timeServer[2], '\x06');
	EXPECT_EQ(timeServer[3], '\x00');
}

TEST_F(EncodeOSEM, TimeServerIP)
{
	EXPECT_EQ(timeServer[4], '\x78');
	EXPECT_EQ(timeServer[5], '\x56');
	EXPECT_EQ(timeServer[6], '\x34');
	EXPECT_EQ(timeServer[7], '\x12');
}

TEST_F(EncodeOSEM, TimeServerPort)
{
	EXPECT_EQ(timeServer[8], '\xBC');
	EXPECT_EQ(timeServer[9], '\x9A');
}

TEST_F(EncodeOSEM, NoTimeServerStruct)
{
	settings.timeServer.ip = 0;
	settings.timeServer.port = 0;
	timeServer[0] = 0;
	timeServer[1] = 0;
	timeServer[2] = 0;
	timeServer[3] = 0;
	MessageHeaderType inputHeader;
	inputHeader.receiverID = 0;
	inputHeader.messageCounter = 0;
	inputHeader.transmitterID = 0;
	auto res = encodeOSEMMessage(
		&inputHeader,
		&settings, encodeBuffer,
		sizeof(encodeBuffer), false);
	ASSERT_GT(res, 0);
	union {
		uint8_t bytes[2];
		uint16_t val;
	} u;
	u.bytes[0] = timeServer[0];
	u.bytes[1] = timeServer[1];
	EXPECT_NE(le16toh(u.val), '\x24');
}


class DecodeOSEM : public ::testing::Test
{
protected:
	DecodeOSEM() {
		decodeBuffer[0] = 0x7F;	// preamble
		decodeBuffer[1] = 0x7E;
		decodeBuffer[2] = 0x52;
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
		decodeBuffer[16] = 0x02;
		decodeBuffer[17] = 0x00;	 // Message ID

		decodeBuffer[18] = 0x20;
		decodeBuffer[19] = 0x00;
		decodeBuffer[20] = 0x0C;
		decodeBuffer[21] = 0x00;	 // ID struct preamble
		decodeBuffer[22] = 0xAB;
		decodeBuffer[23] = 0x90;
		decodeBuffer[24] = 0x00;
		decodeBuffer[25] = 0x00;	 // Device ID
		decodeBuffer[26] = 0xEF;
		decodeBuffer[27] = 0xCD;
		decodeBuffer[28] = 0x00;
		decodeBuffer[29] = 0x00;	 // Sub device header
		decodeBuffer[30] = 0x34;
		decodeBuffer[31] = 0x12;
		decodeBuffer[32] = 0x00;
		decodeBuffer[33] = 0x00;	 // Control centre ID

		decodeBuffer[34] = 0x21;
		decodeBuffer[35] = 0x00;
		decodeBuffer[36] = 0x13;
		decodeBuffer[37] = 0x00;	 // Origin struct preamble
		decodeBuffer[38] = 0x14;
		decodeBuffer[39] = 0x1A;
		decodeBuffer[40] = 0x99;
		decodeBuffer[41] = 0xBE;
		decodeBuffer[42] = 0x1C;
		decodeBuffer[43] = 0x00;	 // Latitude
		decodeBuffer[44] = 0xCB;
		decodeBuffer[45] = 0xF4;
		decodeBuffer[46] = 0x55;
		decodeBuffer[47] = 0x9D;
		decodeBuffer[48] = 0x36;
		decodeBuffer[49] = 0x00;	 // Longitude
		decodeBuffer[50] = 0x39;
		decodeBuffer[51] = 0x30;
		decodeBuffer[52] = 0x00;
		decodeBuffer[53] = 0x00;	 // Altitude
		decodeBuffer[54] = 0x39;
		decodeBuffer[55] = 0x0A;	 // Rotation
		decodeBuffer[56] = 0x03;	 // Coordinate system

		decodeBuffer[57] = 0x22;
		decodeBuffer[58] = 0x00;
		decodeBuffer[59] = 0x0B;
		decodeBuffer[60] = 0x00;	 // DateTime struct preamble
		decodeBuffer[61] = 0x0D;
		decodeBuffer[62] = 0x8A;
		decodeBuffer[63] = 0x34;
		decodeBuffer[64] = 0x01;	 // Date
		decodeBuffer[65] = 0x9F;
		decodeBuffer[66] = 0x08;	 // GPS week
		decodeBuffer[67] = 0x00;
		decodeBuffer[68] = 0xA6;
		decodeBuffer[69] = 0x09;
		decodeBuffer[70] = 0x69;	 // GPS second of week
		decodeBuffer[71] = 0x12;	 // Leap seconds

		decodeBuffer[72] = 0x23;
		decodeBuffer[73] = 0x00;
		decodeBuffer[74] = 0x12;
		decodeBuffer[75] = 0x00;	 // Requirements struct preamble
		decodeBuffer[76] = 0x7B;
		decodeBuffer[77] = 0x00;	 // Max way deviation
		decodeBuffer[78] = 0xC8;
		decodeBuffer[79] = 0x01;	 // Max lateral deviation
		decodeBuffer[80] = 0xA8;
		decodeBuffer[81] = 0x11;	 // Max yaw deviation
		decodeBuffer[82] = 0x0C;
		decodeBuffer[83] = 0x00;	 // Min position accuracy
		decodeBuffer[84] = 0x66;
		decodeBuffer[85] = 0x00;	 // Heab timeout
		decodeBuffer[86] = 0x02;	 // Test mode
		decodeBuffer[87] = 0x04;	 // monr rate
		decodeBuffer[88] = 0x05;	 // monr2 rate
		decodeBuffer[89] = 0x06;	 // heab rate
		decodeBuffer[90] = 0xFF;
		decodeBuffer[91] = 0xFF;
		decodeBuffer[92] = 0xFF;
		decodeBuffer[93] = 0xFF;	 // Max message length

		decodeBuffer[94] = 0x24;
		decodeBuffer[95] = 0x00;
		decodeBuffer[96] = 0x06;
		decodeBuffer[97] = 0x00;	 // Time server struct preamble
		decodeBuffer[98] = 0x78;
		decodeBuffer[99] = 0x56;
		decodeBuffer[100] = 0x34;
		decodeBuffer[101] = 0x12;	 // Time server IP
		decodeBuffer[102] = 0xBC;
		decodeBuffer[103] = 0x9A;	 // Time server port

		decodeBuffer[104] = 0x00;
		decodeBuffer[105] = 0x00;	 // CRC
	}
	virtual void SetUp()
	{
		memset(&settings, 0, sizeof(settings));
		decodeOSEMMessage(&settings, decodeBuffer, sizeof(decodeBuffer), true);
	}
	char decodeBuffer[1024];
	ObjectSettingsType settings;
};

TEST_F(DecodeOSEM, TransmitterID)
{
	EXPECT_EQ(settings.desiredID.transmitter, 0x90AB);
}

TEST_F(DecodeOSEM, SubTransmitterID)
{
	EXPECT_EQ(settings.desiredID.subTransmitter, 0xCDEF);
}

TEST_F(DecodeOSEM, ControlCentreID)
{
	EXPECT_EQ(settings.desiredID.controlCentre, 0x1234);
}

TEST_F(DecodeOSEM, Latitude)
{
	EXPECT_NEAR(settings.coordinateSystemOrigin.latitude_deg, 12.3456789012, 0.0000000001);
}

TEST_F(DecodeOSEM, Longitude)
{
	EXPECT_NEAR(settings.coordinateSystemOrigin.longitude_deg, 23.4567890123, 0.0000000001);
}

TEST_F(DecodeOSEM, Altitude)
{
	EXPECT_NEAR(settings.coordinateSystemOrigin.altitude_m, 123.45, 0.01);
}

TEST_F(DecodeOSEM, Rotation)
{
	EXPECT_NEAR(settings.coordinateSystemRotation_rad, 0.45675, 0.00001);
}

TEST_F(DecodeOSEM, CoordinateSystem)
{
	EXPECT_EQ(settings.coordinateSystemType, COORDINATE_SYSTEM_WGS84);
}

TEST_F(DecodeOSEM, Date)
{
	EXPECT_EQ(settings.currentTime.tv_sec, 1651198942);
	EXPECT_EQ(settings.currentTime.tv_usec, 0);
}

TEST_F(DecodeOSEM, MaxWayDeviation)
{
	EXPECT_NEAR(settings.maxDeviation.position_m, 0.123, 0.001);
}

TEST_F(DecodeOSEM, MaxLateralDeviation)
{
	EXPECT_NEAR(settings.maxDeviation.lateral_m, 0.456, 0.001);
}

TEST_F(DecodeOSEM, MaxYawDeviation)
{
	EXPECT_NEAR(settings.maxDeviation.yaw_rad, 0.789, 0.001);
}

TEST_F(DecodeOSEM, MinPositionAccuracy)
{
	EXPECT_NEAR(settings.minRequiredPositioningAccuracy_m, 0.12, 0.01);
}

TEST_F(DecodeOSEM, HeabTimeout)
{
	EXPECT_EQ(settings.heabTimeout.tv_sec, 1);
}

TEST_F(DecodeOSEM, TestMode)
{
	EXPECT_EQ(settings.testMode, TEST_MODE_SCENARIO);
}

TEST_F(DecodeOSEM, MonrRate)
{
	EXPECT_EQ(settings.rate.monr, 4);
}

TEST_F(DecodeOSEM, Monr2Rate)
{
	EXPECT_EQ(settings.rate.monr2, 5);
}

TEST_F(DecodeOSEM, HeabRate)
{
	EXPECT_EQ(settings.rate.heab, 6);
}

TEST_F(DecodeOSEM, TimeServerIP)
{
	EXPECT_EQ(settings.timeServer.ip, 0x12345678);
}

TEST_F(DecodeOSEM, TimeServerPort)
{
	EXPECT_EQ(settings.timeServer.port, 0x9ABC);
}
