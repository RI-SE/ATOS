#include <gtest/gtest.h>
extern "C" {
#include "header.h"
}
#include "testdefines.h"

class HeaderDecode : public ::testing::Test
{
protected:
	HeaderDecode() {
		*reinterpret_cast<uint16_t*>(message) = htole16(TEST_SYNC_WORD);				// Sync word
		*reinterpret_cast<uint32_t*>(message + 2) = htole32(TEST_HEADER_MESSAGE_LENGTH);// Message length
		*reinterpret_cast<uint8_t*>(message + 6) = TEST_ACKREQPROTVER;	  				// Ack req protocol version
		*reinterpret_cast<uint32_t*>(message + 7) = htole32(TEST_HEADER_TRANSMITTER_ID);// Transmitter ID
		*reinterpret_cast<uint32_t*>(message + 11) = htole32(TEST_DEFAULT_RECEIVER_ID);	// Receiver ID
		*reinterpret_cast<uint8_t*>(message + 15) = TEST_DEFAULT_MESSAGE_COUNTER;		// Message counter
		*reinterpret_cast<uint16_t*>(message + 16) = htole16(TEST_HEADER_MESSAGE_ID);	// Message ID
	}
	void SetUp() override {
		auto ret = decodeISOHeader(message, 18, &header, false);
		ASSERT_EQ(MESSAGE_OK, ret);
	}
	virtual ~HeaderDecode();
	char message[18];
	HeaderType header;
};
HeaderDecode::~HeaderDecode() {}

TEST_F(HeaderDecode, SyncWord) {
	EXPECT_EQ(TEST_SYNC_WORD, header.syncWord);
}

TEST_F(HeaderDecode, MessageLength) {
	EXPECT_EQ(TEST_HEADER_MESSAGE_LENGTH, header.messageLength);
}

TEST_F(HeaderDecode, AckReq) {
	EXPECT_EQ(TEST_ACKREQPROTVER, header.ackReqProtVer);
}

TEST_F(HeaderDecode, TransmitterID) {
	EXPECT_EQ(TEST_HEADER_TRANSMITTER_ID, header.transmitterID);
}

TEST_F(HeaderDecode, ReceiverID) {
	EXPECT_EQ(TEST_DEFAULT_RECEIVER_ID, header.receiverID);
}

TEST_F(HeaderDecode, MessageCounter) {
	EXPECT_EQ(TEST_DEFAULT_MESSAGE_COUNTER, header.messageCounter);
}

TEST_F(HeaderDecode, MessageID) {
	EXPECT_EQ(TEST_HEADER_MESSAGE_ID, header.messageID);
}

class HeaderEncode : public ::testing::Test
{
protected:
	HeaderEncode() {}
	virtual ~HeaderEncode();
	void SetUp() override {
		MessageHeaderType inputHeader;
		inputHeader.transmitterID = TEST_DEFAULT_TRANSMITTER_ID;
		inputHeader.receiverID = TEST_DEFAULT_RECEIVER_ID;
		inputHeader.messageCounter = TEST_DEFAULT_MESSAGE_COUNTER;
		header = buildISOHeader(MESSAGE_ID_TRAJ, &inputHeader, 123, false);
	}
	HeaderType header;
};
HeaderEncode::~HeaderEncode() {}

TEST_F(HeaderEncode, SyncWord) {
	EXPECT_EQ(TEST_SYNC_WORD, le16toh(header.syncWord));
}

TEST_F(HeaderEncode, MessageLength) {
	EXPECT_EQ(103, le32toh(header.messageLength));	// Message length excludes header and footer
}

TEST_F(HeaderEncode, AckReqProtVer) {
	EXPECT_EQ(0x02, header.ackReqProtVer);
}

TEST_F(HeaderEncode, TransmitterID) {
	EXPECT_EQ(TEST_DEFAULT_TRANSMITTER_ID, le32toh(header.transmitterID));
}

TEST_F(HeaderEncode, ReceiverID) {
	EXPECT_EQ(TEST_DEFAULT_RECEIVER_ID, le32toh(header.receiverID));
}

TEST_F(HeaderEncode, MessageCounter) {
	EXPECT_EQ(TEST_DEFAULT_MESSAGE_COUNTER, header.messageCounter);
}

TEST_F(HeaderEncode, MessageID) {
	EXPECT_EQ(MESSAGE_ID_TRAJ, le16toh(header.messageID));
}