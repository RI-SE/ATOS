#include <gtest/gtest.h>

#include <string>

#include "mqtt2ros_utils.hpp"

TEST(Mqtt2RosNavSatParser, PayloadStringFullKeys) {
	std::string payload{"latitude: 1.0, longitude: 2.0, altitude: 3.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadStringShortKeys) {
	std::string payload{"lat: 1.0, lon: 2.0, alt: 3.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadStringMixedKeys) {
	std::string payload{"latitude: 1.0, longitude: 2.0, alt: 3.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadJsonFullKeys) {
	std::string payload{"{\"latitude\": 1.0, \"longitude\": 2.0, \"altitude\": 3.0}"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadJsonShortKeys) {
	std::string payload{"{\"lat\": 1.0, \"lon\": 2.0, \"alt\": 3.0}"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadJsonMixedKeys) {
	std::string payload{"{\"latitude\": 1.0, \"longitude\": 2.0, \"alt\": 3.0}"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadValuesIsInt) {
	std::string payload{"latitude: 1, longitude: 2, altitude: 3"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadHasSpaceSeparator) {
	std::string payload{"latitude: 1 longitude: 2 altitude: 3 "};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadContainsNegativeValues) {
	std::string payload{"lat: 1.0, lon: 2.0, alt: -3.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, -3.0);
}

TEST(Mqtt2RosNavSatParser, PayloadMissingSomeKeys) {
	std::string payload{"lat: 1.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 0.0);
	EXPECT_EQ(msg.altitude, 0.0);
}

TEST(Mqtt2RosNavSatParser, MixedJsonAndKeyValue) {
	std::string payload{"latitude: 1.0, \"longitude\": 2.0, alt: 3.0"};

	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}

TEST(Mqtt2RosNavSatParser, OrderIndependence) {
	std::string payload{"alt: 3.0, lat: 1.0, lon: 2.0"};
	const auto msg{Mqtt2RosUtils::mqtt2navsatfix(payload)};

	EXPECT_EQ(msg.latitude, 1.0);
	EXPECT_EQ(msg.longitude, 2.0);
	EXPECT_EQ(msg.altitude, 3.0);
}
