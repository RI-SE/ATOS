#include "truckobjectcontrol.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

using json = nlohmann::json;
using KeyValues = std::map<std::string, std::string>;

namespace {
constexpr double kEarthRadiusM	 = 6378137.0;
constexpr double kPathLengthM	 = 3000.0;
constexpr double kVertexSpacingM = 10.0;

// Margin around the 7.5 m / 15.0 m limits; exact values are not reproducible through lat/lon.
constexpr double kJustOnPathM	= 7.49;
constexpr double kJustOffPathM	= 7.51;
constexpr double kJustNearPathM = 14.99;
constexpr double kJustTooFarM	= 15.01;

struct TruckSpec {
	std::string uid;
	double along_m;
	double lateral_m;
};

double metersToLatDeg(const double meters) {
	return meters / kEarthRadiusM * 180.0 / M_PI;
}

// Straight north-going path along lon=0, so along-track metres map linearly to latitude.
void writePathFile(const std::string& path) {
	std::ostringstream coordinates;
	coordinates << std::setprecision(15);
	const int points = static_cast<int>(kPathLengthM / kVertexSpacingM) + 1;
	for (int i = 0; i < points; ++i) {
		coordinates << (i > 0 ? "," : "") << "[0," << metersToLatDeg(i * kVertexSpacingM) << "]";
	}
	std::ofstream out(path);
	out << R"({"type":"FeatureCollection","features":[{"type":"Feature","id":"PathSection",)"
		<< R"("geometry":{"type":"LineString","coordinates":[)" << coordinates.str() << "]}}]}";
}

std::string makeCotXml(const TruckSpec& truck) {
	const double lat_deg = metersToLatDeg(truck.along_m);
	const double lon_deg = truck.lateral_m / (kEarthRadiusM * std::cos(lat_deg * M_PI / 180.0)) * 180.0 / M_PI;
	std::ostringstream xml;
	xml << std::setprecision(12) << "<event version=\"2.0\" uid=\"" << truck.uid << "\" type=\"a-f-G-U-C\">"
		<< "<point lat=\"" << lat_deg << "\" lon=\"" << lon_deg << "\" hae=\"0\" ce=\"0\" le=\"0\"/>"
		<< "<detail><track speed=\"36\" course=\"0\"/></detail></event>";
	return xml.str();
}

KeyValues parseKeyValues(const std::string& text) {
	KeyValues out;
	std::stringstream stream(text);
	std::string token;
	while (std::getline(stream, token, ';')) {
		const auto sep = token.find('=');
		if (sep != std::string::npos) {
			out[token.substr(0, sep)] = token.substr(sep + 1);
		}
	}
	return out;
}
} // namespace

class TruckObjectControlTest : public ::testing::Test {
public:
	static void SetUpTestSuite() {
		s_pathFile = (std::filesystem::temp_directory_path() /
					  ("truck_object_control_test_" + std::to_string(::getpid()) + ".geojson"))
					   .string();
		writePathFile(s_pathFile);

		// Global ROS args are applied as parameter overrides to the node under test.
		std::vector<std::string> args{"truck_object_control_test",
									  "--ros-args",
									  "-p",
									  "trajectory_geojson_path:=" + s_pathFile,
									  "-p",
									  "cot_tcp_port:=0"};
		std::vector<char*> argv;
		for (auto& arg : args) {
			argv.push_back(arg.data());
		}
		rclcpp::init(static_cast<int>(argv.size()), argv.data());
	}

	static void TearDownTestSuite() {
		rclcpp::shutdown();
		std::filesystem::remove(s_pathFile);
	}

protected:
	std::shared_ptr<TruckObjectControl> node;
	std::shared_ptr<rclcpp::Node> helper;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cotPub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stateSub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fleetSub;
	rclcpp::executors::SingleThreadedExecutor executor;

	std::map<std::string, json> states;
	KeyValues fleetCommand;

	void SetUp() override {
		node   = std::make_shared<TruckObjectControl>();
		helper = rclcpp::Node::make_shared("truck_object_control_test_helper");

		cotPub	 = helper->create_publisher<std_msgs::msg::String>("truck_objects/cot", 50);
		stateSub = helper->create_subscription<std_msgs::msg::String>(
		  "truck_objects/state", 100, [this](std_msgs::msg::String::ConstSharedPtr msg) {
			  const json payload = json::parse(msg->data);
			  states[payload.at("uid").get<std::string>()] = payload;
		  });
		fleetSub = helper->create_subscription<std_msgs::msg::String>(
		  "truck_objects/speed_command", 100, [this](std_msgs::msg::String::ConstSharedPtr msg) {
			  fleetCommand = parseKeyValues(msg->data);
		  });

		executor.add_node(node);
		executor.add_node(helper);

		ASSERT_TRUE(spinUntil([this]() {
			return cotPub->get_subscription_count() > 0 && stateSub->get_publisher_count() > 0 &&
				   fleetSub->get_publisher_count() > 0;
		})) << "Test helper never connected to the node";
	}

	void TearDown() override {
		executor.remove_node(node);
		executor.remove_node(helper);
		fleetSub.reset();
		stateSub.reset();
		cotPub.reset();
		helper.reset();
		node.reset();
	}

	bool spinUntil(const std::function<bool()>& done,
				   const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
		const auto deadline = std::chrono::steady_clock::now() + timeout;
		while (std::chrono::steady_clock::now() < deadline) {
			executor.spin_some();
			if (done()) {
				return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		return done();
	}

	// Publishes the trucks and lets the node run several 200 ms evaluation cycles on the static input.
	void place(const std::vector<TruckSpec>& trucks) {
		std::map<std::string, double> expected_lat;
		for (const auto& truck : trucks) {
			std_msgs::msg::String msg;
			msg.data = makeCotXml(truck);
			cotPub->publish(msg);
			expected_lat[truck.uid] = metersToLatDeg(truck.along_m);
		}

		ASSERT_TRUE(spinUntil([&]() {
			for (const auto& [uid, lat] : expected_lat) {
				const auto it = states.find(uid);
				if (it == states.end() || std::abs(it->second.at("lat").get<double>() - lat) > 1e-9) {
					return false;
				}
			}
			return true;
		})) << "Node never reported the published truck positions";

		spinUntil([]() { return false; }, std::chrono::milliseconds(1200));
	}

	bool hasCommand(const std::string& uid) const {
		const auto it = states.find(uid);
		return it != states.end() && !it->second.at("last_tcp_command").get<std::string>().empty();
	}

	KeyValues commandFor(const std::string& uid) const {
		const auto it = states.find(uid);
		if (it == states.end()) {
			return {};
		}
		return parseKeyValues(it->second.at("last_tcp_command").get<std::string>());
	}

	static std::string s_pathFile;
};

std::string TruckObjectControlTest::s_pathFile;

// Behaviour with on-path trucks only must be unchanged.

TEST_F(TruckObjectControlTest, onPathTruckAheadAt400mGivesSlowdown) {
	place({{"A", 500.0, 0.0}, {"B", 900.0, 0.0}});

	const auto a = commandFor("A");
	EXPECT_EQ(a.at("command"), "SLOWDOWN");
	EXPECT_EQ(a.at("reason"), "truck_ahead_slowdown_state");
	EXPECT_EQ(a.at("truck_ahead_uid"), "B");
	EXPECT_EQ(a.at("truck_ahead_near_path"), "0");
	EXPECT_EQ(a.at("connected_count"), "2");
	EXPECT_EQ(commandFor("B").at("command"), "DRIVE");
}

TEST_F(TruckObjectControlTest, onPathTruckAheadAt150mGivesStop) {
	place({{"A", 500.0, 0.0}, {"B", 650.0, 0.0}});

	const auto a = commandFor("A");
	EXPECT_EQ(a.at("command"), "STOP");
	EXPECT_EQ(a.at("reason"), "truck_ahead_below_stop_distance");
	EXPECT_EQ(fleetCommand.at("reason"), "min_gap_below_stop_distance");
}

TEST_F(TruckObjectControlTest, singleOnPathTruckDrives) {
	place({{"A", 500.0, 0.0}});

	EXPECT_EQ(commandFor("A").at("command"), "DRIVE");
	EXPECT_EQ(commandFor("A").at("truck_ahead_uid"), "none");
}

// Near-path trucks cause SLOWDOWN only.

TEST_F(TruckObjectControlTest, nearPathTruckAt400mAheadGivesSlowdown) {
	place({{"A", 500.0, 0.0}, {"N", 900.0, 10.0}});

	const auto a = commandFor("A");
	EXPECT_EQ(a.at("command"), "SLOWDOWN");
	EXPECT_EQ(a.at("reason"), "truck_ahead_near_path_slowdown");
	EXPECT_EQ(a.at("truck_ahead_uid"), "N");
	EXPECT_EQ(a.at("truck_ahead_near_path"), "1");
	EXPECT_NEAR(std::stod(a.at("distance_to_truck_ahead_m")), 400.0, 15.0);
	EXPECT_EQ(a.at("connected_count"), "1");
}

TEST_F(TruckObjectControlTest, nearPathTruckAt100mAheadGivesSlowdownNotStop) {
	place({{"A", 500.0, 0.0}, {"N", 600.0, 10.0}});

	EXPECT_EQ(commandFor("A").at("command"), "SLOWDOWN");
	EXPECT_EQ(commandFor("A").at("reason"), "truck_ahead_near_path_slowdown");
	EXPECT_EQ(fleetCommand.at("reason"), "min_gap_below_warning_distance");
}

TEST_F(TruckObjectControlTest, nearPathTruckReceivesNoCommand) {
	place({{"A", 500.0, 0.0}, {"N", 900.0, 10.0}});

	EXPECT_TRUE(hasCommand("A"));
	EXPECT_FALSE(hasCommand("N"));
}

TEST_F(TruckObjectControlTest, nearPathTruckFarAheadIsIgnored) {
	place({{"A", 500.0, 0.0}, {"N", 1500.0, 10.0}});

	EXPECT_EQ(commandFor("A").at("command"), "DRIVE");
}

TEST_F(TruckObjectControlTest, nearPathTruckAheadAcrossPathWrapGivesSlowdown) {
	place({{"A", 2800.0, 0.0}, {"N", 100.0, 10.0}});

	const auto a = commandFor("A");
	EXPECT_EQ(a.at("command"), "SLOWDOWN");
	EXPECT_EQ(a.at("truck_ahead_uid"), "N");
	EXPECT_NEAR(std::stod(a.at("distance_to_truck_ahead_m")), 300.0, 15.0);
}

TEST_F(TruckObjectControlTest, nearPathTruckBetweenOnPathTrucksDoesNotHideStop) {
	place({{"A", 500.0, 0.0}, {"N", 600.0, 10.0}, {"C", 650.0, 0.0}});

	const auto a = commandFor("A");
	EXPECT_EQ(a.at("command"), "STOP");
	EXPECT_EQ(a.at("truck_ahead_uid"), "C");
	EXPECT_EQ(a.at("truck_ahead_near_path"), "0");
}

TEST_F(TruckObjectControlTest, nearPathTruckDoesNotReleaseStopCausedByOnPathTruck) {
	place({{"A", 500.0, 0.0}, {"B", 600.0, 0.0}, {"N", 1100.0, 10.0}});

	EXPECT_EQ(commandFor("A").at("command"), "STOP");
	EXPECT_EQ(commandFor("A").at("truck_ahead_uid"), "B");
}

TEST_F(TruckObjectControlTest, onlyNearPathTrucksGiveNoCommands) {
	place({{"N1", 500.0, 10.0}, {"N2", 600.0, 10.0}});

	EXPECT_FALSE(hasCommand("N1"));
	EXPECT_FALSE(hasCommand("N2"));
}

// Lateral distance limits.

TEST_F(TruckObjectControlTest, truckJustInsideOnPathLimitIsCommandedWithFullRules) {
	place({{"A", 500.0, 0.0}, {"B", 650.0, kJustOnPathM}});

	EXPECT_EQ(commandFor("A").at("command"), "STOP");
	EXPECT_TRUE(hasCommand("B"));
}

TEST_F(TruckObjectControlTest, truckJustOutsideOnPathLimitIsNearPath) {
	place({{"A", 500.0, 0.0}, {"B", 650.0, kJustOffPathM}});

	EXPECT_EQ(commandFor("A").at("command"), "SLOWDOWN");
	EXPECT_EQ(commandFor("A").at("truck_ahead_near_path"), "1");
	EXPECT_FALSE(hasCommand("B"));
}

TEST_F(TruckObjectControlTest, truckJustInsideNearPathLimitCounts) {
	place({{"A", 500.0, 0.0}, {"N", 650.0, kJustNearPathM}});

	EXPECT_EQ(commandFor("A").at("command"), "SLOWDOWN");
}

TEST_F(TruckObjectControlTest, truckJustBeyondNearPathLimitIsIgnored) {
	place({{"A", 500.0, 0.0}, {"N", 650.0, kJustTooFarM}});

	EXPECT_EQ(commandFor("A").at("command"), "DRIVE");
	EXPECT_FALSE(hasCommand("N"));
}

// Hysteresis for near-path trucks follows the SLOWDOWN -> DRIVE rule (release above 700 m).

TEST_F(TruckObjectControlTest, nearPathSlowdownIsHeldUntilGapExceeds700m) {
	place({{"A", 500.0, 0.0}, {"N", 900.0, 10.0}});
	EXPECT_EQ(commandFor("A").at("command"), "SLOWDOWN");

	place({{"N", 1100.0, 10.0}});
	EXPECT_EQ(commandFor("A").at("command"), "SLOWDOWN");

	place({{"N", 1300.0, 10.0}});
	EXPECT_EQ(commandFor("A").at("command"), "DRIVE");
}

TEST_F(TruckObjectControlTest, nearPathTruckBetween500mAnd700mDoesNotStartSlowdown) {
	place({{"A", 500.0, 0.0}, {"N", 1100.0, 10.0}});

	EXPECT_EQ(commandFor("A").at("command"), "DRIVE");
}
