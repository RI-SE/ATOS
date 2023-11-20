/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "testobject.hpp"
#include "util.h"
#include "atosTime.h"
#include "osi_handler.hpp"
#include "journal.hpp"

using namespace ATOS;
using std::placeholders::_1;

TestObject::TestObject(uint32_t id) :
	Node("object_" + std::to_string(id)),
	comms(get_logger(), id),
	osiChannel(SOCK_STREAM, get_logger()),
	conf(get_logger())
	 {
		pathSub = std::make_shared<ROSChannels::Path::Sub>(*this, id, std::bind(&TestObject::onPathMessage, this, _1, id));
		monrPub = std::make_shared<ROSChannels::Monitor::Pub>(*this, id);
		navSatFixPub = std::make_shared<ROSChannels::NavSatFix::Pub>(*this, id);
		stateChangePub = std::make_shared<ROSChannels::ObjectStateChange::Pub>(*this);
}
void TestObject::onPathMessage(const ROSChannels::Path::message_type::SharedPtr, int){
	;
}

TestObject::TestObject(TestObject&& other) :
	rclcpp::Node(other.get_name()),
	comms(other.get_logger(), other.getTransmitterID()),
	osiChannel(SOCK_STREAM, other.get_logger()),
	state(other.state),
	conf(other.conf),
	lastMonitor(other.lastMonitor),
	maxAllowedMonitorPeriod(other.maxAllowedMonitorPeriod)
{
	this->comms = other.comms;
	this->osiChannel = other.osiChannel;
	other.comms.cmd.socket = 0;
	other.comms.mntr.socket = 0;
	other.osiChannel.socket = 0;
	this->nextMonitor = std::move(other.nextMonitor);
}

void TestObject::setObjectIP(
	const in_addr_t newIP) {
	struct sockaddr_in addr;
	addr.sin_addr.s_addr = newIP;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(ISO_22133_DEFAULT_OBJECT_TCP_PORT);
	this->setCommandAddress(addr);
	addr.sin_port = htons(ISO_22133_OBJECT_UDP_PORT);
	this->setMonitorAddress(addr);
	addr.sin_port = htons(OSI_DEFAULT_OBJECT_TCP_PORT);
	this->setOsiAddress(addr);
}

void TestObject::setOrigin(const GeographicPositionType& pos){
	this->conf.setOrigin(pos);
}

void TestObject::setCommandAddress(
		const sockaddr_in &newAddr) {
	if (!this->comms.isConnected()) {
		this->comms.cmd.addr = newAddr;
	}
}

void TestObject::setMonitorAddress(
		const sockaddr_in &newAddr) {
	if (!this->comms.isConnected()) {
		this->comms.mntr.addr = newAddr;
	}
}

void TestObject::setOsiAddress(
		const sockaddr_in &newAddr) {
	if (!this->comms.isConnected()) {
		this->osiChannel.addr = newAddr;
	}
}

void TestObject::setObjectConfig(
		ObjectConfig& newObjectConfig) {
	this->conf = newObjectConfig;
}

void TestObject::setTriggerStart(
		const bool startOnTrigger) {
	auto st = this->getState();
	if (st != OBJECT_STATE_ARMED && 
		st != OBJECT_STATE_RUNNING &&
		st != OBJECT_STATE_POSTRUN &&
		st != OBJECT_STATE_ABORTING &&
    	st != OBJECT_STATE_REMOTE_CONTROL &&
    	st != OBJECT_STATE_PRE_ARMING &&
    	st != OBJECT_STATE_PRE_RUNNING) {
		this->startOnTrigger = startOnTrigger;
	}
	else {
		std::stringstream errMsg;
		errMsg << "Attempted to change trigger configuration while in state ";
		errMsg << objectStateToASCII(st);
		throw std::invalid_argument(errMsg.str());
	}
}

void TestObject::setLastReceivedPath(ROSChannels::Path::message_type::SharedPtr trajlet){
	// Warning: ensure thread safety here if other thread uses
	// lastReceivedPath.
	this->lastReceivedPath = trajlet;
}

ObjectDataType TestObject::getAsObjectData() const {
	ObjectDataType retval;
	// TODO check on isValid for each
	retval.origin.Latitude = this->conf.getOrigin().latitude_deg;
	retval.origin.Longitude = this->conf.getOrigin().longitude_deg;
	retval.origin.Altitude = this->conf.getOrigin().altitude_m;

	retval.Enabled = OBJECT_ENABLED; // TODO maybe a setting for this
	retval.ClientIP = this->comms.cmd.addr.sin_addr.s_addr;
	retval.ClientID =  this->getTransmitterID();
	auto diff = this->getTimeSinceLastMonitor();
	if (diff.count() == 0) {
		retval.lastPositionUpdate = {0, 0};
	}
	else {
		struct timeval tvnow, tvdiff;
		TimeSetToCurrentSystemTime(&tvnow);
		tvdiff = to_timeval(diff);
		timersub(&tvnow, &tvdiff, &retval.lastPositionUpdate);
	}
	retval.propertiesReceived = false; // TODO once OPRO is parsed

	return retval;
}

MonitorMessage TestObject::awaitNextMonitor() {
	MonitorMessage retval;
	// Read until receive buffer is empty, return last read message
	if (this->comms.mntr.pendingMessageType(true) == MESSAGE_ID_MONR) {
		this->comms.mntr >> retval;
		return retval;
	}
	else {
		throw std::invalid_argument("Received non-MONR message on MONR channel");
	}
}

void TestObject::handleISOMessage(bool awaitNext) {
	auto message = this->comms.pendingMessageType(awaitNext);
	switch (message) {
	case MESSAGE_ID_MONR: 
		{
			struct timeval currentTime;
			auto prevObjState = this->getState();
			auto monr = this->readMonitorMessage();
			TimeSetToCurrentSystemTime(&currentTime);
			this->publishMonitor(monr);
			if (this->getState() != prevObjState) {
				this->publishStateChange(prevObjState);
			}
		}
		break;
	case MESSAGE_ID_TREO:
		RCLCPP_WARN(get_logger(), "Unhandled TREO message");
		break;
	case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO:
		this->parseObjectPropertyMessage();
		break;
	case MESSAGE_ID_GREM:
		this->parseGremMessage();
		break;
	default:
		RCLCPP_WARN(get_logger(), "Received unknown message type");
		break;
	}
}

void TestObject::updateMonitor(const MonitorMessage& data) {
	if (data.first != this->getTransmitterID()) {
		throw std::invalid_argument("Attempted to set monitor data with non-matching transmitter ID ("
									+ std::to_string(data.first) + " != " + std::to_string(this->getTransmitterID()) + ")");
	}
	this->state = data.second.state;
	this->lastMonitor = data.second;
}

ObjectStateType TestObject::getState(const bool awaitUpdate) {
	return getState(awaitUpdate, maxAllowedMonitorPeriod);
}

ObjectStateType TestObject::getState(
		const bool awaitUpdate,
		const std::chrono::milliseconds timeout) {
	if (awaitUpdate) {
		if (!nextMonitor.valid()) {
			nextMonitor = std::async(std::launch::async, &TestObject::awaitNextMonitor, this);
		}
		auto status = nextMonitor.wait_for(timeout);
		if (status == std::future_status::ready) {
			this->updateMonitor(nextMonitor.get());
		}
		else {
			throw std::runtime_error("Timed out while waiting for monitor data");
		}
	}
	return this->getState();
}

void TestObject::publishMonr(const ROSChannels::Monitor::message_type monr){
	monrPub->publish(monr);
}

void TestObject::publishNavSatFix(const ROSChannels::NavSatFix::message_type navSatFix){
	navSatFixPub->publish(navSatFix);
}

std::string TestObject::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->comms.cmd.addr.sin_addr, ipAddr, sizeof (ipAddr));
	std::string retval = "";
	retval += conf.toString(); // TODO add more
	return retval;
}

void TestObject::parseConfigurationFile(
		const fs::path &objectFile) {
	struct sockaddr_in addr;
	this->conf.parseConfigurationFile(objectFile);
	addr.sin_addr.s_addr = conf.getIP();
	addr.sin_family = AF_INET;
	addr.sin_port = htons(ISO_22133_DEFAULT_OBJECT_TCP_PORT);
	this->setCommandAddress(addr);
	addr.sin_port = htons(ISO_22133_OBJECT_UDP_PORT);
	this->setMonitorAddress(addr);
	addr.sin_port = htons(OSI_DEFAULT_OBJECT_TCP_PORT);
	this->setOsiAddress(addr);
}

void TestObject::establishConnection(std::shared_future<void> stopRequest) {
	this->lastMonitorTime = std::chrono::steady_clock::time_point(); // reset
	this->comms.connect(stopRequest, TestObject::connRetryPeriod);

	if (this->isOsiCompatible()) {
		try {
			this->osiChannel.connect(stopRequest, TestObject::connRetryPeriod);
		}
		catch (std::runtime_error& e) {
			this->osiChannel.disconnect();
			throw std::runtime_error(std::string("Failed to establish OSI connection. Reason: \n") + e.what());
		}
	}
}


void TestObject::sendHeartbeat(
		const ControlCenterStatusType ccStatus) {
	HeabMessageDataType heartbeat;
	TimeSetToCurrentSystemTime(&heartbeat.dataTimestamp);
	heartbeat.controlCenterStatus = ccStatus;
	this->comms.mntr << heartbeat;
}

void TestObject::sendSettings() {

	ObjectSettingsType objSettings;
	objSettings.testMode = TEST_MODE_PREPLANNED;

	objSettings.desiredID.transmitter = conf.getTransmitterID();
	objSettings.desiredID.controlCentre = 0;
	objSettings.desiredID.subTransmitter = 0;

	objSettings.coordinateSystemOrigin = conf.getOrigin();
	objSettings.coordinateSystemType = COORDINATE_SYSTEM_WGS84;
	objSettings.coordinateSystemRotation_rad = 0.0;

	TimeSetToCurrentSystemTime(&objSettings.currentTime);
	
	objSettings.heabTimeout.tv_usec = 20000;
	objSettings.heabTimeout.tv_sec = 0;

	objSettings.rate.heab = 10;
	objSettings.rate.monr = 100;
	objSettings.rate.monr2 = 1;

	objSettings.maxDeviation.lateral_m = 5.0;
	objSettings.maxDeviation.position_m = 5.0;
	objSettings.maxDeviation.yaw_rad = 15.0 * M_PI/180.0;

	objSettings.minRequiredPositioningAccuracy_m = 1.0;

	objSettings.timeServer.ip = 0;
	objSettings.timeServer.port = 0;

	this->comms.cmd << objSettings;
	this->sendTrajectory();
}

void TestObject::sendTrajectory() {
	this->comms.cmd << conf.getTrajectory();
}

void TestObject::sendArm() {
	RCLCPP_INFO(get_logger(), "Arming object %d", conf.getTransmitterID());
	this->comms.cmd << OBJECT_COMMAND_ARM;
}

void TestObject::sendDisarm() {
	RCLCPP_INFO(get_logger(), "Disarming object %d", conf.getTransmitterID());
	this->comms.cmd << OBJECT_COMMAND_DISARM;
}

void TestObject::sendAllClear() {
	RCLCPP_INFO(get_logger(), "Clearing abort for object %d", conf.getTransmitterID());
	this->comms.cmd << OBJECT_COMMAND_ALL_CLEAR;
}

void TestObject::sendRemoteControl(bool on) {
	this->comms.cmd << (on ? OBJECT_COMMAND_REMOTE_CONTROL : OBJECT_COMMAND_DISARM);
}

void TestObject::sendOsiData(
		const OsiHandler::LocalObjectGroundTruth_t& osidata,
		const std::string& projStr,
		const std::chrono::system_clock::time_point& timestamp) {
	OsiHandler osi;
	auto rawData = osi.encodeSvGtMessage(osidata, timestamp, projStr, false);
	std::vector<char> vec(rawData.length());
	std::copy(rawData.begin(), rawData.end(), vec.begin());
	this->osiChannel << vec;
}

void TestObject::sendStart(std::chrono::system_clock::time_point startTime) {
	RCLCPP_INFO(get_logger(), "Starting object %d", conf.getTransmitterID());
	StartMessageType strt;
	strt.startTime.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(startTime.time_since_epoch()).count();
	strt.startTime.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(startTime.time_since_epoch()).count() % 1000000;
	strt.isTimestampValid = true;
	this->comms.cmd << strt;
}

void TestObject::sendControlSignal(const ControlSignalPercentage::SharedPtr csp) {
	this->comms.mntr << csp;
}

void TestObject::publishMonitor(MonitorMessage& monr){
	// Publish to journal
	auto objData = this->getAsObjectData();
	objData.MonrData = monr.second;
	JournalRecordMonitorData(&objData);

	// Publish to ROS topic
	auto rosMonr = ROSChannels::Monitor::fromISOMonr(monr.first,monr.second);
	publishMonr(rosMonr);

	// TODO: Make a translator node that listens on Monitor topic and does this..
	auto origin = this->getOrigin();
	std::array<double,3> llh_0 = {origin.latitude_deg, origin.longitude_deg, origin.altitude_m};
	publishNavSatFix(ROSChannels::NavSatFix::fromROSMonr(llh_0, rosMonr));
}

void TestObject::publishStateChange(ObjectStateType &prevObjState){
	ROSChannels::ObjectStateChange::message_type msg;
	msg.id = this->getTransmitterID();
	msg.prev_state.state = prevObjState;
	msg.state.state = this->getState();
	
	stateChangePub->publish(msg);
}