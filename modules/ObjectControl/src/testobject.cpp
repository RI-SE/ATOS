#include "testobject.hpp"
#include "util.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "osi_handler.hpp"
using namespace ATOS;

TestObject::TestObject(rclcpp::Logger log,
		std::shared_ptr<ROSChannels::Path::Sub> pathSub,
		std::shared_ptr<ROSChannels::Monitor::Pub> monrPub
	) :
	osiChannel(SOCK_STREAM, log),
	comms(log),
	Loggable(log),
	pathSub(pathSub),
	monrPub(monrPub)
	 {
}

TestObject::TestObject(TestObject&& other) :
	osiChannel(SOCK_STREAM, other.get_logger()),
	comms(other.get_logger()),
	state(other.state),
	conf(other.conf),
	lastMonitor(other.lastMonitor),
	maxAllowedMonitorPeriod(other.maxAllowedMonitorPeriod),
	Loggable(other.get_logger())
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

std::string TestObject::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->comms.cmd.addr.sin_addr, ipAddr, sizeof (ipAddr));
	std::string retval = "";
	retval += conf.toString(); // TODO add more
	return retval;
}

void TestObject::parseConfigurationFile(
		const fs::path &objectFile,
		bool getTrajFromRos) {
	struct sockaddr_in addr;
	this->conf.parseConfigurationFile(objectFile,getTrajFromRos);
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

void Channel::connect(
		std::shared_future<void> stopRequest,
		const std::chrono::milliseconds retryPeriod) {
	char ipString[INET_ADDRSTRLEN];
	std::stringstream errMsg;

	if (inet_ntop(AF_INET, &this->addr.sin_addr, ipString, sizeof (ipString))
			== nullptr) {
		errMsg << "inet_ntop: " << strerror(errno);
		throw std::invalid_argument(errMsg.str());
	}

	if (this->addr.sin_addr.s_addr == 0) {
		errMsg << "Invalid connection IP specified: " << ipString;
		throw std::invalid_argument(errMsg.str());
	}

	std::string type = "???";
	if (this->channelType == SOCK_STREAM) {
		type = "TCP";
	}
	else if (this->channelType == SOCK_DGRAM) {
		type = "UDP";
	}

	this->socket = ::socket(AF_INET, this->channelType, 0);
	if (this->socket < 0) {
		errMsg << "Failed to open " << type << " socket: " << strerror(errno);
		this->disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Begin connection attempt
	RCLCPP_INFO(get_logger(), "Attempting %s connection to %s:%u", type.c_str(), ipString,
			   ntohs(this->addr.sin_port));

	while (true) {
		if (::connect(this->socket,
					reinterpret_cast<struct sockaddr *>(&this->addr),
					sizeof (this->addr)) == 0) {
			break;
		}
		else {
			RCLCPP_ERROR(get_logger(), "Failed %s connection attempt to %s:%u, retrying in %.3f s ...",
					   type.c_str(), ipString, ntohs(this->addr.sin_port), retryPeriod.count() / 1000.0);
			if (stopRequest.wait_for(retryPeriod)
					!= std::future_status::timeout) {
				errMsg << "Connection attempt interrupted";
				throw std::runtime_error(errMsg.str());
			}
		}
	}
}

void Channel::disconnect() {
	if (this->socket != -1) {
		if (shutdown(this->socket, SHUT_RDWR) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket shutdown: %s", strerror(errno));
		}
		if (close(this->socket) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket close: %s", strerror(errno));
		}
		this->socket = -1;
	}
}

void ObjectConnection::connect(
		std::shared_future<void> stopRequest,
		const std::chrono::milliseconds retryPeriod) {
	try {
		this->cmd.connect(stopRequest, retryPeriod);
		this->mntr.connect(stopRequest, retryPeriod);
	} catch (std::runtime_error& e) {
		this->disconnect();
		throw std::runtime_error(std::string("Failed to establish ISO 22133 connection. Reason: \n") + e.what());
	}

	return;
}


bool ObjectConnection::isConnected() const {
	if (!isValid()) {
		return false;
	}
	pollfd fds[2];
	fds[0].fd = mntr.socket;
	fds[0].events = POLLIN | POLLOUT;
	fds[1].fd = cmd.socket;
	fds[1].events = POLLIN | POLLOUT;
	return poll(fds, 2, 0) >= 0;
}

bool ObjectConnection::isValid() const {
	return this->cmd.isValid() && this->mntr.isValid();
}

void ObjectConnection::disconnect() {
	this->cmd.disconnect();
	this->mntr.disconnect();
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
	objSettings.desiredID.transmitter = conf.getTransmitterID();
	objSettings.desiredID.controlCentre = ::getTransmitterID();
	objSettings.desiredID.subTransmitter = 0;

	objSettings.coordinateSystemOrigin = conf.getOrigin();
	objSettings.coordinateSystemType = COORDINATE_SYSTEM_WGS84;
	objSettings.coordinateSystemRotation_rad = 0.0;

	TimeSetToCurrentSystemTime(&objSettings.currentTime);
	
	objSettings.heabTimeout.tv_usec = 20000;
	objSettings.heabTimeout.tv_sec = 0;

	objSettings.rate.heab = 100;
	objSettings.rate.monr = 100;
	objSettings.rate.monr2 = 1;

	objSettings.maxDeviation.lateral_m = 5.0;
	objSettings.maxDeviation.position_m = 5.0;
	objSettings.maxDeviation.yaw_rad = 15.0 * M_PI/180.0;

	objSettings.minRequiredPositioningAccuracy_m = 1.0;

	objSettings.timeServer.ip = 0;
	objSettings.timeServer.port = 0;

	this->comms.cmd << objSettings;
	this->comms.cmd << conf.getTrajectory();
}

void TestObject::sendArm() {
	this->comms.cmd << OBJECT_COMMAND_ARM;
}

void TestObject::sendDisarm() {
	this->comms.cmd << OBJECT_COMMAND_DISARM;
}

void TestObject::sendAllClear() {
	this->comms.cmd << OBJECT_COMMAND_ALL_CLEAR;
}

void TestObject::sendRemoteControl(bool on) {
	
	this->comms.cmd << (on ? OBJECT_COMMAND_REMOTE_CONTROL : OBJECT_COMMAND_DISARM);
}

//void TestObject::sendOsiGlobalObjectGroundTruth(
//	GlobalObjectGroundTruth_t gogt) {
	//this->comms.osi << llls;
//}

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
	StartMessageType strt;
	strt.startTime.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(startTime.time_since_epoch()).count();
	strt.startTime.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(startTime.time_since_epoch()).count() % 1000000;
	strt.isTimestampValid = true;
	this->comms.cmd << strt;
}

void TestObject::sendControlSignal(const ControlSignalPercentage::SharedPtr csp) {
	this->comms.mntr << csp;
}

ISOMessageID Channel::pendingMessageType(bool awaitNext) {
	auto result = recv(this->socket, this->receiveBuffer.data(), this->receiveBuffer.size(), (awaitNext ? 0 : MSG_DONTWAIT) | MSG_PEEK);
	if (result < 0 && !awaitNext && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		return MESSAGE_ID_INVALID;
	}
	else if (result < 0) {
		throw std::runtime_error(std::string("Failed to check pending message type (recv: ")
									 + strerror(errno) + ")");
	}
	else if (result == 0) {
		throw std::runtime_error("Connection reset by peer");
	}
	else {
		ISOMessageID retval = getISOMessageType(this->receiveBuffer.data(), this->receiveBuffer.size(), false);
		if (retval == MESSAGE_ID_INVALID) {
			throw std::runtime_error("Non-ISO message received from " + this->remoteIP());
		}
		return retval;
	}
}

ISOMessageID ObjectConnection::pendingMessageType(bool awaitNext) {
	if (awaitNext) {
		if (!isValid()) {
			throw std::invalid_argument("Attempted to check pending message type for unconnected object");
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(mntr.socket, &fds);
		FD_SET(cmd.socket, &fds);
		auto result = select(std::max(mntr.socket,cmd.socket)+1,
							 &fds, nullptr, nullptr, nullptr);
		if (result < 0) {
			throw std::runtime_error(std::string("Failed socket operation (select: ") + strerror(errno) + ")"); // TODO clearer
		}
		else if (!isValid()) {
			throw std::invalid_argument("Connection invalidated during select call");
		}
		else if (FD_ISSET(mntr.socket, &fds)) {
			return this->mntr.pendingMessageType();
		}
		else if (FD_ISSET(cmd.socket, &fds)) {
			return this->cmd.pendingMessageType();
		}
		throw std::logic_error("Call to select returned unexpectedly: " + std::to_string(result));
	}
	else {
		auto retval = this->mntr.pendingMessageType();
		return retval != MESSAGE_ID_INVALID ? retval : this->cmd.pendingMessageType();
	}
}

Channel& operator<<(Channel& chnl, const HeabMessageDataType& heartbeat) {
	auto nBytes = encodeHEABMessage(&heartbeat.dataTimestamp, heartbeat.controlCenterStatus,
									chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode HEAB message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send HEAB: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ObjectSettingsType& settings) {
	auto nBytes = encodeOSEMMessage(&settings, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode OSEM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send OSEM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const Trajectory& traj) {
	ssize_t nBytes;

	// TRAJ header
	nBytes = encodeTRAJMessageHeader(
				traj.id, TRAJECTORY_INFO_RELATIVE_TO_ORIGIN, traj.name.c_str(),traj.name.length(),
				static_cast<uint32_t>(traj.points.size()), chnl.transmitBuffer.data(),
				chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send TRAJ message header: ") + strerror(errno));
	}

	// TRAJ points
	for (const auto& pt : traj.points) {
		struct timeval relTime;
		CartesianPosition pos = pt.getISOPosition();
		SpeedType spd = pt.getISOVelocity();
		AccelerationType acc = pt.getISOAcceleration();

		relTime = to_timeval(pt.getTime());

		nBytes = encodeTRAJMessagePoint(&relTime, pos, spd, acc, static_cast<float>(pt.getCurvature()),
										chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
		if (nBytes < 0) {
			// TODO what to do here?
			throw std::invalid_argument(std::string("Failed to encode TRAJ message point: ") + strerror(errno));
		}
		nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);

		if (nBytes < 0) {
			// TODO what to do here?
			throw std::runtime_error(std::string("Failed to send TRAJ message point: ") + strerror(errno));
		}
	}

	// TRAJ footer
	nBytes = encodeTRAJMessageFooter(chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message footer: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send TRAJ message footer: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ObjectCommandType& cmd) {
	auto nBytes = encodeOSTMMessage(cmd, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode OSTM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send OSTM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const StartMessageType& strt) {
	auto nBytes = encodeSTRTMessage(&strt, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode STRT message: ") + strerror(errno));
	}

	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send STRT: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator>>(Channel& chnl, MonitorMessage& monitor) {
	if (chnl.pendingMessageType() == MESSAGE_ID_MONR) {
		struct timeval tv;
		TimeSetToCurrentSystemTime(&tv);
		auto nBytes = decodeMONRMessage(chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), tv,
										&monitor.first, &monitor.second, false);
		if (nBytes < 0) {
			throw std::invalid_argument("Failed to decode MONR message");
		}
		else {
			nBytes = recv(chnl.socket, chnl.receiveBuffer.data(), static_cast<size_t>(nBytes), 0);
			if (nBytes <= 0) {
				throw std::runtime_error("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}

Channel& operator>>(Channel& chnl, ObjectPropertiesType& prop) {
	if (chnl.pendingMessageType() == MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO) {
		auto nBytes = decodeOPROMessage(&prop, chnl.receiveBuffer.data(), chnl.receiveBuffer.size(), false);
		if (nBytes < 0) {
			throw std::invalid_argument(strerror(errno));
		}
		else {
			nBytes = recv(chnl.socket, chnl.receiveBuffer.data(), static_cast<size_t>(nBytes), 0);
			if (nBytes <= 0) {
				throw std::runtime_error("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const ControlSignalPercentage::SharedPtr csp) {
	RemoteControlManoeuvreMessageType rcmm;
	rcmm.command = MANOEUVRE_NONE;
	rcmm.isThrottleManoeuvreValid = true;
	rcmm.isBrakeManoeuvreValid = true;
	rcmm.isSteeringManoeuvreValid = true;
	rcmm.throttleUnit = ISO_UNIT_TYPE_THROTTLE_PERCENTAGE;
	rcmm.brakeUnit = ISO_UNIT_TYPE_BRAKE_PERCENTAGE;
	rcmm.steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
	rcmm.throttleManoeuvre.pct = csp->throttle;
	rcmm.brakeManoeuvre.pct = csp->brake;
	rcmm.steeringManoeuvre.pct = csp->steering_angle;
	auto nBytes = encodeRCMMMessage(&rcmm, chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode RCM message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send RCM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const std::vector<char>& data) {
	auto nBytes = send(chnl.socket, data.data(), data.size(), 0);
	if (nBytes < 0) {
		throw std::runtime_error(std::string("Failed to send raw data: ") + strerror(errno));
	}
	return chnl;
}

std::string Channel::remoteIP() const {
	char ipString[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->addr.sin_addr, ipString, sizeof (ipString));
	return std::string(ipString);
}