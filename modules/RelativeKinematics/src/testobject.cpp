#include "testobject.hpp"
#include "util.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "osi_handler.hpp"

TestObject::TestObject() {
	// TODO
	origin.latitude_deg = origin.longitude_deg = origin.altitude_m = 0.0;
	origin.isLongitudeValid = origin.isLatitudeValid = origin.isAltitudeValid = false;

}

TestObject::TestObject(TestObject&& other) :
	state(other.state),
	objectFile(other.objectFile),
	trajectoryFile(other.trajectoryFile),
	transmitterID(other.transmitterID),
	isAnchorObject(other.isAnchorObject),
	trajectory(other.trajectory),
	origin(other.origin),
	lastMonitor(other.lastMonitor),
	maxAllowedMonitorPeriod(other.maxAllowedMonitorPeriod)
{
	this->comms = other.comms;
	other.comms.cmd.socket = 0;
	other.comms.mntr.socket = 0;
	this->nextMonitor = std::move(other.nextMonitor);
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
		this->comms.osi.addr = newAddr;
	}
}

/*
void TestObject::setObjectConfig(ObjectConfig& newObjectConfig){
	this->objectConfig = newObjectConfig; }
*/

ObjectDataType TestObject::getAsObjectData() const {
	ObjectDataType retval;
	// TODO check on isValid for each
	retval.origin.Latitude = this->origin.latitude_deg;
	retval.origin.Longitude = this->origin.longitude_deg;
	retval.origin.Altitude = this->origin.altitude_m;

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
		to_timeval(diff, tvdiff);
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

std::string TestObject::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->comms.cmd.addr.sin_addr, ipAddr, sizeof (ipAddr));
	std::string retval = "";
	retval += "Object ID " + std::to_string(transmitterID)
			+ ", IP " + ipAddr + ", trajectory file " + trajectoryFile.filename().string()
			+ ", object file " + objectFile.filename().string() + ", anchor: " + (isAnchorObject?"Yes":"No");
	return retval;
}

void TestObject::parseConfigurationFile(
		const fs::path &objectFile) {

	char setting[100];
	int result;
	struct sockaddr_in addr;
	char path[MAX_FILE_PATH];

	UtilGetTrajDirectoryPath(path, sizeof (path));

	// Get IP setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, objectFile.c_str(),
								 objectFile.string().length()+1, setting,
								 sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find IP setting in file " + objectFile.string());
	}
	result = inet_pton(AF_INET, setting, &addr.sin_addr);
	if (result == -1) {
		using namespace std;
		throw system_error(make_error_code(static_cast<errc>(errno)));
	}
	else if (result == 0) {
		throw std::invalid_argument("Address " + std::string(setting)
									+ " in object file " + objectFile.string()
									+ " is not a valid IPv4 address");
	}
	addr.sin_family = AF_INET;
	addr.sin_port = htons(ISO_22133_DEFAULT_OBJECT_TCP_PORT);
	this->setCommandAddress(addr);
	addr.sin_port = htons(ISO_22133_OBJECT_UDP_PORT);
	this->setMonitorAddress(addr);
	addr.sin_port = htons(OSI_DEFAULT_OBJECT_TCP_PORT);
	this->setOsiAddress(addr);

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find trajectory setting in file " + objectFile.string());
	}

	fs::path trajFile(std::string(path) + std::string(setting));
	if (!fs::exists(trajFile)) {
		throw std::invalid_argument("Configured trajectory file " + std::string(setting)
									+ " in file " + objectFile.string() + " not found");
	}
	this->trajectoryFile = trajFile;

	// Get ID setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find ID setting in file " + objectFile.string());
	}
	char *endptr;
	uint32_t id = static_cast<uint32_t>(strtoul(setting, &endptr, 10));

	if (endptr == setting) {
		throw std::invalid_argument("ID " + std::string(setting) + " in file "
									+ objectFile.string() + " is invalid");
	}
	this->transmitterID = id;

	// Get anchor setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IS_ANCHOR, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		this->isAnchorObject = false;
	}
	else {
		if (setting[0] == '1' || setting[0] == '0') {
			this->isAnchorObject = setting[0] == '1';
		}
		else {
			std::string anchorSetting(setting);
			for (char &c : anchorSetting) {
				c = std::tolower(c, std::locale());
			}
			if (anchorSetting.compare("true") == 0) {
				this->isAnchorObject = true;
			}
			else if (anchorSetting.compare("false") == 0) {
				this->isAnchorObject = false;
			}
			else {
				throw std::invalid_argument("Anchor setting " + std::string(setting) + " in file "
											+ objectFile.string() + " is invalid");
			}
		}
	}

	// Get OSI compatible setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IS_OSI_COMPATIBLE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		this->isOsiObject = false;
	}
	else {
		if (setting[0] == '1' || setting[0] == '0') {
			this->isOsiObject = setting[0] == '1';
		}
		else {
			std::string osiSetting(setting);
			for (char &c : osiSetting) {
				c = std::tolower(c, std::locale());
			}
			if (osiSetting.compare("true") == 0) {
				this->isOsiObject = true;
			}
			else if (osiSetting.compare("false") == 0) {
				this->isOsiObject = false;
			}
			else {
				throw std::invalid_argument("OSI setting " + std::string(setting) + " in file "
											+ objectFile.string() + " is invalid");
			}
		}
	}

	this->objectFile = objectFile;

	this->origin = {};
	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_LATITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.latitude_deg = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isLatitudeValid = true;
		}
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_LONGITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.longitude_deg = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isLongitudeValid = true;
		}
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ORIGIN_ALTITUDE, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) != -1) {
		origin.altitude_m = strtod(setting, &endptr);
		if (setting != endptr) {
			origin.isAltitudeValid = true;
		}
	}

	if (origin.isAltitudeValid == origin.isLatitudeValid
			&& origin.isLatitudeValid == origin.isLongitudeValid) {
		if (!origin.isAltitudeValid) {
			GeoPosition orig;
			if (UtilReadOriginConfiguration(&orig) != -1) {
				this->origin.latitude_deg = orig.Latitude;
				this->origin.longitude_deg = orig.Longitude;
				this->origin.altitude_m = orig.Altitude;
				this->origin.isLatitudeValid = true;
				this->origin.isLongitudeValid = true;
				this->origin.isAltitudeValid = true;
			}
			else {
				throw std::invalid_argument("No origin setting found");
			}
		}
	}
	else {
		throw std::invalid_argument("Partial origin setting in file " + objectFile.string());
	}
}

void TestObject::parseTrajectoryFile(
		const fs::path& file) {
	trajectory.initializeFromFile(file.filename());
	LogMessage(LOG_LEVEL_INFO, "Successfully parsed trajectory %s for object %u",
			   file.filename().c_str(), this->getTransmitterID());
}

void TestObject::establishConnection(std::shared_future<void> stopRequest) {
	this->lastMonitorTime = std::chrono::steady_clock::time_point(); // reset
	this->comms.connect(stopRequest, TestObject::connRetryPeriod, this->isOsiCompatible());
}

void ObjectConnection::connect(
		std::shared_future<void> stopRequest,
		const std::chrono::milliseconds retryPeriod,
		const bool isOsiObject) {
	char ipString[INET_ADDRSTRLEN];
	std::stringstream errMsg;

	if (inet_ntop(AF_INET, &this->cmd.addr.sin_addr, ipString, sizeof (ipString))
			== nullptr) {
		errMsg << "inet_ntop: " << strerror(errno);
		throw std::invalid_argument(errMsg.str());
	}

	if (this->cmd.addr.sin_addr.s_addr == 0) {
		errMsg << "Invalid connection IP specified: " << ipString;
		throw std::invalid_argument(errMsg.str());
	}

	this->cmd.socket = socket(AF_INET, SOCK_STREAM, 0);
	if (this->cmd.socket < 0) {
		errMsg << "Failed to open ISO control socket: " << strerror(errno);
		this->disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Begin connection attempt
	LogMessage(LOG_LEVEL_INFO, "Attempting connection to ISO object: %s:%u", ipString,
			   ntohs(this->cmd.addr.sin_port));
	while (true) {
		if (::connect(this->cmd.socket,
					reinterpret_cast<struct sockaddr *>(&this->cmd.addr),
					sizeof (this->cmd.addr)) == 0) {
			break;
		}
		else {
			LogMessage(LOG_LEVEL_ERROR, "Failed connection attempt to ISO object %s, retrying in %.3f s ...",
					   ipString, retryPeriod.count() / 1000.0);
			if (stopRequest.wait_for(retryPeriod)
					!= std::future_status::timeout) {
				errMsg << "Connection attempt interrupted";
				throw std::runtime_error(errMsg.str());
			}
		}
	}

	if(isOsiObject == true){
		if (inet_ntop(AF_INET, &this->osi.addr.sin_addr, ipString, sizeof (ipString))
				== nullptr) {
			errMsg << "inet_ntop: " << strerror(errno);
			throw std::invalid_argument(errMsg.str());
		}

		if (this->osi.addr.sin_addr.s_addr == 0) {
			errMsg << "Invalid connection IP specified: " << ipString;
			throw std::invalid_argument(errMsg.str());
		}

		this->osi.socket = socket(AF_INET, SOCK_STREAM, 0);
		if (this->osi.socket < 0) {
			errMsg << "Failed to open OSI control socket: " << strerror(errno);
			this->disconnect();
			throw std::runtime_error(errMsg.str());
		}

		// Begin connection attempt
		LogMessage(LOG_LEVEL_INFO, "Attempting connection to OSI object: %s:%u", ipString,
				   ntohs(this->osi.addr.sin_port));
		while (true) {
			if (::connect(this->osi.socket,
						reinterpret_cast<struct sockaddr *>(&this->osi.addr),
						sizeof (this->osi.addr)) == 0) {
				break;
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Failed connection attempt to OSI object%s, retrying in %.3f s ...",
						   ipString, retryPeriod.count() / 1000.0);
				if (stopRequest.wait_for(retryPeriod)
						!= std::future_status::timeout) {
					errMsg << "Connection attempt interrupted";
					throw std::runtime_error(errMsg.str());
				}
			}
		}
	}

	// Create monitor socket
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");
	this->mntr.socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (!this->isValid()) {
		errMsg << "Failed to create monitor socket: " << strerror(errno);
		this->disconnect();
		throw std::runtime_error(errMsg.str());
	}

	sockaddr* sin = reinterpret_cast<sockaddr*>(&this->mntr.addr);
	if (::connect(this->mntr.socket, sin, sizeof (this->mntr.addr)) < 0) {
		errMsg << "Failed to connect monitor socket: " << strerror(errno);
		this->disconnect();
		throw std::runtime_error(errMsg.str());
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
	return this->cmd.socket != -1 && this->mntr.socket != -1;
}

void ObjectConnection::disconnect() {
	if (this->cmd.socket != -1) {
		if (shutdown(this->cmd.socket, SHUT_RDWR) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Command socket shutdown: %s", strerror(errno));
		}
		if (close(this->cmd.socket) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Command socket close: %s", strerror(errno));
		}
		this->cmd.socket = -1;
	}
	if (this->mntr.socket != -1) {
		if (shutdown(this->mntr.socket, SHUT_RDWR) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Safety socket shutdown: %s", strerror(errno));
		}
		if (close(this->mntr.socket) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Safety socket close: %s", strerror(errno));
		};
		this->mntr.socket = -1;
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
	objSettings.desiredTransmitterID = transmitterID;
	objSettings.isTransmitterIDValid = true;

	objSettings.coordinateSystemOrigin = origin;

	TimeSetToCurrentSystemTime(&objSettings.currentTime);
	objSettings.isTimestampValid = true;

	objSettings.isLateralDeviationLimited = false;
	objSettings.isPositionDeviationLimited = false;
	objSettings.isPositioningAccuracyRequired = false;

	this->comms.cmd << objSettings;
	this->comms.cmd << trajectory;
}

void TestObject::sendArm() {
	this->comms.cmd << OBJECT_COMMAND_ARM;
}

void TestObject::sendDisarm() {
	this->comms.cmd << OBJECT_COMMAND_DISARM;
}

//void TestObject::sendOsiGlobalObjectGroundTruth(
//	GlobalObjectGroundTruth_t gogt) {
	//this->comms.osi << llls;
//}

void TestObject::sendOsiData(std::vector<char> osidata) {
	this->comms.cmd << osidata;
}


void TestObject::sendStart() {
	StartMessageType strt;
	TimeSetToCurrentSystemTime(&strt.startTime); // TODO make possible to modify
	strt.isTimestampValid = true;
	this->comms.cmd << strt;
}

ISOMessageID Channel::pendingMessageType(bool awaitNext) {
	auto result = recv(this->socket, this->receiveBuffer.data(), this->receiveBuffer.size(), (awaitNext ? 0 : MSG_DONTWAIT) | MSG_PEEK);
	if (result < 0 && !awaitNext && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		return MESSAGE_ID_INVALID;
	}
	else if (result < 0) {
		throw std::ios_base::failure(std::string("Failed to check pending message type (recv: ")
									 + strerror(errno) + ")");
	}
	else if (result == 0) {
		throw std::ios_base::failure("Connection reset by peer");
	}
	else {
		ISOMessageID retval = getISOMessageType(this->receiveBuffer.data(), this->receiveBuffer.size(), false);
		if (retval == MESSAGE_ID_INVALID) {
			throw std::invalid_argument("Non-ISO message received from " + this->remoteIP());
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
			throw std::ios_base::failure(std::string("Failed socket operation (select: ") + strerror(errno) + ")"); // TODO clearer
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
		throw std::invalid_argument(std::string("Failed to send HEAB: ") + strerror(errno));
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
		throw std::invalid_argument(std::string("Failed to send OSEM: ") + strerror(errno));
	}
	return chnl;
}

Channel& operator<<(Channel& chnl, const Trajectory& traj) {
	ssize_t nBytes;

	// TRAJ header
	nBytes = encodeTRAJMessageHeader(
				traj.id, traj.version, traj.name.c_str(),traj.name.length(),
				static_cast<uint32_t>(traj.points.size()), chnl.transmitBuffer.data(),
				chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to send TRAJ message header: ") + strerror(errno));
	}

	// TRAJ points
	for (const auto& pt : traj.points) {
		struct timeval relTime;
		CartesianPosition pos = pt.getISOPosition();
		SpeedType spd = pt.getISOVelocity();
		AccelerationType acc = pt.getISOAcceleration();

		relTime.tv_sec = static_cast<time_t>(pt.getTime());
		relTime.tv_usec = static_cast<time_t>((pt.getTime() - relTime.tv_sec) * 1000000);

		nBytes = encodeTRAJMessagePoint(&relTime, pos, spd, acc, static_cast<float>(pt.getCurvature()),
										chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
		if (nBytes < 0) {
			// TODO what to do here?
			throw std::invalid_argument(std::string("Failed to encode TRAJ message point: ") + strerror(errno));
		}
		nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);

		if (nBytes < 0) {
			// TODO what to do here?
			throw std::invalid_argument(std::string("Failed to send TRAJ message point: ") + strerror(errno));
		}
	}

	// TRAJ footer
	nBytes = encodeTRAJMessageFooter(chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message footer: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to send TRAJ message footer: ") + strerror(errno));
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
		throw std::invalid_argument(std::string("Failed to send OSTM: ") + strerror(errno));
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
		throw std::invalid_argument(std::string("Failed to send STRT: ") + strerror(errno));
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
				throw std::ios_base::failure("Unable to clear from socket buffer");
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
				throw std::ios_base::failure("Unable to clear from socket buffer");
			}
		}
	}
	return chnl;
}


Channel& operator<<(Channel& chnl, std::vector<char>& osi) {
	
	auto nBytes = send(chnl.socket, osi.data(), osi.size(), 0);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to send OSI data: ") + strerror(errno));
	}
	return chnl;
}



std::string Channel::remoteIP() const {
	char ipString[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->addr.sin_addr, ipString, sizeof (ipString));
	return std::string(ipString);
}
