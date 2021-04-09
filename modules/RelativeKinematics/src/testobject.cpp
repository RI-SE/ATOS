#include "testobject.hpp"
#include "util.h"
#include "maestroTime.h"
#include "datadictionary.h"

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
	isVUT(other.isVUT),
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

ObjectDataType TestObject::awaitNextMonitor() {
	ssize_t result = 0;
	ObjectDataType retval;
	struct timeval tv;
	// Read until receive buffer is empty, return last read message
	result = recv(this->comms.mntr.socket, this->comms.mntr.receiveBuffer.data(),
				  this->comms.mntr.receiveBuffer.size(), 0);
	if (result < 0) {
		this->comms.disconnect();
		throw std::ios_base::failure(std::string("Failed to receive from monitor socket: ") + strerror(errno));
	}
	else if (result == 0) {
		this->comms.disconnect();
		throw std::ios_base::failure("Remote closed monitor socket");
	}

	if (getISOMessageType(this->comms.mntr.receiveBuffer.data(), static_cast<size_t>(result), false) == MESSAGE_ID_MONR) {
		retval.ClientIP = this->comms.cmd.addr.sin_addr.s_addr;
		TimeSetToCurrentSystemTime(&tv);
		if (decodeMONRMessage(this->comms.mntr.receiveBuffer.data(), static_cast<size_t>(result),
							  tv, &retval.ClientID, &retval.MonrData, false) < 0) {
			this->comms.disconnect();
			throw std::invalid_argument("Error decoding MONR from object"); // TODO add details which exact object sent the data
		}
		retval.lastPositionUpdate = tv;
		// TODO check age
	}
	else {
		throw std::invalid_argument("Received non-MONR message on MONR channel");
	}

	return retval;
}

void TestObject::updateMonitor(const ObjectDataType &data) {
	if (data.ClientID != this->getTransmitterID()) {
		throw std::invalid_argument("Attempted to set monitor data with non-matching transmitter ID");
	}
	this->state = data.MonrData.state;
	this->lastMonitor = data;
}

ObjectStateType TestObject::getState(bool awaitUpdate) {
	if (awaitUpdate) {
		if (!nextMonitor.valid()) {
			nextMonitor = std::async(std::launch::async, &TestObject::awaitNextMonitor, this);
		}
		auto status = nextMonitor.wait_for(maxAllowedMonitorPeriod);
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
			+ ", object file " + objectFile.filename().string() + ", VUT:" + (isVUT?"Yes":"No");
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

	// Get VUT setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_IS_VUT, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		this->isVUT = false;
	}
	else {
		if (setting[0] == '1' || setting[0] == '0') {
			this->isVUT = setting[0] == '1';
		}
		else {
			std::string vutSetting(setting);
			for (char &c : vutSetting) {
				c = std::tolower(c, std::locale());
			}
			if (vutSetting.compare("true") == 0) {
				this->isVUT = true;
			}
			else if (vutSetting.compare("false") == 0) {
				this->isVUT = false;
			}
			else {
				throw std::invalid_argument("VUT setting " + std::string(setting) + " in file "
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

void TestObject::establishConnection(
		std::shared_future<void> stopRequest) {
	char ipString[INET_ADDRSTRLEN];
	std::stringstream errMsg;

	if (inet_ntop(AF_INET, &this->comms.cmd.addr.sin_addr, ipString, sizeof (ipString))
			== nullptr) {
		errMsg << "inet_ntop: " << strerror(errno);
		throw std::invalid_argument(errMsg.str());
	}

	if (this->comms.cmd.addr.sin_addr.s_addr == 0) {
		errMsg << "Invalid connection IP specified: " << ipString;
		throw std::invalid_argument(errMsg.str());
	}

	this->comms.cmd.socket = socket(AF_INET, SOCK_STREAM, 0);
	if (this->comms.cmd.socket < 0) {
		errMsg << "Failed to open control socket: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Begin connection attempt
	LogMessage(LOG_LEVEL_INFO, "Attempting connection to: %s:%u", ipString,
			   ntohs(this->comms.cmd.addr.sin_port));
	while (true) {
		if (connect(this->comms.cmd.socket,
					reinterpret_cast<struct sockaddr *>(&this->comms.cmd.addr),
					sizeof (this->comms.cmd.addr)) == 0) {
			break;
		}
		else {
			LogMessage(LOG_LEVEL_ERROR, "Failed connection attempt to %s, retrying in %.3f s ...",
					   ipString, TestObject::connRetryPeriod.count() / 1000.0);
			if (stopRequest.wait_for(TestObject::connRetryPeriod)
					!= std::future_status::timeout) {
				errMsg << "Connection attempt interrupted";
				throw std::runtime_error(errMsg.str());
			}
		}
	}

	// Create monitor socket
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");
	this->comms.mntr.socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (!this->comms.isValid()) {
		errMsg << "Failed to create monitor socket: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	sockaddr* sin = reinterpret_cast<sockaddr*>(&this->comms.mntr.addr);
	if (connect(this->comms.mntr.socket, sin, sizeof (this->comms.mntr.addr)) < 0) {
		errMsg << "Failed to connect monitor socket: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	return;
}


bool ObjectConnection::isConnected() const {
	// TODO
	return false;
}

bool ObjectConnection::isValid() const {
	return this->cmd.socket != -1 && this->mntr.socket != -1;
}

void ObjectConnection::disconnect() {
	if (this->cmd.socket != -1) {
		close(this->cmd.socket);
		this->cmd.socket = -1;
	}
	if (this->mntr.socket != -1) {
		close(this->mntr.socket);
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
		throw std::invalid_argument(std::string("Failed to send TRAJ message: ") + strerror(errno));
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
			throw std::invalid_argument(std::string("Failed to send TRAJ message: ") + strerror(errno));
		}
	}

	// TRAJ footer
	nBytes = encodeTRAJMessageFooter(chnl.transmitBuffer.data(), chnl.transmitBuffer.size(), false);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to encode TRAJ message footer: ") + strerror(errno));
	}
	nBytes = send(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes), 0);
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to send TRAJ message: ") + strerror(errno));
	}
	return chnl;
}
