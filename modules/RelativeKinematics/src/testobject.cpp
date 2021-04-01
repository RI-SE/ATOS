#include "testobject.hpp"
#include "util.h"
#include "maestroTime.h"

TestObject::TestObject() {
	// TODO
}

void TestObject::setCommandAddress(
		const sockaddr_in &newAddr) {
	if (!this->comms.connected()) {
		this->comms.cmd.addr = newAddr;
	}
}

void TestObject::setMonitorAddress(
		const sockaddr_in &newAddr) {
	if (!this->comms.connected()) {
		this->comms.mntr.addr = newAddr;
	}
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
					   ipString, TestObject::connRetryPeriod_ms / 1000.0);
			if (stopRequest.wait_for(std::chrono::milliseconds(TestObject::connRetryPeriod_ms))
					!= std::future_status::timeout) {
				errMsg << "Connection attempt interrupted";
				throw std::runtime_error(errMsg.str());
			}
		}
	}

	// Set command socket to nonblocking
	if (fcntl(this->comms.cmd.socket, F_SETFL,
			  fcntl(this->comms.cmd.socket, F_GETFL, 0) | O_NONBLOCK) == -1) {
		errMsg << "fcntl: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Create monitor socket
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");
	this->comms.mntr.socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (!this->comms.valid()) {
		errMsg << "Failed to create monitor socket: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Set monitor socket to nonblocking
	if (fcntl(this->comms.mntr.socket, F_SETFL,
			  fcntl(this->comms.mntr.socket, F_GETFL, 0) | O_NONBLOCK) == -1) {
		errMsg << "fcntl: " << strerror(errno);
		this->comms.disconnect();
		throw std::runtime_error(errMsg.str());
	}
	return;
}


bool ObjectConnection::connected() const {
	// TODO
	return false;
}

bool ObjectConnection::valid() const {
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

Channel& operator<<(Channel& chnl, const ObjectSettingsType& settings) {
	auto nBytes = encodeOSEMMessage(&settings, chnl.transmitBuffer.data(), sizeof (chnl.transmitBuffer.size()), false);
	if (nBytes < 0) {
		throw std::invalid_argument("Failed to encode OSEM message");
	}
	nBytes = write(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes));
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
	nBytes = write(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes));
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
		nBytes = write(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes));
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
	nBytes = write(chnl.socket, chnl.transmitBuffer.data(), static_cast<size_t>(nBytes));
	if (nBytes < 0) {
		throw std::invalid_argument(std::string("Failed to send TRAJ message: ") + strerror(errno));
	}
	return chnl;
}
