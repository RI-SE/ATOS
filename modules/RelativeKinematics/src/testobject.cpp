#include "testobject.hpp"
#include "util.h"

TestObject::TestObject() {
	// TODO
}

void TestObject::setCommandAddress(
		const sockaddr_in &newAddr) {
	if (!this->channel.connected()) {
		this->channel.cmdAddr = newAddr;
	}
}

void TestObject::setMonitorAddress(
		const sockaddr_in &newAddr) {
	if (!this->channel.connected()) {
		this->channel.mntrAddr = newAddr;
	}
}

std::string TestObject::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->channel.cmdAddr.sin_addr, ipAddr, sizeof (ipAddr));
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
	this->setCommandAddress(addr);
	this->setMonitorAddress(addr);

	// Get trajectory file setting
	if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFile.c_str(),
								 objectFile.string().length(),
								 setting, sizeof (setting)) == -1) {
		throw std::invalid_argument("Cannot find trajectory setting in file " + objectFile.string());
	}

	fs::path trajFile(std::string(path) + std::string(setting));
	std::cout << "HÄR: " << trajFile;
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

	if (inet_ntop(AF_INET, &this->channel.cmdAddr.sin_addr, ipString, sizeof (ipString))
			== nullptr) {
		errMsg << "inet_ntop: " << strerror(errno);
		throw std::invalid_argument(errMsg.str());
	}

	if (this->channel.cmdAddr.sin_addr.s_addr == 0) {
		errMsg << "Invalid connection IP specified: " << ipString;
		throw std::invalid_argument(errMsg.str());
	}

	this->channel.commandSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (this->channel.commandSocket < 0) {
		errMsg << "Failed to open control socket: " << strerror(errno);
		this->channel.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Begin connection attempt
	LogMessage(LOG_LEVEL_INFO, "Attempting to connect to socket: %s:%u", ipString,
			   this->channel.cmdAddr.sin_port);
	while (true) {
		if (connect(this->channel.commandSocket,
					reinterpret_cast<struct sockaddr *>(&this->channel.cmdAddr),
					sizeof (this->channel.cmdAddr)) == 0) {
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
	if (fcntl(this->channel.commandSocket, F_SETFL,
			  fcntl(this->channel.commandSocket, F_GETFL, 0) | O_NONBLOCK) == -1) {
		errMsg << "fcntl: " << strerror(errno);
		this->channel.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Create monitor socket
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");
	this->channel.monitorSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (!this->channel.valid()) {
		errMsg << "Failed to create monitor socket: " << strerror(errno);
		this->channel.disconnect();
		throw std::runtime_error(errMsg.str());
	}

	// Set monitor socket to nonblocking
	if (fcntl(this->channel.monitorSocket, F_SETFL,
			  fcntl(this->channel.monitorSocket, F_GETFL, 0) | O_NONBLOCK) == -1) {
		errMsg << "fcntl: " << strerror(errno);
		this->channel.disconnect();
		throw std::runtime_error(errMsg.str());
	}
	return;
}


bool TestObject::ObjectConnection::connected() const {
	// TODO
}

bool TestObject::ObjectConnection::valid() const {
	return this->commandSocket != -1 && this->monitorSocket != -1;
}

void TestObject::ObjectConnection::disconnect() {
	if (this->commandSocket != -1) {
		close(this->commandSocket);
		this->commandSocket = -1;
	}
	if (this->monitorSocket != -1) {
		close(this->monitorSocket);
		this->monitorSocket = -1;
	}
}

