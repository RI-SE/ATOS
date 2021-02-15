#include "testobject.hpp"
#include "util.h"

TestObject::TestObject() {
	// TODO
}

void TestObject::setCommandAddress(
		const sockaddr_in &newAddr) {
	this->commandAddress = newAddr;
}

void TestObject::setMonitorAddress(
		const sockaddr_in &newAddr) {
	this->monitorAddress = newAddr;
}

std::string TestObject::toString() const {
	char ipAddr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &this->commandAddress, ipAddr, sizeof (ipAddr));
	std::string retval = "";
	retval += "Object ID " + std::to_string(transmitterID)
			+ ", IP " + ipAddr + ", trajectory file " + trajectoryFile.stem().string()
			+ ", object file " + objectFile.stem().string();
	return retval;
}

void TestObject::parseConfigurationFile(
		const fs::path &objectFile) {

	char setting[100];
	int result;
	struct sockaddr_in addr;

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

	fs::path trajFile(setting);
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
}


void TestObject::parseTrajectoryFile(
		const fs::path& file) {
	// TODO
}
