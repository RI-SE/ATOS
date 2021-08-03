#include <map>
#include <algorithm>

#include "util.h"
#include "objectconfig.hpp"

#define MODULE_NAME "BackToStart"

std::map<uint32_t,ObjectConfig> objects; //!< List of configured test objects

static void loadObjectFiles();

int main()
{
	COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0,10000000};
	struct timespec remTime;

	LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	// Initialize message bus connection
	while (iCommInit()) {
		nanosleep(&sleepTimePeriod,&remTime);
	}

	while (true) {
		if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0) {
			util_error("Message bus receive error");
		}

		switch (command) {
		case COMM_INV:
			nanosleep(&sleepTimePeriod,&remTime);
			break;
		case COMM_OBC_STATE:
			break;
		case COMM_CONNECT:
			try {
				loadObjectFiles();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Loading of object files failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
		case COMM_REMOTECTRL_MANOEUVRE:
			LogMessage(LOG_LEVEL_INFO,"Received COMM_REMOTECTRL_MANOEUVRE command");
		default:
			LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
		}
	}
	return 0;
}

void loadObjectFiles() {
	objects.clear();
	char path[MAX_FILE_PATH];
	std::vector<std::invalid_argument> errors;

	UtilGetObjectDirectoryPath(path, sizeof (path));
	fs::path objectDir(path);
	if (!fs::exists(objectDir)) {
		throw std::ios_base::failure("Object directory does not exist");
	}

	for (const auto& entry : fs::directory_iterator(objectDir)) {
		if (fs::is_regular_file(entry.status())) {
			const auto inputFile = entry.path();
			ObjectConfig object;
			try {
				object.parseConfigurationFile(entry.path());
				LogMessage(LOG_LEVEL_INFO, "Loaded configuration: %s", object.toString().c_str());

				// Check preexisting
				auto foundObject = objects.find(object.getTransmitterID());
				if (foundObject == objects.end()) {
					objects.emplace(object.getTransmitterID(), std::move(object));
				}
				else {
					auto badID = object.getTransmitterID();
					std::string errMsg = "Duplicate object ID " + std::to_string(badID)
						+ " detected";
					throw std::invalid_argument(errMsg);
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, e.what());
				errors.push_back(e);
			}
		}
	}
	if (!errors.empty()) {
		objects.clear();
		std::ostringstream ostr;
		auto append = [&ostr](const std::invalid_argument& e){
			ostr << e.what() << std::endl;
		};
		std::for_each(errors.begin(), errors.end(), append);
		throw std::invalid_argument("Failed to parse object file(s):\n" + ostr.str());
	}
}

