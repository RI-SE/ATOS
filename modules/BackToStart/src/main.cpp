#include <map>
#include <algorithm>
#include "datadictionary.h"
#include "util.h"
#include "objectconfig.hpp"
#include "trajectory.hpp"

#define MODULE_NAME "BackToStart"

#define MAX_BTS_DISTANCE_TOLERANCE_M 15.0
#define MAX_BTS_HEADING_TOLERANCE_DEG 30.0

std::map<uint32_t,ObjectConfig> objects; //!< List of configured test objects

static void loadObjectFiles();
static void backToStart();

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

	DataDictionaryInitObjectData();

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
		case COMM_INIT:
			try {
				loadObjectFiles();
			} catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Loading of object files failed - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;

		case COMM_BACKTOSTART_CALL:
			LogMessage(LOG_LEVEL_INFO,"Received COMM_BACKTOSTART_CALL command");
			backToStart();
			break;
		default:
			break;
		}
	}
	return 0;
}


/*!
 * \brief checkIfBackToStartAllowed verifies if the position and heading of an
 *			object is in line with the first point of a trajectory
 * \param transmitterID of objectToCheck
 * \param generatedTrajectory trajectory to check against
 * \return True if allowed False if not
 */
bool isObjectNearTrajectoryStart(
		const uint32_t transmitterID,
		const Trajectory& trajectory) {

	ObjectMonitorType monitorData;
	DataDictionaryGetMonitorData(transmitterID, &monitorData);

	auto firstPointInTraj = trajectory.points.front().getISOPosition();

	LogMessage(LOG_LEVEL_DEBUG, "First point in trajectory: %s", trajectory.points.front().toString().c_str());
	return UtilIsPositionNearTarget(monitorData.position, firstPointInTraj, MAX_BTS_DISTANCE_TOLERANCE_M)
			&& UtilIsAngleNearTarget(monitorData.position, firstPointInTraj, MAX_BTS_HEADING_TOLERANCE_DEG);
}



void backToStart() {

	//Get transmitter IDs
	uint32_t noOfObjects;
	DataDictionaryGetNumberOfObjects(&noOfObjects);
	std::vector<uint32_t> transmitterIDs(noOfObjects);
	DataDictionaryGetObjectTransmitterIDs(transmitterIDs.data(), transmitterIDs.size());

	char btsResponseBuffer[sizeof (BTSResponse)];

	//Array to save b2s trajs
	std::vector<Trajectory> b2sTrajectories;

	for (const auto txID : transmitterIDs) {
		LogMessage(LOG_LEVEL_DEBUG, "Handling back-to-start for trajectory %u", txID);
		std::string trajName = "BTS" + std::to_string(txID);
		Trajectory b2sTraj;

		//Name
		b2sTraj.name = trajName;

		//Testpath
		auto currentTraj = objects.at(txID).getTrajectory();

		//Add first turn
		Trajectory turn1 = Trajectory::createWilliamsonTurn(5, 1, currentTraj.points.back());
		b2sTraj.points.insert(std::end(b2sTraj.points), std::begin(turn1.points), std::end(turn1.points));

		//Add reversed original traj
		auto rev = currentTraj.reversed();
		rev = rev.delayed(b2sTraj.points.back().getTime());
		b2sTraj.points.insert(std::end(b2sTraj.points), std::begin(rev.points), std::end(rev.points));

		//Add last turn
		Trajectory turn2 = Trajectory::createWilliamsonTurn(5, 1, b2sTraj.points[b2sTraj.points.size()-1]);
		turn2 = turn2.delayed(b2sTraj.points.back().getTime());
		b2sTraj.points.insert(std::end(b2sTraj.points), std::begin(turn2.points), std::end(turn2.points));

		//Check distance
		if(!isObjectNearTrajectoryStart(txID, b2sTraj))
		{
			LogMessage(LOG_LEVEL_INFO, "Object %u not near starting point: sending back-to-start failure", txID);

			memset(btsResponseBuffer, 0, sizeof (btsResponseBuffer));
			BTSResponse btsResponse = BTS_FAIL;
			memcpy(btsResponseBuffer, &btsResponse, sizeof (btsResponse));
			iCommSend(COMM_BACKTOSTART_RESPONSE, btsResponseBuffer, sizeof (btsResponse));
			return;
		}
		b2sTrajectories.push_back(b2sTraj);

	}

	if (UtilDeleteTrajectoryFiles() == FAILED_DELETE) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to remove trajectory files");
		return;
	}

	//If pass save files
	for (const auto& traj : b2sTrajectories) {
		LogMessage(LOG_LEVEL_DEBUG, "Generated back-to-start trajectory %s", traj.name.c_str());
		traj.saveToFile(traj.name + ".traj");
	}

	memset(btsResponseBuffer, 0, sizeof (btsResponseBuffer));
	BTSResponse btsResponse = BTS_PASS;
	memcpy(btsResponseBuffer, &btsResponse, sizeof (btsResponse));
	iCommSend(COMM_BACKTOSTART_RESPONSE, btsResponseBuffer, sizeof (btsResponse));

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
