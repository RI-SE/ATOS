#include <map>
#include <algorithm>
#include "datadictionary.h"
#include "util.h"
#include "objectconfig.hpp"
#include "trajectory.hpp"

#define MODULE_NAME "BackToStart"

#define MAX_BTS_DISTANCE_TOLERANCE 15
#define MAX_BTS_HEADING_TOLERANCE 30

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
                backToStart();
            } catch (std::invalid_argument& e) {
                LogMessage(LOG_LEVEL_ERROR, "Loading of object files failed - %s", e.what());
                iCommSend(COMM_FAILURE, nullptr, 0);
            }
            break;

        case COMM_REMOTECTRL_MANOEUVRE:
            LogMessage(LOG_LEVEL_INFO,"Received COMM_REMOTECTRL_MANOEUVRE command");
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
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

    LogMessage(LOG_LEVEL_INFO,"FIRST POINT IN TRAJ: %f:%f:%f:%f",firstPointInTraj.xCoord_m, firstPointInTraj.yCoord_m, firstPointInTraj.zCoord_m,firstPointInTraj.heading_rad);

    LogMessage(LOG_LEVEL_INFO, "HEADING: %f, POSITION_X: %f", firstPointInTraj.heading_rad, firstPointInTraj.xCoord_m);

    return UtilIsPositionNearTarget(monitorData.position, firstPointInTraj, MAX_BTS_DISTANCE_TOLERANCE)
            && UtilIsAngleNearTarget(monitorData.position, firstPointInTraj, MAX_BTS_HEADING_TOLERANCE);
}



void backToStart() {

    //Get transmitter IDs
    uint32_t *noOfObjects;
    DataDictionaryGetNumberOfObjects(noOfObjects);
    uint32_t transmitterIDs[*noOfObjects];
    DataDictionaryGetObjectTransmitterIDs(transmitterIDs, *noOfObjects);

    //Array to save b2s trajs
    Trajectory b2sTrajectories[*noOfObjects];

    for(int i = 0; i < objects.size(); i++)
    {
        LogMessage(LOG_LEVEL_INFO, "TRAJECTORY: %d", i);
        std::string trajName = "BTS" + std::to_string(i);
        Trajectory currentTraj;
        Trajectory b2sTraj;

        //Name
        b2sTraj.name = trajName;

        //Testpath
        LogMessage(LOG_LEVEL_INFO, "TXID: %d", transmitterIDs[i]);
        currentTraj = objects.at(transmitterIDs[i]).getTrajectory();

        //TODO
        //Get transmitter ID:s
        //Check distance and rotation
        //Send pass or fail
        //generate BTS
        //send BTS trajectories
        //???
        //profit

        //Add first turn
        b2sTraj.addWilliamsonTurn(5,currentTraj.points[currentTraj.points.size()-1], 0);

        //Add reversed original traj
        Trajectory rev = currentTraj;
        rev = rev.reversed(b2sTraj.points[b2sTraj.points.size()-1].getTime());
        b2sTraj.points.insert(std::end(b2sTraj.points), std::begin(rev.points), std::end(rev.points));

        //Add last turn
        b2sTraj.addWilliamsonTurn(5,b2sTraj.points[b2sTraj.points.size()-1], b2sTraj.points[b2sTraj.points.size()-1].getTime());

        //Check distance
        //if(!isObjectNearTrajectoryStart(transmitterIDs[i], b2sTraj))
        //{
            LogMessage(LOG_LEVEL_INFO, "BTS FAILED, SENDING 0 TO GUC");
            const char *btsChar = "BTS-FAIL";
            iCommSend(COMM_BACKTOSTART, btsChar, sizeof (btsChar));
            //return;
        //}
    }

    //If pass save files
    for(int i = 0; i < *noOfObjects; i++)
    {
        b2sTrajectories[i].saveToFile(b2sTrajectories[i].name);
    }

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
