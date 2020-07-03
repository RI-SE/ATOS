#include <iostream>
#include <vector>
#include <array>
#include <signal.h>
#include <dirent.h>
#include <fstream>
#include <regex>
#include <systemd/sd-daemon.h>

#include "supervisionstate.h"
#include "geofence.h"
#include "trajectory.h"
#include "logging.h"
#include "util.h"

#define MODULE_NAME "Supervision"
#define MAX_GEOFENCE_NAME_LEN 256

#define ARM_MAX_DISTANCE_TO_START_M 1.0
#define ARM_MAX_ANGLE_TO_START_DEG 10.0

/*------------------------------------------------------------
  -- Type definitions.
  ------------------------------------------------------------*/
typedef enum {
    ALL_OBJECTS_NEAR_START,         //!< The queried object is near its starting position and all objects have been checked
    SINGLE_OBJECT_NOT_NEAR_START,   //!< The queried object is not near its starting position
    SINGLE_OBJECT_NEAR_START,       //!< The queried object is near its starting position but all objects have not yet been checked
    OBJECT_HAS_NO_TRAJECTORY        //!< The queried object has no trajectory
} PositionStatus;

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static bool isViolatingGeofence(const ObjectDataType &monitorData, std::vector<Geofence> geofences);
static void loadGeofenceFiles(std::vector<Geofence> &geofences);
static void loadTrajectoryFiles(std::vector<Trajectory> &trajectories);
static Geofence parseGeofenceFile(const std::string geofenceFile);
static PositionStatus updateNearStartingPositionStatus(const ObjectDataType &MONRData, std::vector<std::pair<Trajectory&, bool>> armVerified);

static void signalHandler(int signo);

/*------------------------------------------------------------
  -- Static variables
  ------------------------------------------------------------*/
static bool quit = false;

/*------------------------------------------------------------
  -- Main task
  ------------------------------------------------------------*/
int main()
{
    COMMAND command = COMM_INV;
    char mqRecvData[MQ_MSG_SIZE], mqSendData[MQ_MSG_SIZE];
    char ipString[INET_ADDRSTRLEN];
    std::vector<Geofence> geofences;
    std::vector<Trajectory> trajectories;
    std::vector<std::pair<Trajectory&, bool>> armVerified;
    const struct timespec sleepTimePeriod = {0,10000000};
    struct timespec remTime;
    SupervisionState state;

    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u",getpid());

    // Set up signal handlers
    if (signal(SIGINT, signalHandler) == SIG_ERR)
        util_error("Unable to initialize signal handler");

    // Initialize message bus connection
    while(iCommInit() && !quit) {
        nanosleep(&sleepTimePeriod,&remTime);
    }

	// Notify service handler that startup was successful
	sd_notify(0, "READY=1");

    while(!quit) {
        if (state.get() == SupervisionState::ERROR) {
            iCommSend(COMM_ABORT, nullptr, 0);
        }

        if (iCommRecv(&command, mqRecvData, sizeof (mqRecvData), nullptr) < 0) {
            util_error("Message bus receive error");
        }

        switch (command) {
        case COMM_INIT:
            state.set(SupervisionState::VERIFYING_INIT);
            if (!geofences.empty())
                geofences.clear();
            if (!trajectories.empty()) {
                trajectories.clear();
                armVerified.clear();
            }
            try {
                loadGeofenceFiles(geofences);
                loadTrajectoryFiles(trajectories);
                for (Trajectory &trajectory : trajectories) {
                    armVerified.push_back({trajectory, false});
                }
            }
            catch (std::invalid_argument e) {
                LogMessage(LOG_LEVEL_ERROR, "Unable to initialize due to file parsing error");
                iCommSend(COMM_DISCONNECT, nullptr, 0);
            }
            state.set(SupervisionState::READY);
            break;
        case COMM_OSEM:
            // TODO: check so that OSEM is not null island?

			break;
        case COMM_MONR:
			ObjectDataType monitorMessage;
			UtilPopulateMonitorDataStruct(mqRecvData, sizeof (mqRecvData), &monitorMessage);

			if (state.get() == SupervisionState::RUNNING && isViolatingGeofence(monitorMessage, geofences)) {
                LogMessage(LOG_LEVEL_WARNING, "Object with IP %s is violating a geofence: sending ABORT",
						   inet_ntop(AF_INET, &monitorMessage.ClientIP, ipString, sizeof (ipString)));
                iCommSend(COMM_ABORT, nullptr, 0);
                state.set(SupervisionState::READY);
            }

            if (state.get() == SupervisionState::VERIFYING_ARM) {
				if (isViolatingGeofence(monitorMessage, geofences)) {
                    LogMessage(LOG_LEVEL_INFO, "Arm not approved: object with IP address %s is violating a geofence",
							   inet_ntop(AF_INET, &monitorMessage.ClientIP, ipString, sizeof (ipString)));
                    iCommSend(COMM_DISARM, nullptr, 0);
                    state.set(SupervisionState::READY);
                    break;
                }

				switch (updateNearStartingPositionStatus(monitorMessage, armVerified)) {
                case SINGLE_OBJECT_NOT_NEAR_START: // Object not near start: disarm
                    LogMessage(LOG_LEVEL_INFO, "Arm not approved: sending disarm");
                    iCommSend(COMM_DISARM, nullptr, 0);
                    state.set(SupervisionState::READY);
                    break;
                case ALL_OBJECTS_NEAR_START:
                    LogMessage(LOG_LEVEL_INFO, "Arm approved");
                    state.set(SupervisionState::READY);
                    break;
                case SINGLE_OBJECT_NEAR_START: // Need to wait for all objects to report position
                    break;
                case OBJECT_HAS_NO_TRAJECTORY: // Object has no trajectory, no need to check it
                    break;
                }
            }
            break;
        case COMM_ARM:
            try {
                std::for_each(armVerified.begin(), armVerified.end(),
                              [](std::pair<Trajectory&, bool> &pair) { pair.second = false; });
                state.set(SupervisionState::VERIFYING_ARM);
            }
            catch (std::invalid_argument e)
            {
                LogMessage(LOG_LEVEL_ERROR, "Attempted to verify ARM while previous command not yet verified");
                iCommSend(COMM_DISARM, nullptr, 0);
            }
            break;
        case COMM_STRT:
            try {
                state.set(SupervisionState::RUNNING);
            }
            catch (std::invalid_argument e)
            {
                LogMessage(LOG_LEVEL_ERROR, "START command received while not ready: sending ABORT");
                iCommSend(COMM_ABORT, nullptr, 0);
                state.set(SupervisionState::READY);
            }

            break;
        case COMM_DISCONNECT:
        case COMM_STOP:
        case COMM_ABORT:
            state.set(SupervisionState::READY);
            break;
        case COMM_INV:
            // TODO sleep?
            break;
        case COMM_OBC_STATE:
            break;
        case COMM_GETSTATUS:
            memset(mqSendData, 0, sizeof (mqSendData));
            sprintf(mqSendData, "%s", MODULE_NAME);
            if (iCommSend(COMM_GETSTATUS_OK, mqSendData, sizeof (mqSendData)) < 0) {
                LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending GETSTATUS.");
            }
            break;
        case COMM_GETSTATUS_OK:
            break;
        default:
            LogMessage(LOG_LEVEL_INFO,"Received unhandled command %u",command);
        }
    }

    iCommClose();
    return 0;
}

void signalHandler(int signo) {
    if (signo == SIGINT) {
        LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
        quit = true;
    }
    else {
        LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
    }
}


/*!
* \brief Open a directory and look for trajectory files which are then parsed.
* \param Trajectories A vector of trajectories to be filled
*/
void loadTrajectoryFiles(std::vector<Trajectory> &trajectories) {

    struct dirent *ent;
    DIR *dir;
    unsigned int n = 0;
    char trajectoryPathDir[MAX_FILE_PATH];
    UtilGetTrajDirectoryPath(trajectoryPathDir, sizeof (trajectoryPathDir));
    LogMessage(LOG_LEVEL_DEBUG, "Loading trajectories");

    dir = opendir(trajectoryPathDir);
    if (dir == nullptr) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open trajectory directory");
        throw std::invalid_argument("Cannot open trajectory directory");
    }

    // Count the number of trajectory files in the directory
    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_type == DT_REG) {
            n++;
        }
    }
    closedir(dir);

    LogMessage(LOG_LEVEL_DEBUG, "Found %u trajectory files: proceeding to parse", n);

    dir = opendir(trajectoryPathDir);
    if (dir == nullptr) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open trajectory directory");
        throw std::invalid_argument("Cannot open trajectory directory");
    }

    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_type != DT_REG) {
            LogMessage(LOG_LEVEL_DEBUG, "Ignored <%s>", ent->d_name);
        }
        else {
            try {
                Trajectory trajectory;
                trajectory.initializeFromFile(ent->d_name);
                trajectories.push_back(trajectory); // This does not copy IP value correctly
                LogMessage(LOG_LEVEL_DEBUG, "Loaded trajectory with %u points", trajectories.back().points.size());
            } catch (std::invalid_argument e) {
                closedir(dir);
                trajectories.clear();
                LogMessage(LOG_LEVEL_ERROR, "Error parsing file <%s>", ent->d_name);
                throw;
            } catch (std::ifstream::failure e) {
                closedir(dir);
                trajectories.clear();
                LogMessage(LOG_LEVEL_ERROR, "Error opening file <%s>", ent->d_name);
                throw;
            }
        }
    }
    closedir(dir);
    LogMessage(LOG_LEVEL_INFO, "Loaded %d trajectories", trajectories.size());
    return;
}

/*!
* \brief loadGeofenceFiles Open a directory and look for .geofence files which are then passed to parseGeofenceFile().
* \param geofences A vector of Geofence objects used for saving data
*/
void loadGeofenceFiles(std::vector<Geofence> &geofences) {

    struct dirent *pDirent;
    DIR *pDir;
    char *ext;
    unsigned int n = 0;
    char geofencePathDir[MAX_FILE_PATH];
    UtilGetGeofenceDirectoryPath(geofencePathDir, sizeof (geofencePathDir));
    LogMessage(LOG_LEVEL_DEBUG, "Loading geofences");

    pDir = opendir(geofencePathDir);
    if (pDir == nullptr) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open geofence directory");
        throw std::invalid_argument("Cannot open geofence directory");
    }

    // Count the nuber of geofence files in the directory
    while ((pDirent = readdir(pDir)) != nullptr) {
        ext = strrchr(pDirent->d_name, '.');
        if (strcmp(ext, ".geofence") == 0) {
            n++;
        }
    }
    closedir(pDir);

    LogMessage(LOG_LEVEL_DEBUG, "Found %u geofence files: proceeding to parse", n);

    pDir = opendir(geofencePathDir);
    if (pDir == nullptr) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open geofence directory");
        throw std::invalid_argument("Cannot open geofence directory");
    }

    while ((pDirent = readdir(pDir)) != nullptr) {
        ext = strrchr(pDirent->d_name, '.');
        if (strcmp(ext, ".geofence") != 0 && strcmp(pDirent->d_name, ".") != 0
            && strcmp(pDirent->d_name, "..") != 0) {
            LogMessage(LOG_LEVEL_WARNING, "File <%s> is not a valid .geofence file", pDirent->d_name);
        }
        else if (strcmp(pDirent->d_name, ".") == 0 || strcmp(pDirent->d_name, "..") == 0) {
            LogMessage(LOG_LEVEL_DEBUG, "Ignored <%s>", pDirent->d_name);
        }
        else {
            try {
                Geofence geofence = parseGeofenceFile(pDirent->d_name);
                geofences.push_back(geofence);
            } catch (std::invalid_argument e) {
                closedir(pDir);
                geofences.clear();
                LogMessage(LOG_LEVEL_ERROR, "Error parsing file <%s>", pDirent->d_name);
                throw;
            } catch (std::ifstream::failure e) {
                closedir(pDir);
                geofences.clear();
                LogMessage(LOG_LEVEL_ERROR, "Error opening file <%s>", pDirent->d_name);
                throw;
            }

            LogMessage(LOG_LEVEL_DEBUG, "Loaded geofence with %u vertices", geofences.back().polygonPoints.size());
        }
    }
    closedir(pDir);
    LogMessage(LOG_LEVEL_INFO, "Loaded %d geofences", geofences.size());

    return;
}

/*!
* \brief parseGeofenceFile Parse a geofence file into a Geofence object
* \param geofenceFile A string containing a .geofence filename.
* \return A Geofence object representing the data in the input file
*/
Geofence parseGeofenceFile(const std::string geofenceFile) {

    using namespace std;
    Geofence geofence;
    char geofenceDirPath[MAX_FILE_PATH];
    ifstream file;
    string errMsg;

    string floatPattern("[-+]?[0-9]*\\.?[0-9]+");
    string intPattern("[0-9]+");
    regex headerPattern("GEOFENCE;([a-zA-Z0-9]+);(" + intPattern
                        +");(permitted|forbidden);(" + floatPattern + ");(" + floatPattern + ");");
    regex linePattern("LINE;(" + floatPattern + ");(" + floatPattern + ");ENDLINE;");
    regex footerPattern("ENDGEOFENCE;");
    smatch match;

    bool isHeaderParsedSuccessfully = false;
    unsigned long nPoints = 0;

    UtilGetGeofenceDirectoryPath(geofenceDirPath, sizeof (geofenceDirPath));
    string geofenceFilePath(geofenceDirPath);
    geofenceFilePath += geofenceFile;

    file.open(geofenceFilePath);
    if (file.is_open()) {
        string line;
        errMsg = "Encountered unexpected end of file while reading file <" + geofenceFilePath + ">";
        for (unsigned long lineCount = 0; getline(file, line); lineCount++) {
            if (lineCount == 0) {
                if (regex_search(line, match, headerPattern)) {
                    geofence.name = match[1];
                    nPoints = stoul(match[2]);
                    geofence.polygonPoints.reserve(nPoints);
                    geofence.isPermitted = match[3].compare("permitted") == 0;
                    geofence.minHeight = stod(match[4]);
                    geofence.minHeight = stod(match[5]);
                    isHeaderParsedSuccessfully = true;
                }
                else {
                    errMsg = "The header of geofence file <" + geofenceFilePath + "> is badly formatted";
                    break;
                }
            }
            else if (lineCount > 0 && !isHeaderParsedSuccessfully) {
                errMsg = "Attempt to parse geofence file <" + geofenceFilePath + "> before encountering header";
                break;
            }
            else if (lineCount > nPoints + 1) {
                errMsg = "Geofence line count of file <" + geofenceFilePath
                        + "> does not match specified line count";
                break;
            }
            else if (lineCount == nPoints + 1) {
                if (regex_search(line, match, footerPattern)) {
                    file.close();
                    LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", geofenceFilePath.c_str());
                    return geofence;
                }
                else {
                    errMsg = "Final line of geofence file <" + geofenceFilePath + "> badly formatted";
                    break;
                }
            }
            else {
                if (regex_search(line, match, linePattern)) {
                    CartesianPosition pos;
                    pos.xCoord_m = stod(match[1]);
                    pos.yCoord_m = stod(match[2]);
                    pos.zCoord_m = (geofence.maxHeight + geofence.minHeight) / 2.0;
					pos.isPositionValid = true;
					pos.heading_rad = 0;
					pos.isHeadingValid = false;

                    LogMessage(LOG_LEVEL_DEBUG, "Point: (%.3f, %.3f, %.3f)",
                               pos.xCoord_m,
                               pos.yCoord_m,
                               pos.zCoord_m);
                    geofence.polygonPoints.push_back(pos);
                }
                else {
                    errMsg = "Line " + to_string(lineCount) + " of geofence file <"
                            + geofenceFilePath + "> badly formatted";
                    break;
                }
            }

        }
        file.close();
        LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", geofenceFilePath.c_str());
        LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
        throw invalid_argument(errMsg);
    }
    else {
        errMsg = "Unable to open file <" + geofenceFilePath + ">";
        LogMessage(LOG_LEVEL_ERROR,errMsg.c_str());
        throw ifstream::failure(errMsg);
    }
}


/*!
 * \brief SupervisionCheckGeofences Checks all geofences to verify that the point represented by the MONR data lies within all permitted geofences and outside all forbidden geofences
 * \param monitorData monitor data struct containing the object coordinate data
 * \param geofences Vector containing all geofences
 * \return True if MONR coordinate violates a geofence, false if not.
 */
bool isViolatingGeofence(const ObjectDataType &monitorData, std::vector<Geofence> geofences) {

	const CartesianPosition monitorPoint = monitorData.MonrData.position;
    char isInPolygon = 0;
    int retval = false;

    for (Geofence geofence : geofences)
    {
		isInPolygon = UtilIsPointInPolygon(monitorPoint, geofence.polygonPoints.data(),
                                     static_cast<unsigned int>(geofence.polygonPoints.size()));
        if (isInPolygon == -1) {
            LogMessage(LOG_LEVEL_WARNING, "No points in polygon");
            throw std::invalid_argument("No points in polygon");
        }

        if ((geofence.isPermitted && isInPolygon)
                || (!geofence.isPermitted && !isInPolygon)) {
            // Inside the polygon if it is permitted, alt. outside the polygon if it is forbidden: all is fine
        }
        else {
            if (geofence.isPermitted)
                LogMessage(LOG_LEVEL_WARNING,
						   "Object with ID %u is outside a permitted area %s",
						   monitorData.ClientID, geofence.name.c_str());
            else
                LogMessage(LOG_LEVEL_WARNING,
						   "Object with ID %u is inside a forbidden area %s",
						   monitorData.ClientID, geofence.name.c_str());
            retval = true;
        }
    }

    return retval;
}

/*!
 * \brief updateNearStartingPositionStatus Loops through the armVerified vector for a trajectory that matches the input monitor data packet,
 *  and sets the armVerified boolean to true if the object is near its starting position. It also performs a check on the entire vector
 *  to determine whether all objects have been verified near starting position.
 * \param monitorData Monitor data struct containing the object coordinate data
 * \param armVerified Vector containing trajectories paired with a boolean showing whether or not the associated object has been previously verified
    to be near its starting position
 * \return A value according to ::PositionStatus
 */
PositionStatus updateNearStartingPositionStatus(const ObjectDataType &monitorData, std::vector<std::pair<Trajectory&, bool>> armVerified) {

    char ipString[INET_ADDRSTRLEN];
    for (std::pair<Trajectory&, bool> &element : armVerified) {
		if (element.first.ip == monitorData.ClientIP) {
            if (element.first.points.empty()) {
                element.second = true;
                return OBJECT_HAS_NO_TRAJECTORY;
            }

            CartesianPosition trajectoryPoint = element.first.points.front().getCartesianPosition();
			CartesianPosition objectPosition = monitorData.MonrData.position;
            if (UtilIsPositionNearTarget(objectPosition, trajectoryPoint, ARM_MAX_DISTANCE_TO_START_M)
					&& UtilIsAngleNearTarget(objectPosition, trajectoryPoint, ARM_MAX_ANGLE_TO_START_DEG * M_PI / 180.0)) {
                if (element.second == false) {
                    LogMessage(LOG_LEVEL_INFO, "Object with IP %s and position (%.2f, %.2f, %.2f) detected within %.2f m and %.2f degrees of the first point (%.2f, %.2f, %.2f) in trajectory %s",
							   inet_ntop(AF_INET, &monitorData.ClientIP, ipString, sizeof (ipString)),
                               objectPosition.xCoord_m, objectPosition.yCoord_m, objectPosition.zCoord_m,
                               ARM_MAX_DISTANCE_TO_START_M, ARM_MAX_ANGLE_TO_START_DEG,
                               trajectoryPoint.xCoord_m, trajectoryPoint.yCoord_m, trajectoryPoint.zCoord_m,
                               element.first.name.c_str());
                }
                element.second = true;
                // Object was near starting position, now check if all objects have passed
                if (std::any_of(armVerified.begin(), armVerified.end(),
                                [](const std::pair<Trajectory&, bool> &pair) { return pair.second == false; })) {
                    return SINGLE_OBJECT_NEAR_START;
                }
                else {
                    return ALL_OBJECTS_NEAR_START;
                }
            }
            else {
                if (!UtilIsPositionNearTarget(objectPosition, trajectoryPoint, ARM_MAX_DISTANCE_TO_START_M)) {
                    LogMessage(LOG_LEVEL_INFO, "Object with IP %s and position (%.2f, %.2f, %.2f) farther than %.2f m from first point (%.2f, %.2f, %.2f) in trajectory %s",
								inet_ntop(AF_INET, &monitorData.ClientIP, ipString, sizeof (ipString)),
                                objectPosition.xCoord_m, objectPosition.yCoord_m, objectPosition.zCoord_m,
                                ARM_MAX_DISTANCE_TO_START_M, trajectoryPoint.xCoord_m, trajectoryPoint.yCoord_m, trajectoryPoint.zCoord_m,
                                element.first.points.front().getZCoord(), element.first.name.c_str());
                }
                else {
                    LogMessage(LOG_LEVEL_INFO, "Object with IP %s (heading: %.2f degrees) not facing direction specified by first point (heading: %.2f degrees) in trajectory %s (tolerance: %.2f degrees)",
								inet_ntop(AF_INET, &monitorData.ClientIP, ipString, sizeof (ipString)), objectPosition.heading_rad * 180.0 / M_PI,
							   trajectoryPoint.heading_rad * 180.0 / M_PI, element.first.name.c_str(), ARM_MAX_ANGLE_TO_START_DEG);
                }
                element.second = false;
                return SINGLE_OBJECT_NOT_NEAR_START;
            }
        }
    }
    return OBJECT_HAS_NO_TRAJECTORY;
}
