/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : supervision.c
  -- Author      :
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <dirent.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>
#include <limits.h>

#include "maestroTime.h"
#include "supervision.h"

#define MAX_GEOFENCE_NAME_LEN 256

#define MODULE_NAME "Supervisor"

#define VERIFY_ARM_TIMEOUT_MS 500

/*------------------------------------------------------------
  -- Type definitions.
  ------------------------------------------------------------*/
typedef struct {
	U16 numberOfPoints;
	I8 isPermitted;
	char name[MAX_GEOFENCE_NAME_LEN];
	CartesianPosition *polygonPoints;
} GeofenceType;

typedef struct {
    double time;                            //!< Time from start of test [s]
    CartesianPosition *point;
    double longitudinalSpeed;               //!< Speed in the direction of the heading [m/s]
    double lateralSpeed;                    //!< Speed in the direction perpendicular to the heading [m/s]
    double longitudinalAcceleration;        //!< Acceleration in the direction of the heading [m/s²]
    double lateralAcceleration;             //!< Acceleration in the direction perpendicular to the heading [m/s²]
    double curvature;                       //!< Curvature of the curve [1/m]
    uint8_t mode;                           //!< Value describing if the object is controlled by the drive file
} TrajPointType;

typedef enum {
    READY = 0,
    VERIFYING_INIT = 1,
    VERIFYING_ARM = 2
} SupervisorStateType;
static char * const stateNames[] = {"READY", "INIT", "ARM"};

typedef struct {
    in_addr_t ipAddr;
    double length;
    double width;
    double height;
    double mass;
    double turnRadius;
} ObjectPropertyType;


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

static SupervisorStateType state = READY;
static SupervisorStateType getState(void);
static int setState(SupervisorStateType newState);

int SupervisionCheckGeofences(MonitorDataType MONRdata, GeofenceType * geofences,
							  unsigned int numberOfGeofences);
int loadGeofenceFiles(GeofenceType * geofences[], unsigned int *nGeof);
int parseGeofenceFile(char *geofenceFile, GeofenceType * geofence);
void freeGeofences(GeofenceType * geoFence, unsigned int *nGeof);

static int verifyArm(MonitorDataType MONRdata, ObjectPropertyType objectProperties[], unsigned int nObjects);
static int initializeObjectProperties(ObjectPropertyType* objectProperties[], unsigned int *nObjects);
static void resetVerifiedStatus(unsigned int numberOfObjects);
static int checkIfNearFirstPointOfTrajectory(MonitorDataType MONRdata, ObjectPropertyType objectProperties);

static void signalHandler(int signo);

/*------------------------------------------------------------
-- Static variables
------------------------------------------------------------*/
static volatile int iExit = 0;
static uint8_t* isVerified = NULL;

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervision_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	char busReceiveBuffer[MBUS_MAX_DATALEN];	//!< Buffer for receiving from message bus
	MonitorDataType MONRMessage;

	unsigned int numberOfGeofences = 0;
	GeofenceType *geofenceArray = NULL;

    unsigned int numberOfObjects = 0;
    ObjectPropertyType *objectProperties = NULL;

    struct timeval currentTime, verificationTimeout;
    const struct timeval armWaitTime = {VERIFY_ARM_TIMEOUT_MS % 1000, (VERIFY_ARM_TIMEOUT_MS * 1000) % 1000000};

	enum COMMAND command;

	// Create log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());

	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Set up message bus connection
	if (iCommInit())
		util_error("Unable to connect to message queue bus");

	while (!iExit) {
		bzero(busReceiveBuffer, sizeof (busReceiveBuffer));
		(void)iCommRecv(&command, busReceiveBuffer, sizeof (busReceiveBuffer), NULL);
		if (command == COMM_ABORT) {
			// TODO:

		}

		if (command == COMM_EXIT) {
			iExit = 1;

			freeGeofences(geofenceArray, &numberOfGeofences);

			LogMessage(LOG_LEVEL_INFO, "Supervision exiting...");
			(void)iCommClose();
		}

		switch (command) {
        case COMM_INIT:
            setState(VERIFYING_INIT);
			if (geofenceArray != NULL)
				freeGeofences(geofenceArray, &numberOfGeofences);

			if (loadGeofenceFiles(&geofenceArray, &numberOfGeofences) == -1)
                util_error("Unable to load geofences");	// TODO: Do something more e.g. stop the INIT process

            if (initializeObjectProperties(&objectProperties, &numberOfObjects) == -1) {
                util_error("Unable to initialize object properties");
            }
            setState(READY);
			break;
		case COMM_MONI:
			// Ignore old style MONR data
			break;
		case COMM_MONR:
			UtilPopulateMonitorDataStruct(busReceiveBuffer, sizeof (busReceiveBuffer), &MONRMessage, 0);
			// TODO: react to output from SupervisionCheckGeofences
			SupervisionCheckGeofences(MONRMessage, geofenceArray, numberOfGeofences);

            if (state == VERIFYING_ARM) {
                switch (verifyArm(MONRMessage, objectProperties, numberOfObjects)) {
                case -1:
                    LogMessage(LOG_LEVEL_INFO, "ARM should not be allowed - objects not in position");
                    // TODO: There was an error - ARM is not ok
                    break;
                case 0:
                    setState(READY);
                    break;
                case 1:
                    // Nothing to do except to wait for more MONR
                    break;
                }
            }

			break;
		case COMM_OBC_STATE:
			break;
		case COMM_CONNECT:
			break;
        case COMM_ARMD:
            setState(VERIFYING_ARM);
            TimeSetToCurrentSystemTime(&verificationTimeout);
            timeradd(&verificationTimeout, &armWaitTime, &verificationTimeout);
            // TODO: Check one MONR of each object against first point of their traj files
            // TODO:
            break;
		case COMM_LOG:
			break;
		case COMM_INV:
			break;
		default:
			LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
		}

        TimeSetToCurrentSystemTime(&currentTime);
        if (timercmp(&currentTime,&verificationTimeout,>)) {
            LogMessage(LOG_LEVEL_WARNING,"Timed out while verifying %s",stateNames[state]);
        }
	}
}


/*------------------------------------------------------------
-- Private functions
------------------------------------------------------------*/
void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		iExit = 1;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}


int setState(SupervisorStateType newState)
{
    switch (newState) {
    case VERIFYING_INIT:
    case VERIFYING_ARM:
        if (state == READY)
            state = newState;
        else {
            LogMessage(LOG_LEVEL_ERROR, "Received command %s to verify while still not done verifying %s",
                       stateNames[newState], stateNames[state]);
            // TODO: Do something more
        }
        break;
    case READY:
        state = newState;
    }
}

/*!
 * \brief SupervisionVerifyArm
 * \param MONRdata
 * \return 0 if ARM command verified
 */
int verifyArm(MonitorDataType MONRdata, ObjectPropertyType objectProperties[], unsigned int numberOfObjects) {

    int allVerified = 1;
    char ipString[INET_ADDRSTRLEN];

    if (isVerified == NULL) {
        LogMessage(LOG_LEVEL_ERROR, "Received MONR before initialized properly");
        return -1;
    }

    for (unsigned int i = 0; i < numberOfObjects; ++i) {
        if (MONRdata.ClientIP == objectProperties[i].ipAddr) {
            if (checkIfNearFirstPointOfTrajectory(MONRdata, objectProperties[i]) == 0) {
                isVerified[i] = 1;
            }
            else {
                inet_ntop(AF_INET, &objectProperties[i].ipAddr, ipString, sizeof (ipString));
                LogMessage(LOG_LEVEL_INFO, "Object with IP %s is not near its starting position", ipString);
                resetVerifiedStatus(numberOfObjects);
                return -1;
            }
        }
        allVerified = allVerified && isVerified[i];
    }

    if (allVerified) {
        resetVerifiedStatus(numberOfObjects);
        return 0;
    }
    return 1;
}

/*!
 * \brief checkNearFirstPointOfTrajectory
 * \param MONRdata
 * \param objectProperties
 * \return
 */
int checkIfNearFirstPointOfTrajectory(MonitorDataType MONRdata, ObjectPropertyType objectProperties) {

    // TODO: Implement this in a better way
    // Hardcoded values for now
    struct dirent *ent;
    DIR *dir;
    TrajPointType firstTrajPoint;
    char ipString[INET_ADDRSTRLEN];
    char trajPathDir[MAX_FILE_PATH];
    inet_ntop(AF_INET, &objectProperties.ipAddr, ipString, sizeof (ipString));
    UtilGetTrajDirectoryPath(trajPathDir, sizeof (trajPathDir));
    LogMessage(LOG_LEVEL_DEBUG, "Opening trajectory directory");
    dir = opendir(trajPathDir);
    if (dir == NULL) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open trajectory directory");
        return -1;
    }

    while ((ent = readdir(dir)) != NULL) {
        if (!strcmp(ent->d_name,".") || !strcmp(ent->d_name,".."))
            continue;
        else if (!strcmp(ent->d_name, ipString)) {
            // TODO: read corresponding file first row of traj
            firstTrajPoint.time = 0.0;
            firstTrajPoint.point->xCoord_m = 0.0;
            firstTrajPoint.point->yCoord_m = 0.0;
            firstTrajPoint.point->zCoord_m = 0.0;
            firstTrajPoint.longitudinalSpeed = 0.0;
            firstTrajPoint.lateralSpeed = 0.0;
            firstTrajPoint.longitudinalAcceleration = 0.0;
            firstTrajPoint.lateralAcceleration = 0.0;
            firstTrajPoint.curvature = 0.0;
            firstTrajPoint.mode = 0;

        }
    }
    closedir(dir);

}

/*!
 * \brief resetVerifiedStatus
 * \param numberOfObjects
 */
void resetVerifiedStatus(unsigned int numberOfObjects)
{
    for (unsigned int i = 0; i < numberOfObjects; ++i) {
        isVerified[i] = 0;
    }
}


int initializeObjectProperties(ObjectPropertyType* objectProperties[], unsigned int *nObjects) {

    struct dirent *ent;
    DIR *dir;
    char trajPathDir[MAX_FILE_PATH];
    unsigned int nTrajs = 0;

    UtilGetTrajDirectoryPath(trajPathDir, sizeof (trajPathDir));
    LogMessage(LOG_LEVEL_DEBUG, "Opening trajectory directory");
    dir = opendir(trajPathDir);
    if (dir == NULL) {
        LogMessage(LOG_LEVEL_ERROR, "Cannot open trajectory directory");
        return -1;
    }

    while ((ent = readdir(dir)) != NULL) {
        if (!strcmp(ent->d_name,".") || !strcmp(ent->d_name,".."))
            continue;
        else
            nTrajs++;
    }
    closedir(dir);

    *nObjects = nTrajs;
    free(*objectProperties);
    *objectProperties = (ObjectPropertyType*) malloc(nTrajs*sizeof (ObjectPropertyType));
    if (*objectProperties == NULL) {
        LogMessage(LOG_LEVEL_ERROR, "Unable to allocate memory for object properties");
        return -1;
    }

    // TODO: fill in the IPs

    free(isVerified);
    isVerified = malloc(nTrajs * sizeof (uint8_t));
    resetVerifiedStatus(nTrajs);

    LogMessage(LOG_LEVEL_DEBUG, "Found %u trajectory files", nTrajs);
    return 0;
}


/*!
* \brief Open a directory and look for .geofence files which are then passed to parseGeofenceFile().
* \param *geofences A pointer to geofence struct used for saving data
* \param *nGeof A pointer to count number of geofences loaded.
*
*/
int loadGeofenceFiles(GeofenceType * geofences[], unsigned int *nGeof) {

	struct dirent *pDirent;
	DIR *pDir;
	char *ext;
	unsigned int n = 0;
	char geofencePathDir[MAX_FILE_PATH];

	UtilGetGeofenceDirectoryPath(geofencePathDir, sizeof (geofencePathDir));

	LogMessage(LOG_LEVEL_DEBUG, "Loading geofences");

	pDir = opendir(geofencePathDir);
	if (pDir == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Cannot open geofence directory");
		return -1;
	}

	// Count the nuber of geofence files in the directory
	while ((pDirent = readdir(pDir)) != NULL) {
		ext = strrchr(pDirent->d_name, '.');
		if (strcmp(ext, ".geofence") == 0) {
			n++;
		}
	}
	closedir(pDir);
	*nGeof = n;

	*geofences = (GeofenceType *) malloc(n * sizeof (GeofenceType));

	if (*geofences == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to allocate memory for geofences");
		return -1;
	}

	LogMessage(LOG_LEVEL_DEBUG, "Found %u geofence files: proceeding to parse", *nGeof);

	pDir = opendir(geofencePathDir);
	if (pDir == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Cannot open geofence directory");
		return -1;
	}

	n = 0;
	while ((pDirent = readdir(pDir)) != NULL) {
		ext = strrchr(pDirent->d_name, '.');
		if (strcmp(ext, ".geofence") != 0 && strcmp(pDirent->d_name, ".") != 0
			&& strcmp(pDirent->d_name, "..") != 0) {
			LogMessage(LOG_LEVEL_WARNING, "File <%s> is not a valid .geofence file", pDirent->d_name);
		}
		else if (strcmp(pDirent->d_name, ".") == 0 || strcmp(pDirent->d_name, "..") == 0) {
			LogMessage(LOG_LEVEL_DEBUG, "Ignored <%s>", pDirent->d_name);
		}
		else {
			if (parseGeofenceFile(pDirent->d_name, (*geofences) + n) == -1) {
				closedir(pDir);
				LogMessage(LOG_LEVEL_ERROR, "Error parsing file <%s>", pDirent->d_name);
				return -1;
			}
			LogMessage(LOG_LEVEL_DEBUG, "Loaded geofence with %u vertices", (*geofences)[n].numberOfPoints);
			n++;
		}
	}
	closedir(pDir);
	LogMessage(LOG_LEVEL_INFO, "Loaded %d geofences", *nGeof);

	return 0;
}


/*!
* \brief Open a directory and look for .geofence files which are then passed to parseGeofenceFile().
* \param *geofenceFile A string containing a .geofence filename.
* \param *geofence A pointer to the geofence struct used for saving data.
* \param index An integer used to keep track of which index to store data in.
* \return 0 on success, -1 on failure
*/
int parseGeofenceFile(char *geofenceFile, GeofenceType * geofence) {

	char geofencePathDir[MAX_FILE_PATH];
	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;
	int tempInt;
	int isHeaderParsedSuccessfully = 0;

	UtilGetGeofenceDirectoryPath(geofencePathDir, sizeof (geofencePathDir));
	strcat(geofencePathDir, geofenceFile);

	LogMessage(LOG_LEVEL_DEBUG, "Opening <%s>", geofencePathDir);
	fp = fopen(geofencePathDir, "r");

	if (fp != NULL) {
		int lineCount = 0;

		while ((read = getline(&line, &len, fp)) != -1) {
			char delim[] = ";";
			char *ptr = strtok(line, delim);

			while (ptr != NULL) {
				if (strcmp(ptr, "GEOFENCE") == 0 && strcmp(ptr, "ENDGEOFENCE") != 0 && lineCount != 0) {
					// In case there's a second header somewhere in the middle of the file, catch that
					LogMessage(LOG_LEVEL_ERROR, "Found misplaced header in file <%s>", geofenceFile);
					fclose(fp);
					if (isHeaderParsedSuccessfully)
						free(geofence->polygonPoints);
					return -1;
				}
				else if (strcmp(ptr, "GEOFENCE") == 0 && strcmp(ptr, "ENDGEOFENCE") != 0) {
					/* Parse header */

					// Geofence name string
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}
					strcpy(geofence->name, ptr);

					// Geofence number of points
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}
					tempInt = atoi(ptr);
					if (tempInt < 0) {
						LogMessage(LOG_LEVEL_ERROR,
								   "Header of file <%s> contains a negative number of points", geofenceFile);
						fclose(fp);
						return -1;
					}
					else if (tempInt > USHRT_MAX) {
						LogMessage(LOG_LEVEL_ERROR, "Geofence file <%s> contains too many points",
								   geofenceFile);
						fclose(fp);
						return -1;
					}
					geofence->numberOfPoints = (unsigned short)tempInt;

					// Permitted|forbidden string
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}

					if (strcasecmp(ptr, "permitted") == 0) {
						geofence->isPermitted = 1;
					}
					else if (strcasecmp(ptr, "forbidden") == 0) {
						geofence->isPermitted = 0;
					}
					else {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}

					// Minimum height
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}
					LogMessage(LOG_LEVEL_DEBUG, "Ignored minimum height %s in geofence <%s> (unimplemented)",
							   ptr, geofenceFile);

					// Maximum height
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}
					LogMessage(LOG_LEVEL_DEBUG, "Ignored maximum height %s in geofence <%s> (unimplemented)",
							   ptr, geofenceFile);

					// Final delimiter
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse header of file <%s>", geofenceFile);
						fclose(fp);
						return -1;
					}

					// Successfully parsed the header; now we can allocate memory for the rest
					geofence->polygonPoints =
						(CartesianPosition *) malloc(geofence->numberOfPoints * sizeof (CartesianPosition));
					if (geofence->polygonPoints == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to allocate memory for coordinate array");
						fclose(fp);
						return -1;
					}

					isHeaderParsedSuccessfully = 1;
				}
				else if ((strcmp(ptr, "LINE") == 0) && !isHeaderParsedSuccessfully) {
					// In case there's a point before the header, catch that
					LogMessage(LOG_LEVEL_ERROR, "Encountered geofence point above header in file <%s>",
							   geofenceFile);
					fclose(fp);
					return -1;
				}
				else if (strcmp(ptr, "LINE") == 0) {
					/* Parse non-header line */

					// Check so that there are not more points than previously specified
					if (lineCount >= geofence->numberOfPoints) {
						LogMessage(LOG_LEVEL_ERROR,
								   "Geofence file <%s> contains more rows than specified in the header",
								   geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}

					// Parse x coordinate
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse x coordinate in row %d of file <%s>",
								   lineCount + 2, geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}
					geofence->polygonPoints[lineCount].xCoord_m = atof(ptr);

					// Parse y coordinate
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse y coordinate in row %d of file <%s>",
								   lineCount + 2, geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}
					geofence->polygonPoints[lineCount].yCoord_m = atof(ptr);

					LogMessage(LOG_LEVEL_DEBUG, "Point: (%.3f, %.3f)",
							   geofence->polygonPoints[lineCount].xCoord_m,
							   geofence->polygonPoints[lineCount].yCoord_m);

					// Check so that there is an ENDLINE text
					if ((ptr = strtok(NULL, delim)) == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Could not find ENDLINE at row %d of file <%s>",
								   lineCount + 2, geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}

					if (strcmp(ptr, "ENDLINE") != 0) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to parse row %d of file <%s>", lineCount + 2,
								   geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}

					// Increment the line counter
					lineCount++;
				}
				else if (strcmp(ptr, "ENDGEOFENCE") == 0 && !isHeaderParsedSuccessfully) {
					LogMessage(LOG_LEVEL_ERROR, "Found misplaced ENDGEOFENCE in file <%s>", geofenceFile);
					fclose(fp);
					return -1;
				}
				else if (strcmp(ptr, "ENDGEOFENCE") == 0) {
					if (lineCount == geofence->numberOfPoints) {
						/* Successful parse, return */
						fclose(fp);
						LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", geofencePathDir);
						return 0;
					}
					else {
						LogMessage(LOG_LEVEL_ERROR,
								   "Mismatch between specified number of points (%u) and row count (%d) in file <%s>",
								   geofence->numberOfPoints, lineCount, geofenceFile);
						fclose(fp);
						free(geofence->polygonPoints);
						return -1;
					}
				}
				ptr = strtok(NULL, delim);
			}
		}

		fclose(fp);
		LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", geofencePathDir);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", geofencePathDir);
		return -1;
	}

	// If we reach here, it means we did not find an ENDGEOFENCE before EOF
	LogMessage(LOG_LEVEL_ERROR, "Reached end of file <%s> unexpectedly while parsing", geofencePathDir);
	if (isHeaderParsedSuccessfully)
		free(geofence->polygonPoints);
	return -1;
}

/*!
 * \brief SupervisionCheckGeofences Checks all geofences to verify that the point represented by the MONR data lies within all permitted geofences and outside all forbidden geofences
 * \param MONRdata MONR struct containing the object coordinate data
 * \param geofences Struct array containing all geofences
 * \param numberOfGeofences Length of struct array
 * \return 1 if MONR coordinate violates a geofence, 0 if not. -1 on error
 */
int SupervisionCheckGeofences(MonitorDataType MONRdata, GeofenceType * geofences,
							  unsigned int numberOfGeofences) {
	const CartesianPosition monrPoint =
		{ MONRdata.MONR.XPositionI32 / 1000.0, MONRdata.MONR.YPositionI32 / 1000.0,
		MONRdata.MONR.ZPositionI32 / 1000.0, 0.0
	};
	char isInPolygon = 0;
	int retval = 0;

	for (unsigned int i = 0; i < numberOfGeofences; i++) {
		isInPolygon =
			UtilIsPointInPolygon(monrPoint, geofences[i].polygonPoints, geofences[i].numberOfPoints);

		if (isInPolygon == -1) {
			LogMessage(LOG_LEVEL_WARNING, "No points in polygon");
			return -1;
		}

		if ((geofences[i].isPermitted && isInPolygon)
			|| (!geofences[i].isPermitted && !isInPolygon)) {
			// Inside the polygon if it is permitted, alt. outside the polygon if it is forbidden: all is fine
		}
		else {
			if (geofences[i].isPermitted)
				LogMessage(LOG_LEVEL_WARNING,
						   "Object with MONR transmitter ID %u is outside a permitted area %s",
						   MONRdata.MONR.Header.TransmitterIdU8, geofences[i].name);
			else
				LogMessage(LOG_LEVEL_WARNING,
						   "Object with MONR transmitter ID %u is inside a forbidden area %s",
						   MONRdata.MONR.Header.TransmitterIdU8, geofences[i].name);

			retval = 1;
		}
	}

	return retval;
}

/*!
 * \brief freeGeofences Frees allocated memory for geofence struct array
 * \param geofences Struct array containing geofences
 * \param nGeofences Length of struct array
 */
void freeGeofences(GeofenceType * geofences, unsigned int *nGeofences) {
	for (unsigned int i = 0; i < *nGeofences; i++) {
		free(geofences[i].polygonPoints);
	}
	free(geofences);
	*nGeofences = 0;
}
