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

#include "supervision.h"

/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000
#define MAX_GEOFENCE_NAME_LEN 256
#define GEOFENCE_DIRECTORY "./geofence/"

#define MODULE_NAME "Supervisor"

/*------------------------------------------------------------
  -- Type definitions.
  ------------------------------------------------------------*/
typedef struct {
	U16 numberOfPoints;
	I8 isPermitted;
	char name[MAX_GEOFENCE_NAME_LEN];
	CartesianPosition *polygonPoints;
} GeofenceType;


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

int SupervisionCheckGeofences(MonitorDataType MONRdata, GeofenceType * geofences,
							  unsigned int numberOfGeofences);
int loadGeofenceFiles(GeofenceType * geofences[], unsigned int *nGeof);
int parseGeofenceFile(char *geofenceFile, GeofenceType * geofence);
static void signalHandler(int signo);
void freeGeofences(GeofenceType * geoFence, unsigned int *nGeof);

/*------------------------------------------------------------
-- Static variables
------------------------------------------------------------*/
static volatile int iExit = 0;

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervision_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	char busReceiveBuffer[MBUS_MAX_DATALEN];	//!< Buffer for receiving from message bus
	MonitorDataType MONRMessage;

	unsigned int numberOfGeofences = 0;
	GeofenceType *geofenceArray = NULL;

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
			if (geofenceArray != NULL)
				freeGeofences(geofenceArray, &numberOfGeofences);

			if (loadGeofenceFiles(&geofenceArray, &numberOfGeofences) == -1)
				util_error("Unable to load geofences");	// TODO: Do something more e.g. stop the INIT process

			break;
		case COMM_MONI:
			// Ignore old style MONR data
			break;
		case COMM_MONR:
			UtilPopulateMonitorDataStruct(busReceiveBuffer, sizeof (busReceiveBuffer), &MONRMessage, 0);
			// TODO: react to output from SupervisionCheckGeofences
			SupervisionCheckGeofences(MONRMessage, geofenceArray, numberOfGeofences);

			break;
		case COMM_OBC_STATE:
			break;
		case COMM_CONNECT:
			break;
		case COMM_LOG:
			break;
		case COMM_INV:
			break;
		default:
			LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
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

	LogMessage(LOG_LEVEL_DEBUG, "Loading geofences");

	pDir = opendir(GEOFENCE_DIRECTORY);
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

	pDir = opendir(GEOFENCE_DIRECTORY);
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

	char pcFileNameBuffer[MAX_FILE_PATH] = "";

	strcat(pcFileNameBuffer, GEOFENCE_DIRECTORY);
	strcat(pcFileNameBuffer, geofenceFile);

	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;
	int tempInt;
	int isHeaderParsedSuccessfully = 0;

	LogMessage(LOG_LEVEL_DEBUG, "Opening <%s>", pcFileNameBuffer);
	fp = fopen(pcFileNameBuffer, "r");
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
						LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", pcFileNameBuffer);
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
		LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", pcFileNameBuffer);
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcFileNameBuffer);
		return -1;
	}

	// If we reach here, it means we did not find an ENDGEOFENCE before EOF
	LogMessage(LOG_LEVEL_ERROR, "Reached end of file <%s> unexpectedly while parsing", pcFileNameBuffer);
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
MONRdata.MONR.ZPositionI32 / 1000.0, 0.0 };
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
