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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>

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
typedef struct
{
    U16 numberOfPoints;
    I8 isPermitted;
    char name[MAX_GEOFENCE_NAME_LEN];
    CartesianPosition *polygonPoints;
} GeofenceType;


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

int SupervisionCheckGeofences(MONRType MONRdata, GeofenceType *geofences, unsigned int numberOfGeofences);
int loadGeofenceFiles(GeofenceType *geofences[], unsigned int *nGeof);
int parseGeofenceFile(char* geofenceFile, GeofenceType *geofence);

void freeGeofences(GeofenceType *geoFence, unsigned int *nGeof);
void printFences(GeofenceType *geoFence, unsigned int nGeof);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervision_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    MONRType MONRMessage;

    unsigned int numberOfGeofences = 0;
    GeofenceType *geofenceArray = NULL;

    enum COMMAND command;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());

    while(!iExit)
    {
        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));
        (void)iCommRecv(&command,busReceiveBuffer, sizeof(busReceiveBuffer), NULL);
        if (command == COMM_ABORT)
        {
            // TODO:

        }

        if (command == COMM_EXIT)
        {
            iExit = 1;

            freeGeofences(geofenceArray, &numberOfGeofences);

            LogMessage(LOG_LEVEL_INFO, "Supervision exiting...");
            (void)iCommClose();
        }

        switch (command)
        {
        case COMM_INIT:
            if (geofenceArray != NULL)
                freeGeofences(geofenceArray, &numberOfGeofences);

            if (loadGeofenceFiles(&geofenceArray, &numberOfGeofences) == -1)
                util_error("Unable to load geofences");

            break;
        case COMM_MONI:
            // Ignore old style MONR data
            break;
        case COMM_MONR:
            UtilPopulateMONRStruct(busReceiveBuffer, sizeof(busReceiveBuffer), &MONRMessage, 0);
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

/*!
* \brief Open a directory and look for .geofence files which are then passed to parseGeofenceFile().
* \param *geofences A pointer to geofence struct used for saving data
* \param *nGeof A pointer to count number of geofences loaded.
*
*/
int loadGeofenceFiles(GeofenceType *geofences[], unsigned int *nGeof){

    struct dirent *pDirent;
    DIR *pDir;
    char *ext;
    unsigned int n = 0;

    LogMessage(LOG_LEVEL_DEBUG,"Loading geofences");

    pDir = opendir (GEOFENCE_DIRECTORY);
    if (pDir == NULL) {
        LogMessage(LOG_LEVEL_ERROR,"Cannot open geofence directory");
        return -1;
    }

    // Count the nuber of geofence files in the directory
    while ((pDirent = readdir(pDir)) != NULL)
    {
        ext = strrchr(pDirent->d_name, '.');
        if(strcmp(ext, ".geofence") == 0)
        {
            n++;
        }
    }
    closedir (pDir);
    *nGeof = n;

    *geofences = (GeofenceType *)malloc(n * sizeof (GeofenceType));

    if (*geofences == NULL)
    {
        LogMessage(LOG_LEVEL_ERROR,"Unable to allocate memory for geofences");
        return -1;
    }

    LogMessage(LOG_LEVEL_DEBUG, "Found %u geofence files: proceeding to parse", *nGeof);

    pDir = opendir (GEOFENCE_DIRECTORY);
    if (pDir == NULL) {
        LogMessage(LOG_LEVEL_ERROR,"Cannot open geofence directory");
        return -1;
    }

    n = 0;
    while ((pDirent = readdir(pDir)) != NULL) {
        ext = strrchr(pDirent->d_name, '.');
        if(strcmp(ext, ".geofence") != 0 && strcmp(pDirent->d_name,".") != 0 && strcmp(pDirent->d_name,"..") != 0)
        {
            LogMessage(LOG_LEVEL_WARNING, "File <%s> is not a valid .geofence file", pDirent->d_name);
        }
        else
        {
            if(parseGeofenceFile(pDirent->d_name, (*geofences)+n) == -1)
            {
                closedir(pDir);
                return -1;
            }
            n++;
        }
    }
    closedir (pDir);
    LogMessage(LOG_LEVEL_INFO, "Loaded %d geofences",*nGeof);

    return 0;
}


/*!
* \brief Open a directory and look for .geofence files which are then passed to parseGeofenceFile().
* \param *geofenceFile A string containing a .geofence filename.
* \param *geofence A pointer to the geofence struct used for saving data.
* * \param index An integer used to keep track of which index to store data in.
*
*/
int parseGeofenceFile(char* geofenceFile, GeofenceType *geofence){

    char pcFileNameBuffer[MAX_FILE_PATH] = "";
    strcat(pcFileNameBuffer, GEOFENCE_DIRECTORY);
    strcat(pcFileNameBuffer, geofenceFile);

    FILE *fp;
    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    LogMessage(LOG_LEVEL_DEBUG, "Opening <%s>", geofenceFile);
    fp = fopen(pcFileNameBuffer, "r");
    if ( fp != NULL )
    {
        while ((read = getline(&line, &len, fp)) != -1) {
            static int lineCount = 0;
            char delim[] = ";";
            char *ptr = strtok(line, delim);

            while (ptr != NULL)
            {
                if(strcmp( ptr,  "GEOFENCE" ) == 0){ //PARSE HEADER
                    lineCount = 0;

                    ptr = strtok(NULL, delim);
                    strcpy (geofence->name, ptr);

                    ptr = strtok(NULL, delim);
                    geofence->numberOfPoints = atoi(ptr);

                    geofence->polygonPoints = (CartesianPosition*)malloc(geofence->numberOfPoints*sizeof(CartesianPosition));
                    if (geofence->polygonPoints == NULL)
                    {
                        LogMessage(LOG_LEVEL_ERROR, "Unable to allocate memory for coordinate array");
                        return -1;
                    }
                    ptr = strtok(NULL, delim);

                    if(strcmp( ptr,  "permitted" ) == 0){
                        geofence->isPermitted = 1;
                    }
                    else
                    {
                        geofence->isPermitted = 0;
                    }

                    ptr = strtok(NULL, delim);
                    // printf("Min Height: %s\n", ptr);
                    ptr = strtok(NULL, delim);
                    // printf("Max Height: %s\n", ptr);
                    ptr = strtok(NULL, delim);
                }

                if(strcmp( ptr, "LINE" ) == 0){
                    ptr = strtok(NULL, delim);
                    geofence->polygonPoints[lineCount].xCoord_m = atof(ptr);

                    ptr = strtok(NULL, delim);
                    geofence->polygonPoints[lineCount].yCoord_m = atof(ptr);

                    lineCount++;
                }
                ptr = strtok(NULL, delim);

            }
        }


        fclose ( fp );
        LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", pcFileNameBuffer);
    }
    else
    {
        LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", pcFileNameBuffer);
        return -1;
    }
    return 0;
}

/*!
 * \brief SupervisionCheckGeofences Checks all geofences to verify that the point represented by the MONR data lies within all permitted geofences and outside all forbidden geofences
 * \param MONRdata MONR struct containing the object coordinate data
 * \param geofences Struct array containing all geofences
 * \param numberOfGeofences Length of struct array
 * \return 1 if MONR coordinate violates a geofence, 0 if not. -1 on error
 */
int SupervisionCheckGeofences(MONRType MONRdata, GeofenceType *geofences, unsigned int numberOfGeofences)
{
    const CartesianPosition monrPoint = {MONRdata.XPositionI32/1000.0, MONRdata.YPositionI32/1000.0, MONRdata.ZPositionI32/1000.0, 0.0};
    char isInPolygon = 0;
    int retval = 0;

    for (unsigned int i = 0; i < numberOfGeofences; i++)
    {
        isInPolygon = UtilIsPointInPolygon(monrPoint, geofences[i].polygonPoints, geofences[i].numberOfPoints);

        if (isInPolygon == -1)
        {
            LogMessage(LOG_LEVEL_WARNING,"No points in polygon");
            return -1;
        }

        if ( (geofences[i].isPermitted && isInPolygon)
             || (!geofences[i].isPermitted && !isInPolygon) )
        {
            // Inside the polygon if it is permitted, alt. outside the polygon if it is forbidden: all is fine
        }
        else
        {
            if (geofences[i].isPermitted)
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is outside a permitted area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);
            else
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is inside a forbidden area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);

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
void freeGeofences(GeofenceType *geofences, unsigned int *nGeofences){
    for(unsigned int i = 0; i < *nGeofences; i++){
        free(geofences[i].polygonPoints);
    }
    free(geofences);
    *nGeofences = 0;
}
