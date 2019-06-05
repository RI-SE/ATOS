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
#define FENCE_DIRECTORY "./fence/"
/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

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
int SupervisionCheckGeofences(MONRType MONRdata, GeofenceType *geofences, char numberOfGeofences);
int loadGeofences(GeofenceType *geofences, int *numberOfGeofences);
int parseGeofencefile(char* geofenceFile, GeofenceType *geofences, int *numberOfGeofences);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervision_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    MONRType MONRMessage;

    int nGeof = 0;
    GeofenceType* geoPtrs;

    /* TODO: replace with permanent
    const int nPts = 4;

    GeofenceType perm1, perm2, forb1;

    perm1.numberOfPoints = nPts;
    perm2.numberOfPoints = nPts;
    forb1.numberOfPoints = nPts;

    perm1.isPermitted = 1;
    perm2.isPermitted = 1;
    forb1.isPermitted = 0;

    perm1.polygonPoints = (CartesianPosition*)malloc(nPts*sizeof(CartesianPosition));
    perm2.polygonPoints = (CartesianPosition*)malloc(nPts*sizeof(CartesianPosition));
    forb1.polygonPoints = (CartesianPosition*)malloc(nPts*sizeof(CartesianPosition));


    /* end */

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
            LogMessage(LOG_LEVEL_INFO, "Supervision exiting...");
            (void)iCommClose();
        }

        switch (command)
        {
        case COMM_INIT:
            // TODO: Read geofence file for each object and populate data structure
            loadGeofences(geoPtrs, nGeof);
            break;
        case COMM_MONR:
            UtilPopulateMONRStruct(busReceiveBuffer, &MONRMessage, 0);
            // TODO: react to output from SupervisionCheckGeofences
            SupervisionCheckGeofences(MONRMessage, geoPtrs, nGeof);
            break;
        case COMM_OBC_STATE:
            break;
        case COMM_INV:
            break;
        default:
            LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %d", command);
        }
    }
}

int loadGeofences(GeofenceType *geofences, int *numberOfGeofences){
        LogMessage(LOG_LEVEL_INFO,"Loading Geofences");

        int len;
          struct dirent *pDirent;
          DIR *pDir;

          pDir = opendir (FENCE_DIRECTORY);
          if (pDir == NULL) {
              LogMessage(LOG_LEVEL_INFO,"Cannot open directory");
              return 1;
          }

          while ((pDirent = readdir(pDir)) != NULL) {
               LogMessage(LOG_LEVEL_INFO,"[%s]\n", pDirent->d_name);

               //TODO: double check if .geofence file
               parseGeofencefile(pDirent->d_name, geofences, numberOfGeofences);
          }
          closedir (pDir);
          return 0;

}

int parseGeofencefile(char* geofenceFile, GeofenceType *geofences, int *numberOfGeofences){

    char pcFileNameBuffer[MAX_FILE_PATH] = "";
    strcat(pcFileNameBuffer, FENCE_DIRECTORY);
    strcat(pcFileNameBuffer, geofenceFile);

    FILE *fp;
    char str[MAX_FILE_PATH];

    fp = fopen(pcFileNameBuffer, "r");
        if ( fp != NULL )
        {
           char line [ 128 ]; /* or other suitable maximum line size */
           /*while ( fgets ( line, sizeof line, fp ) != NULL ) /* read a line */
           {
              fputs ( line, stdout ); /* write the line */
              LogMessage(LOG_LEVEL_INFO,"Looping....");
              strcpy(geofences[0].name, line);
              LogMessage(LOG_LEVEL_INFO, geofences[0].name);

           }
           fclose ( fp );
        }
        else
        {
           perror ( pcFileNameBuffer ); /* why didn't the file open? */
        }
    return 0;
}

int SupervisionCheckGeofences(MONRType MONRdata, GeofenceType *geofences, char numberOfGeofences)
{
    CartesianPosition monrPoint = {MONRdata.XPositionI32/1000.0, MONRdata.YPositionI32/1000.0, MONRdata.ZPositionI32/1000.0, 0.0};
    char isInPolygon = 0;
    int retval = 0;

    for (int i = 0; i < numberOfGeofences; i++)
    {
        isInPolygon = UtilIsPointInPolygon(monrPoint, geofences[i].polygonPoints, geofences[i].numberOfPoints);
        if ( (geofences[i].isPermitted && isInPolygon)
             || (!geofences[i].isPermitted && !isInPolygon) )
        {
            // Inside the polygon if it is permitted, alt. outside the polygon if it is forbidden: all is fine
        }
        else
        {
            if (geofences[i].isPermitted)
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is outside the permitted area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);
            else
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is inside the forbidden area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);

            retval = -1;
        }
    }

    return retval;
}
