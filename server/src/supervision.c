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

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervision_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    MONRType MONRMessage;


    /* TODO: replace with permanent */
    const int nPts = 4;
    const int nGeof = 3;

    GeofenceType geoPtrs[nGeof];
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

    perm1.polygonPoints[0].xCoord_m = -100;
    perm1.polygonPoints[1].xCoord_m = 80;
    perm1.polygonPoints[2].xCoord_m = 80;
    perm1.polygonPoints[3].xCoord_m = -100;

    perm1.polygonPoints[0].yCoord_m = -100;
    perm1.polygonPoints[1].yCoord_m = -100;
    perm1.polygonPoints[2].yCoord_m = 60;
    perm1.polygonPoints[3].yCoord_m = 60;

    perm2.polygonPoints[0].xCoord_m = -100;
    perm2.polygonPoints[1].xCoord_m = 100;
    perm2.polygonPoints[2].xCoord_m = 100;
    perm2.polygonPoints[3].xCoord_m = -100;

    perm2.polygonPoints[0].yCoord_m = -25;
    perm2.polygonPoints[1].yCoord_m = -25;
    perm2.polygonPoints[2].yCoord_m = 100;
    perm2.polygonPoints[3].yCoord_m = 100;

    forb1.polygonPoints[0].xCoord_m = 40;
    forb1.polygonPoints[1].xCoord_m = 50;
    forb1.polygonPoints[2].xCoord_m = 50;
    forb1.polygonPoints[3].xCoord_m = 40;

    forb1.polygonPoints[0].yCoord_m = 50;
    forb1.polygonPoints[1].yCoord_m = 50;
    forb1.polygonPoints[2].yCoord_m = 60;
    forb1.polygonPoints[3].yCoord_m = 60;

    geoPtrs[0] = perm1;
    geoPtrs[1] = perm2;
    geoPtrs[2] = forb1;
    /* end */

    perm1.name[0] = '1';
    perm2.name[0] = '2';
    forb1.name[0] = 'f';

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
            break;
        case COMM_MONI:
            // Ignore old style MONR data
            break;
        case COMM_MONR:
            UtilPopulateMONRStruct(busReceiveBuffer, &MONRMessage, 0);
            // TODO: react to output from SupervisionCheckGeofences
            SupervisionCheckGeofences(MONRMessage, geoPtrs, nGeof);
            break;
        case COMM_OBC_STATE:
            break;
        case COMM_LOG:
            break;
        case COMM_INV:
            break;
        default:
            LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
        }
    }

    // TODO: Temporary
    free(perm1.polygonPoints);
    free(perm2.polygonPoints);
    free(forb1.polygonPoints);
}

int SupervisionCheckGeofences(MONRType MONRdata, GeofenceType *geofences, char numberOfGeofences)
{
    const CartesianPosition monrPoint = {MONRdata.XPositionI32/1000.0, MONRdata.YPositionI32/1000.0, MONRdata.ZPositionI32/1000.0, 0.0};
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
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is outside a permitted area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);
            else
                LogMessage(LOG_LEVEL_WARNING,"Object with MONR transmitter ID %u is inside a forbidden area %s", MONRdata.Header.TransmitterIdU8, geofences[i].name);

            retval = -1;
        }
    }

    return retval;
}
