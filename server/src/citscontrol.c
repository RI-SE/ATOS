/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : citscontrol.c
  -- Author      : Sebastian Loh Lindholm
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

#include "citscontrol.h"

#define CITS_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define CITS_CONTROL_BUFFER_SIZE_20 20
#define CITS_CONTROL_BUFFER_SIZE_52 52
#define CITS_CONTROL_TASK_PERIOD_MS 1

#define MODULE_NAME "CitsControl"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
I32 GenerateCamMessage(MONRType *MONRData, CAMmessage* lastCam, I16* lastSpeed);
I32 SendCam(CAMmessage* lastCam);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    int camTimeCycle = 0;
    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;
    MONRType MONRMessage;
    CAMmessage lastCam;
    TimeType time;

    I16 lastSpeed = 0;

    UtilGetMillisecond(&time);


    lastCam.header.generationTime = time.MillisecondU16;

    lastCam.referencePosition.latitude.degrees = 0;
    lastCam.referencePosition.longitude.degrees = 0;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());


    int monrCounter = 0;
    while(!iExit)
    {
        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));
        (void)iCommRecv(&command,busReceiveBuffer, sizeof(busReceiveBuffer), NULL);
        if (command == COMM_ABORT)
        {

        }

        if(command == COMM_EXIT)
        {
            iExit = 1;
            printf("citscontrol exiting.\n");
            (void)iCommClose();
        }

        switch (command)
        {
        case COMM_INIT:

            break;
        case COMM_MONI:
            // Ignore old style MONR data
            break;
        case COMM_MONR:
           //TODO: CREATE CAM

            UtilPopulateMONRStruct(busReceiveBuffer, sizeof(busReceiveBuffer), &MONRMessage, 0);

            if(camTimeCycle == 100)
            {
                GenerateCamMessage(&MONRMessage, &lastCam, &lastSpeed);
                SendCam(&lastCam);
                camTimeCycle = 0;
            }
            camTimeCycle++;

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

//TODO MOVE THESE DEFINITIONS TO CORRECT PLACE
#define H_THRESHOLD 1
#define S_THRESHOLD 1
#define T_THRESHOLD 1
#define D_THRESHOLD 1
#define CHECK_PERIOD 1

I32 GenerateCamMessage(MONRType *MONRData, CAMmessage* lastCam, I16* lastSpeed){

    TimeType time;
    CAMmessage tempCam;


    tempCam.header.version = 0;
    tempCam.header.messageID = 1;

    UtilGetMillisecond(&time);
    tempCam.header.generationTime = time.MillisecondU16;
    tempCam.referencePosition.heading = MONRData->HeadingU16;

    //LOG LAT from XY
    double x = MONRData->XPositionI32;
    double y = MONRData->YPositionI32;
    double z = MONRData->ZPositionI32;
    double latitude, longitude, height;

    height = 0;
    xyzToLlh(x, y, z, &latitude, &longitude, &height);

    printf("latitude %f \n", latitude);
    printf("longitude %f \n", longitude);

    tempCam.referencePosition.latitude.degrees = latitude;
    tempCam.referencePosition.longitude.degrees = longitude;



    if(MONRData != NULL ){
        double distanceDelta = UtilGetDistance(tempCam.referencePosition.latitude.degrees, tempCam.referencePosition.longitude.degrees, lastCam->referencePosition.latitude.degrees, lastCam->referencePosition.longitude.degrees);
        double headingDelta = tempCam.referencePosition.heading - lastCam->referencePosition.heading;
        I16 speedDelta = (sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16))) - (*lastSpeed);

        printf("Speed delta %d \n", speedDelta);
        printf("Distance delta %f \n", distanceDelta);
        printf("heading delta %f \n", headingDelta);
        printf("Time delta %d \n", tempCam.header.generationTime - lastCam->header.generationTime);


        if( distanceDelta >= D_THRESHOLD || headingDelta >= H_THRESHOLD || speedDelta >= S_THRESHOLD){
            printf("\"Sending\" CAM \n");

            /*
            printf("Generation time: %d", tempCam.header.generationTime);
            printf("Heading: %d", tempCam.referencePosition.heading);
            printf("Latitude: %d", tempCam.referencePosition.latitude.degrees);
            printf("Longitude time: %d", tempCam.referencePosition.longitude.degrees);
            printf("Elevation time: %d", tempCam.referencePosition.elevation);
            */

            *lastSpeed =  (U16)((double)sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (double)(MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16)));
            *lastCam = tempCam;

        }

        if(tempCam.header.generationTime - lastCam->header.generationTime >= T_THRESHOLD){
            printf("\"Sending\" CAM because of time..\n");
            lastSpeed = (U16)((double)sqrt(MONRData->LateralAccI16*MONRData->LateralAccI16) + (double)(MONRData->LateralAccI16*MONRData->LateralAccI16));
            *lastCam = tempCam;
        }
    }
}

I32 SendCam(CAMmessage* lastCam){
       size_t i = 0;

       unsigned char *dst;

       memcpy(&dst[i], &lastCam->header.version, sizeof lastCam->header.version);
       i += sizeof &lastCam->header.version;

       memcpy(&dst[i], &lastCam->header.messageID, sizeof lastCam->header.messageID);
       i += sizeof &lastCam->header.messageID;

       memcpy(&dst[i], &lastCam->header.generationTime, sizeof lastCam->header.generationTime);
       i += sizeof &lastCam->header.generationTime;

       return i;

}

