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
I32 GenerateCamMessage(MONRType *MONRData, CAMmessage* lastCam);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;
    MONRType MONRMessage;
    CAMmessage lastCam;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());

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


            GenerateCamMessage(&MONRMessage, &lastCam);

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
#define H_THRESHOLD 10
#define S_THRESHOLD 10
#define T_THRESHOLD 10
#define D_THRESHOLD 10
#define CHECK_PERIOD 10

I32 GenerateCamMessage(MONRType *MONRData, CAMmessage* lastCam){

    TimeType time;
    CAMmessage tempCam;

    UtilGetMillisecond(&time);
    tempCam.header.generationTime = time.MillisecondU16;
    tempCam.referencePosition.heading = MONRData->HeadingU16;

    if(MONRData != NULL ){
        double distanceDelta = UtilGetDistance(tempCam.referencePosition.latitude.degrees, tempCam.referencePosition.longitude.degrees, lastCam->referencePosition.latitude.degrees, lastCam->referencePosition.longitude.degrees);
        U16 headingDelta = tempCam.referencePosition.heading - lastCam->referencePosition.heading
        U16 speedDelta = sqrt(MONRData->LateralAccI16*MONRData->LateralAccI16) + (MONRData->LateralAccI16*MONRData->LateralAccI16);

        if( distanceDelta >= D_THRESHOLD || headingDelta >= H_THRESHOLD || speedDelta >= S_THRESHOLD){
              lastCam->referencePosition.heading = tempCam.referencePosition.heading;
        }
        else{
            tempCam.referencePosition.latitude = lastCam->referencePosition.latitude;
            tempCam.referencePosition.longitude = lastCam->referencePosition.longitude;
            tempCam.referencePosition.heading = lastCam->referencePosition.heading;
        }
        if(tempCam.header.generationTime - lastCam->header.generationTime >= T_THRESHOLD){
            *lastCam = tempCam;
        }
    }

    /* PSEUDOCODE FOR CAM
       while true do
           time = System.getTime() DONE
           heading = calcHeading(pHist, p) DONE
           lastPos = lastPosition(pHist)
           lastHist = pHist \ lastPos
           lastHead = calcHeading(lastHist, lastPos)
           speed = calcSpeed(pHist, p)
           if p , null then
               lastSpeed = calcSpeed(lastHist, lastPos)
               if distance(p, lastCam.pos) ≥ D_THRESHOLD or |heading - lastCam.heading| ≥ H_THRESHOLD or |speed - lastCam.speed| ≥ S_THRESHOLD then
                   cam = newCam(time, p, heading, speed)
                   sendCam(cam)
                   lastCam = cam
               pHist = pHist UNION p
           else
               p = lastPos
               heading = lastHead
           if time - lastCam.time ≥ T_THRESHOLD then
               cam = newCam(time, p, heading, speed)
               sendCam(cam)
               lastCam = cam
           System.wait(CHECK_PERIOD)
           */
}
