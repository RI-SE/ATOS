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
#include "generatedfiles/CAM.h"

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
I32 GenerateCamMessage(MONRType *MONRData, CAM_t* lastCam, I16* lastSpeed);
I32 SendCam(CAM_t* lastCam);


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

/*!
 * \brief GenerateCamMessage generates a cam message to send on MQTT
 * \param MONRData MONR data struct
 * \param lastCam struct to fill with cam data if cam should be sent, used as reference to calculate new cam.
 * \param lastSpeed variable keeping track of last speed recorded.
 */
I32 GenerateCamMessage(MONRType *MONRData, CAM_t* lastCam, I16* lastSpeed){

    TimeType time;
    CAM_t tempCam;

    tempCam.header.protocolVersion = 1;
    tempCam.header.messageID = 2;

    UtilGetMillisecond(&time);
    tempCam.cam.generationDeltaTime = time.MillisecondU16;

    //LOG LAT from XY
    double x = MONRData->XPositionI32;
    double y = MONRData->YPositionI32;
    double z = MONRData->ZPositionI32;
    double latitude, longitude, height;

    double distance=0;
    double azimuth1 = 0;
    double azimuth2 =0;
    int fail;

    /* Calculate the geodetic forward azimuth in the direction from origo to point we want to know,
     * A problem right now is that I belive that the GUC and virtualObject needs to have the same origin
   * */

    azimuth1 = UtilDegToRad(90)-atan2(y/1.00,x/1.00);

    // calculate the norm value
    distance = sqrt(pow(x/1.00,2)+pow(y/1.00,2));

    // TODO: Get From RVSSgetParameter
    double origoLong = 12.77011670;
    double origoLat = 57.77315060;

    fail = UtilVincentyDirect(origoLat,origoLong,azimuth1,distance ,&latitude,&longitude,&azimuth2);

    printf("latitude %f \n", latitude);
    printf("longitude %f \n", longitude);

    tempCam.cam.camParameters.basicContainer.referencePosition.latitude = latitude;
    tempCam.cam.camParameters.basicContainer.referencePosition.longitude = longitude;

    tempCam.cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = MONRData->HeadingU16;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16)));

    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = 0; //FORWARD
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = 10; //TEMP WIDTH
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength = 10; //TEMP LENGTH


    if(MONRData != NULL ){
        double distanceDelta = UtilGetDistance(tempCam.cam.camParameters.basicContainer.referencePosition.latitude, tempCam.cam.camParameters.basicContainer.referencePosition.longitude, lastCam->cam.camParameters.basicContainer.referencePosition.latitude, lastCam->cam.camParameters.basicContainer.referencePosition.longitude);
        double headingDelta = tempCam.cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation - lastCam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
        I16 speedDelta = (sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16))) - (*lastSpeed);

        printf("Speed delta %d \n", speedDelta);
        printf("Distance delta %f \n", distanceDelta);
        printf("heading delta %f \n", headingDelta);
        printf("Time delta %d \n", tempCam.cam.generationDeltaTime - lastCam->cam.generationDeltaTime);


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
            //SendCam(&lastCam);

        }

        if(tempCam.cam.generationDeltaTime - lastCam->cam.generationDeltaTime >= T_THRESHOLD){
            printf("\"Sending\" CAM because of time..\n");
            lastSpeed = (U16)((double)sqrt(MONRData->LateralAccI16*MONRData->LateralAccI16) + (double)(MONRData->LateralAccI16*MONRData->LateralAccI16));
            *lastCam = tempCam;
           // SendCam(&lastCam);
        }
    }
}

/*!
 * \brief SendCam publishes a cam message on MQTT with hardcoded topic.
 * \param lastCam cam message struct
 * \return 1 if message sent succesfully
 */
I32 SendCam(CAM_t* lastCam){

       return 1;

}

