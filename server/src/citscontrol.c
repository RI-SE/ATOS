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
#include "CAM.h"
#include "DENM.h"


#include "MQTTClient.h"
#include "citscontrol.h"
#include "util.h"



#define CITS_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define CITS_CONTROL_BUFFER_SIZE_20 20
#define CITS_CONTROL_BUFFER_SIZE_52 52
#define CITS_CONTROL_TASK_PERIOD_MS 1

#define DEFAULT_MQTT_ADDRESS     "tcp://localhost:1883"
#define ERICSSON_MQTT_ADDRESS     "tcp://10.130.100.18:1883"
#define DEFAULT_MQTT_CLIENTID    "ExampleClientPub"
#define DEFAULT_MQTT_TOPIC       "CLIENT/CAM/CS01/1/AZ12"
#define DEFAULT_MQTT_PAYLOAD     "Hello World!"
#define DEFAULT_MQTT_QOS         1
#define DEFAULT_MQTT_TIMEOUT     10000L

#define MODULE_NAME "CitsControl"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
I32 generateCamMessage(MONRType *MONRData, CAM_t* lastCam, I16* lastSpeed);
I32 generateDenmMessage(MONRType *MONRData, DENM_t* lastCam, I16* lastSpeed);

I32 sendCam(CAM_t* lastCam);
void init_mqtt(char* ip_addr, char * clientid);
int connect_mqtt();
int publish_mqtt(char *payload, int payload_len, char *topic);
void delivered_mqtt(void *context, MQTTClient_deliveryToken dt);
int msgarrvd_mqtt(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void connlost_mqtt(void *context, char *cause);



/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

enum CITS_STATE {
    INIT,
    DISCONNECTED,
    CONNECTED,
    SENDING
};

static int state = INIT;
static volatile int pending_state = INIT;

static MQTTClient client;
static MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
static MQTTClient_message pubmsg = MQTTClient_message_initializer;
static volatile MQTTClient_deliveryToken deliveredtoken = 0;
static MQTTClient_deliveryToken sendtoken = 0;
/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    int camTimeCycle = 0;
    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;
    int mqtt_response_code = 0;
    MONRType MONRMessage;
    CAMmessage lastCam;
    TimeType time;

    I16 lastSpeed = 0;
    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "CITS running with PID: %i", getpid());

    UtilGetMillisecond(&time);

    (void)init_mqtt(ERICSSON_MQTT_ADDRESS,DEFAULT_MQTT_CLIENTID);

    MQTTClient_setCallbacks(client, NULL, connlost_mqtt, msgarrvd_mqtt, delivered_mqtt);

    lastCam.header.generationTime = time.MillisecondU16;

    LogMessage(LOG_LEVEL_INFO,"Starting cits control...\n");
    lastCam.referencePosition.latitude.degrees = 0;
    lastCam.referencePosition.longitude.degrees = 0;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());


    int monrCounter = 0;
    while(!iExit)
    {
        // Handle states specific things
        state = pending_state;

        //LogMessage(LOG_LEVEL_DEBUG,"CITS state %d",state);
        switch (state) {
        case INIT:

            if (!connect_mqtt()){
                LogMessage(LOG_LEVEL_INFO,"Connected!");
                //MQTTClient_subscribe(client,DEFAULT_MQTT_TOPIC,DEFAULT_MQTT_QOS);
                pending_state = CONNECTED;
                LogMessage(LOG_LEVEL_DEBUG,"CITS state change from %d to %d",state,pending_state);
            }
            break;
        case CONNECTED:

            if ((mqtt_response_code = publish_mqtt(DEFAULT_MQTT_PAYLOAD,strlen(DEFAULT_MQTT_PAYLOAD),DEFAULT_MQTT_TOPIC))) {
                LogMessage(LOG_LEVEL_ERROR,"Could not publish message, error code %d", mqtt_response_code);
            }
            else {
                pending_state = SENDING;
            }
            break;
        case SENDING:
            if (sendtoken == deliveredtoken) {
                pending_state = CONNECTED;
                LogMessage(LOG_LEVEL_DEBUG,"CITS state change from %d to %d",state,pending_state);
            }
            break;
        }

        // Handle MQ messages

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
                       generateCamMessage(&MONRMessage, &lastCam, &lastSpeed);
                       sendCam(&lastCam);
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

void init_mqtt(char* ip_addr, char * clientid){


    MQTTClient_create(&client, ip_addr, clientid,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
}

int connect_mqtt(){
    int return_code;
    if ((return_code = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        LogMessage(LOG_LEVEL_ERROR,"Failed to connect, return code %d\n", return_code);
        return 1;
    }
    return 0;
}

int publish_mqtt(char *payload, int payload_len, char *topic){

    pubmsg.payload = payload;
    pubmsg.payloadlen = payload_len;
    pubmsg.qos = DEFAULT_MQTT_QOS;
    pubmsg.retained = 0;
    deliveredtoken = 0;

    int retval = MQTTClient_publishMessage(client, topic, &pubmsg, &sendtoken);
    if (!retval) {
        LogMessage(LOG_LEVEL_INFO,"Waiting for publication of %s\n on topic %s for client with ClitentID %s",payload, topic, DEFAULT_MQTT_CLIENTID );
    }
    return retval;
}

int msgarrvd_mqtt(void *context, char *topicName, int topicLen, MQTTClient_message *message){
    (void)context;
    (void)topicLen;
    int i;
    char* payloadptr;
    LogMessage(LOG_LEVEL_DEBUG,"Message arrived! Length=%d",message->payloadlen);
    //if (message->payloadlen == 0) return 1;
    //else if (topicLen == 0) return 2;
    if(message->payloadlen > 0) LogMessage(LOG_LEVEL_DEBUG,"\n\tTopic: %s\n\tmessage: %s",topicName,message->payload);
    //printf("Message arrived\n");
    //printf("     topic: %s\n", topicName);
    //printf("   message: ");
    //payloadptr = message->payload;
    /*
    for(i=0; i<message->payloadlen; i++)
    {
        putchar(*payloadptr++);
    }
    putchar('\n');
    */
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 0;
}

void delivered_mqtt(void *context, MQTTClient_deliveryToken dt)
{
    LogMessage(LOG_LEVEL_DEBUG,"Message delivered with token: %d .", dt);
    deliveredtoken = dt;
}

void connlost_mqtt(void *context, char *cause){
    LogMessage(LOG_LEVEL_DEBUG,"Connection Lost.\n Cause: %s",cause);
    printf("Connection lost \n");
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
I32 generateCamMessage(MONRType *MONRData, CAM_t* lastCam, I16* lastSpeed){

    TimeType time;
    CAM_t tempCam;

    tempCam.header.protocolVersion = 1;
    tempCam.header.messageID = 2;
    tempCam.header.stationID = 1000;

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

    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16));

    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = 0; //FORWARD
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = 10; //TEMP WIDTH
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = 10; //TEMP LENGTH


   //TODO: CRASHES HERE FOR SOME REASON
/*
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = MONRData->LateralAccI16;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = MONRData->LongitudinalAccI16;

        printf("Got here2\n");
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = 0; //HARDCODED CURVATURE
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = 7;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = 0;
*/


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
            //sendCam(lastCam);

        }

        if(tempCam.cam.generationDeltaTime - lastCam->cam.generationDeltaTime >= T_THRESHOLD){
            printf("\"Sending\" CAM because of time..\n");
            lastSpeed = (U16)((double)sqrt(MONRData->LateralAccI16*MONRData->LateralAccI16) + (double)(MONRData->LateralAccI16*MONRData->LateralAccI16));
            *lastCam = tempCam;
            //sendCam(lastCam);
        }
    }
}



/*!
 * \brief GenerateDENMMessage generates a DENM message to send on MQTT
 * \param MONRData MONR data struct
 * \param lastDENM struct to fill with DENM data if DENM should be sent, used as reference to calculate new DENM.
 * \param lastSpeed variable keeping track of last speed recorded.
 */
I32 generateDenemMessage(MONRType *MONRData, DENM_t* lastDenm, I16* lastSpeed){
    TimeType time;
    DENM_t tempDENM;

    tempDENM.header.protocolVersion = 1;
    tempDENM.header.messageID = 1;
    tempDENM.header.stationID = 1234;


    tempDENM.denm.management.actionID.originatingStationID = 12345;
    tempDENM.denm.management.actionID.sequenceNumber = 0;

    UtilGetMillisecond(&time);
   // tempDENM.denm.management.detectionTime = time.MillisecondU16;
   // tempDENM.denm.management.referenceTime = time.MillisecondU16;

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

    tempDENM.denm.management.eventPosition.latitude = latitude;
    tempDENM.denm.management.eventPosition.longitude = longitude;

    tempDENM.denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = 7;
    tempDENM.denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 10;

    tempDENM.denm.management.eventPosition.altitude.altitudeValue = 0;
    tempDENM.denm.management.eventPosition.altitude.altitudeConfidence = 0;

    /*
    tempDENM.denm.management.relevanceDistance = 3;
    tempDENM.denm.management.relevanceTrafficDirection = 1;
    tempDENM.denm.management.validityDuration = 0;
    tempDENM.denm.management.transmissionInterval = 100;
    tempDENM.denm.management.stationType = 8; //HEAVY TRUCK. 5 = passenger car, 1 = Pedestrian
*/
    tempDENM.denm.situation->informationQuality = 7;
    tempDENM.denm.situation->eventType.causeCode = 99;
    tempDENM.denm.situation->eventType.subCauseCode = 1; //Emergency break engaged

    tempDENM.denm.location->eventSpeed->speedValue = 0; //CHECK THIS
    tempDENM.denm.location->eventSpeed->speedConfidence = 0; //unavaliabe

    *lastDenm = tempDENM;

}
/*!
 * \brief SendCam publishes a cam message on MQTT with hardcoded topic.
 * \param lastCam cam message struct
 * \return 1 if message sent succesfully
 */
I32 sendCam(CAM_t* lastCam){

    CAM_t * cam;
    cam = calloc(1, sizeof (*cam));
    assert(cam);
    xer_fprint(stdout, &asn_DEF_CAM, cam);


    publish_mqtt(cam, sizeof(lastCam), "CLIENT/CAM/CS01/1/AZ12B");
    return 1;
}

