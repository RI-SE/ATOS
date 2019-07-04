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
I32 generateCAMMessage(MONRType *MONRData, CAM_t* lastCam);
I32 generateDENMMessage(MONRType *MONRData, DENM_t* denm);

I32 sendCAM(CAM_t* lastCam);
I32 sendDENM(DENM_t* lastDenm);



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
    CAM_t lastCam;
    DENM_t lastDenm;

    TimeType time;

    I16 lastSpeed = 0;
    LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
    LogMessage(LOG_LEVEL_INFO, "CITS running with PID: %i", getpid());

    UtilGetMillisecond(&time);

    (void)init_mqtt(ERICSSON_MQTT_ADDRESS,DEFAULT_MQTT_CLIENTID);

    MQTTClient_setCallbacks(client, NULL, connlost_mqtt, msgarrvd_mqtt, delivered_mqtt);

    lastCam.cam.generationDeltaTime = time.MillisecondU16;

    LogMessage(LOG_LEVEL_INFO,"Starting cits control...\n");
    lastCam.cam.camParameters.basicContainer.referencePosition.latitude = 0;
    lastCam.cam.camParameters.basicContainer.referencePosition.longitude = 0;

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
                       printf("TRIGGERED\n");

                           I16 speedDelta = abs((sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16))) - (lastSpeed));

                           printf("Speed delta %d \n", speedDelta);

                           if(speedDelta >= 0){
                               generateCAMMessage(&MONRMessage, &lastCam);
                               generateDENMMessage(&MONRMessage, &lastDenm);
                               sendCAM(&lastCam);
                               sendDENM(&lastDenm);
                           }
                       lastSpeed = sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16));
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
I32 generateCAMMessage(MONRType *MONRData, CAM_t* cam){

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

    /* Calculate the geodetic forward azimuth in the direction from known origo
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
        *cam = tempCam;
    }
}



/*!
 * \brief GenerateDENMMessage generates a DENM message to send on MQTT
 * \param MONRData MONR data struct
 * \param lastDENM struct to fill with DENM data if DENM should be sent, used as reference to calculate new DENM.
 * \param lastSpeed variable keeping track of last speed recorded.
 */
I32 generateDENMMessage(MONRType *MONRData, DENM_t* denm){
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


    /* CRASHES CITS
    tempDENM.denm.management.relevanceDistance = 3;
    tempDENM.denm.management.relevanceTrafficDirection = 1;
    tempDENM.denm.management.validityDuration = 0;
    tempDENM.denm.management.transmissionInterval = 100;
    tempDENM.denm.management.stationType = 8; //HEAVY TRUCK. 5 = passenger car, 1 = Pedestrian


    tempDENM.denm.situation->informationQuality = 7;
    tempDENM.denm.situation->eventType.causeCode = 99;

    tempDENM.denm.situation->eventType.subCauseCode = 1; //Emergency break engaged


    tempDENM.denm.location->eventSpeed->speedValue = 0; //CHECK THIS
    tempDENM.denm.location->eventSpeed->speedConfidence = 0; //unavaliabe

    */
    if(MONRData != NULL ){
        *denm = tempDENM;

    }

}
/*!
 * \brief SendCam publishes a cam message on MQTT with hardcoded topic.
 * \param lastCam cam message struct
 * \return 1 if message sent succesfully
 */
I32 sendCAM(CAM_t* cam){

   // CAM_t * cam;
   // cam = calloc(1, sizeof (*cam));
   // assert(cam);
   // xer_fprint(stdout, &asn_DEF_CAM, cam);

    printf("SENDING CAM\n");

    publish_mqtt(cam, sizeof (CAM_t), "CLIENT/CAM/CS01/1/AZ12B");
    return 1;
}

/*!
 * \brief SendCam publishes a cam message on MQTT with hardcoded topic.
 * \param lastCam cam message struct
 * \return 1 if message sent succesfully
 */
I32 sendDENM(DENM_t* denm){

   // CAM_t * cam;
   // cam = calloc(1, sizeof (*cam));
   // assert(cam);
   // xer_fprint(stdout, &asn_DEF_CAM, cam);

    printf("SENDING DENM\n");

    publish_mqtt(denm, sizeof (DENM_t), "CLIENT/DENM/CS01/1/AZ12B");
    return 1;
}

