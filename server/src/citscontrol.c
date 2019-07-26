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
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string.h>


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
#include "iso22133.h"




#define H_THRESHOLD 1 //HEADING THRESHOLD
#define S_THRESHOLD 0 //SPEED THRESHOLD
#define D_THRESHOLD 1 //DISTANCE THRESHOLD
#define CHECK_PERIOD 100

#define CITS_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define CITS_CONTROL_BUFFER_SIZE_20 20
#define CITS_CONTROL_BUFFER_SIZE_52 52
#define CITS_CONTROL_TASK_PERIOD_MS 1

#define DEFAULT_MQTT_ADDRESS     "tcp://localhost:1883"
#define ERICSSON_MQTT_ADDRESS     "tcp://10.130.100.18:1883"
#define DEFAULT_MQTT_CLIENTID    "ExampleClientPub1"
#define DEFAULT_MQTT_TOPIC       "CLIENT/CAM/CS01/1/AZ12B"
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

bool validate_constraints(asn_TYPE_descriptor_t *type_descriptor, const void *struct_ptr) ;

void *encode_and_decode_object(asn_TYPE_descriptor_t *type_descriptor, void *struct_ptr) ;
int parseActionConfiguration(ACCMData config, uint16_t* actionIDs, unsigned int* nConfiguredActions);
bool isActionValid(EXACData exac, uint16_t* actionIDs, unsigned int nConfiguredActions);

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
    MonitorDataType mqMONRdata;
    MONRType MONRMessage;
    MONRType LastMONRMessage;
    ACCMData actionConfig;
    EXACData actionData;
    uint16_t* storedActionIDs = NULL;
    unsigned int numberOfStoredActionIDs = 0;

    asn_enc_rval_t ec;

    CAM_t* lastCam;
    lastCam = calloc(1, sizeof(CAM_t));
    if(!lastCam){
         exit(1);
    }

    DENM_t* lastDenm;
    lastDenm = calloc(1, sizeof(DENM_t));
    if(!lastDenm) {
           exit(1);
    }

    TimeType time;

    I16 lastSpeed = 0;
    LogInit(MODULE_NAME,logLevel);
    LogMessage(LOG_LEVEL_INFO, "C-ITS control running with PID: %i", getpid());

    UtilGetMillisecond(&time);

    (void)init_mqtt(ERICSSON_MQTT_ADDRESS,DEFAULT_MQTT_CLIENTID);

    MQTTClient_setCallbacks(client, NULL, connlost_mqtt, msgarrvd_mqtt, delivered_mqtt);

    lastCam->cam.generationDeltaTime = time.MillisecondU16;

    LogMessage(LOG_LEVEL_INFO,"Starting cits control...\n");
    lastCam->cam.camParameters.basicContainer.referencePosition.latitude = 0;
    lastCam->cam.camParameters.basicContainer.referencePosition.longitude = 0;

    if (iCommInit())
        util_error("Unable to connect to message bus");

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
                MQTTClient_subscribe(client,DEFAULT_MQTT_TOPIC,DEFAULT_MQTT_QOS);
                pending_state = CONNECTED;
                LogMessage(LOG_LEVEL_DEBUG,"CITS state change from %d to %d",state,pending_state);
            }
            break;
        case CONNECTED:

          /*  if ((mqtt_response_code = publish_mqtt(DEFAULT_MQTT_PAYLOAD,strlen(DEFAULT_MQTT_PAYLOAD),DEFAULT_MQTT_TOPIC))) {
                LogMessage(LOG_LEVEL_ERROR,"Could not publish message, error code %d", mqtt_response_code);
            }
            else {
                pending_state = SENDING;
            }

            break;
          */
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
            LogMessage(LOG_LEVEL_DEBUG, "Ignored old style MONR data");
            break;
        case COMM_MONR:

            UtilPopulateMonitorDataStruct(busReceiveBuffer, sizeof(busReceiveBuffer),&mqMONRdata,0);
            MONRMessage = mqMONRdata.MONR;

            if(camTimeCycle == CHECK_PERIOD)
            {
                //I16 distanceDelta = UtilCoordinateDistance(LastMONRMessage.XPositionI32, LastMONRMessage.YPositionI32, MONRMessage.XPositionI32, MONRMessage.YPositionI32);
                //I16 headingDelta = abs(LastMONRMessage.HeadingU16 - MONRMessage.HeadingU16);
                //I16 speedDelta = abs((sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16))) - (lastSpeed));


                //if(speedDelta >= S_THRESHOLD || distanceDelta >= D_THRESHOLD || headingDelta >= H_THRESHOLD){
                //    generateCAMMessage(&MONRMessage, lastCam);
                //    generateDENMMessage(&MONRMessage, lastDenm);
                //    sendCAM(lastCam);
                //    sendDENM(lastDenm);
                //}
                //lastSpeed = sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16));
                //LastMONRMessage = MONRMessage;
                //camTimeCycle = 0;
            }
            camTimeCycle++;

            break;
        case COMM_ACCM:
            LogMessage(LOG_LEVEL_INFO,"Received action configuration");
            UtilPopulateACCMDataStructFromMQ(busReceiveBuffer, sizeof(busReceiveBuffer), &actionConfig);
            parseActionConfiguration(actionConfig, storedActionIDs, &numberOfStoredActionIDs);
            break;
        case COMM_EXAC:
            LogMessage(LOG_LEVEL_INFO, "Received EXAC");
            UtilPopulateEXACDataStructFromMQ(busReceiveBuffer, sizeof(busReceiveBuffer), &actionData);
            if (isActionValid(actionData, storedActionIDs, numberOfStoredActionIDs)
                    && state != DISCONNECTED && state != INIT)
            {
                LogMessage(LOG_LEVEL_INFO,"Received action request: sending DENM message");
                // TODO: Start timer
                // TODO: On timer, send DENM
            }
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
    //if(message->payloadlen > 0) {
    //    LogMessage(LOG_LEVEL_DEBUG,"\n\tTopic: %s\n\tmessage: %s",topicName,message->payload);
    //}
    /*

    if(message->payloadlen > 0) LogMessage(LOG_LEVEL_DEBUG,"\n\tTopic: %s\n\tmessage: %s",topicName,message->payload);
    printf("Message arrived\n");
    printf("topic: %s\n", topicName);
    printf("message: ");
    payloadptr = message->payload;

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



static int write_out(const void *buffer, size_t size, void *app_key){
    FILE *out_fp = app_key;
    size_t wrote = fwrite(buffer, 1, size, out_fp);
    return (wrote == size) ? 0 : -1;
}



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

    tempCam.cam.camParameters.basicContainer.referencePosition.latitude = latitude;
    tempCam.cam.camParameters.basicContainer.referencePosition.longitude = longitude;

    /*
    printf("calc latitude %f \n",  latitude);
    printf("calc longitude %f \n",  longitude);

    printf("CAM latitude %f \n",  tempCam.cam.camParameters.basicContainer.referencePosition.latitude);
    printf("CAM longitude %f \n",  tempCam.cam.camParameters.basicContainer.referencePosition.longitude);
    */

    tempCam.cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = MONRData->HeadingU16;

    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = sqrt((MONRData->LateralSpeedI16*MONRData->LateralSpeedI16) + (MONRData->LongitudinalSpeedI16*MONRData->LongitudinalSpeedI16));

    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_forward; //FORWARD
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = 10; //TEMP WIDTH
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = 10; //TEMP LENGTH


    //tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = MONRData->LateralAccI16;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = MONRData->LongitudinalAccI16;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = 0;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = 7;
    tempCam.cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = 0;


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
    tempDENM.header.stationID = 1000;


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

    tempDENM.denm.management.eventPosition.latitude = latitude;
    tempDENM.denm.management.eventPosition.longitude = longitude;

    /*
    printf("DENM latitude %f \n", tempDENM.denm.management.eventPosition.latitude);
    printf("DENM longitude %f \n", tempDENM.denm.management.eventPosition.longitude);
    */

    tempDENM.denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = 7;
    tempDENM.denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 10;

    tempDENM.denm.management.eventPosition.altitude.altitudeValue = 0;
    tempDENM.denm.management.eventPosition.altitude.altitudeConfidence = 0;

    //tempDENM.denm.management.relevanceDistance = 3;
    //tempDENM.denm.management.relevanceTrafficDirection = 1;
    //tempDENM.denm.management.validityDuration = 0;
    //tempDENM.denm.management.transmissionInterval = 100;
    //tempDENM.denm.management.stationType = StationType_heavyTruck; //HEAVY TRUCK. 5 = passenger car, 1 = Pedestrian
    //tempDENM.denm.situation->informationQuality = InformationQuality_highest;
    //tempDENM.denm.situation->eventType.causeCode = CauseCodeType_dangerousSituation;

    //tempDENM.denm.situation->eventType.subCauseCode =  1; //Emergency break engaged


    //tempDENM.denm.location->eventSpeed->speedValue = 0; //CHECK THIS
    //tempDENM.denm.location->eventSpeed->speedConfidence = 0; //unavaliabe


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
    printf("SENDING CAM\n");

    FILE *fp = fopen("tmp", "wb");
    //asn_enc_rval_t ec = der_encode(&asn_DEF_CAM, cam, write_out, fp);
    fclose(fp);

    publish_mqtt((char*)cam, sizeof (CAM_t), "CLIENT/CAM/CS01/1/AZ12B");
    return 1;
}

/*!
 * \brief sendDENM publishes a cam message on MQTT with hardcoded topic.
 * \param denm cam message struct
 * \return 1 if message sent succesfully
 */
I32 sendDENM(DENM_t* denm){

    printf("SENDING DENM\n");

    FILE *fp = fopen("tmp", "wb");
    //asn_enc_rval_t ec = der_encode(&asn_DEF_DENM, denm, write_out, fp);
    fclose(fp);

    publish_mqtt((char*)denm, sizeof (DENM_t), "CLIENT/DENM/CS01/1/AZ12B");
    return 1;
}

/*!
 * \brief parseActionConfiguration Parses an ACCM message into an action ID if it is of the infrastructure type
 * \param config ACCM data struct
 * \param actionIDs Pointer to a dynamically allocated array of actionIDs (may be null if empty)
 * \param nConfiguredActions Length of the actionID array
 * \return 0 if ACCM was decoded into an action, 1 if ACCM contained irrelevant action, -1 otherwise
 */
int parseActionConfiguration(ACCMData config, uint16_t* actionIDs, unsigned int* nConfiguredActions)
{
    int retval = -1;

    if (config.actionType == ACTION_INFRASTRUCTURE)
    {
        switch (config.actionTypeParameter1)
        {
        case ACTION_PARAMETER_VS_BRAKE_WARNING:
            actionIDs = realloc(actionIDs, (*nConfiguredActions+1)*sizeof(uint16_t));
            actionIDs[*nConfiguredActions] = config.actionID;
            (*nConfiguredActions)++;
            retval = 0;
            break;
        case ACTION_PARAMETER_UNAVAILABLE:
            LogMessage(LOG_LEVEL_WARNING, "First parameter of ACCM message is empty");
            retval = 1;
            break;
        default:
            LogMessage(LOG_LEVEL_WARNING, "Ignored ACCM parameter 1",config.actionTypeParameter1);
            retval = 1;
            break;
        }

        if (config.actionTypeParameter2 != ACTION_PARAMETER_UNAVAILABLE)
        {
            LogMessage(LOG_LEVEL_WARNING, "Ignored ACCM parameter 2");
            retval = 1;
        }
        if (config.actionTypeParameter3 != ACTION_PARAMETER_UNAVAILABLE)
        {
            LogMessage(LOG_LEVEL_WARNING, "Ignored ACCM parameter 3");
            retval = 1;
        }
    }
    else
        retval = 1;

    return retval;
}

/*!
 * \brief isActionValid Checks if specified action is among the configured actions
 * \param exac Action to check
 * \param actionIDs Array of configured actions
 * \param nConfiguredActions Length of actionIDs list
 * \return True if action is among configured actions, false otherwise
 */
bool isActionValid(EXACData exac, uint16_t* actionIDs, unsigned int nConfiguredActions)
{
    for (unsigned int i = 0; i < nConfiguredActions; ++i)
    {
        if (exac.actionID == actionIDs[i])
            return true;
    }
    return false;
}

