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
#include <signal.h>
#include <math.h>


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
#include "maestroTime.h"
#include "datadictionary.h"

#define H_THRESHOLD 1 //HEADING THRESHOLD
#define S_THRESHOLD 0 //SPEED THRESHOLD
#define D_THRESHOLD 1 //DISTANCE THRESHOLD
#define DENM_RETRANSMIT_CYCLE_S 1
#define DENM_RETRANSMIT_CYCLE_US 0
#define DENM_VALIDITY_DURATION_S 10
#define MAX_SIMULTANEOUS_DENM 10
#define CAM_TIME_CYCLE_S 1
#define CAM_TIME_CYCLE_US 0
#define SIG SIGRTMIN // Signal type to use for timers

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

#define ITS_STATION_ID 1000

#define DENM_V_1_2_1_MESSAGE_ID 1
#define DENM_V_1_2_1_PROTOCOL_VERSION 1
#define CAM_V_1_3_1_MESSAGE_ID 2
#define CAM_V_1_3_1_PROTOCOL_VERSION 1

#define MODULE_NAME "CitsControl"


/*------------------------------------------------------------
  -- Typedefs
  ------------------------------------------------------------*/
typedef struct
{
    struct timeval retransmissionEndTime;
    struct timeval nextRetransmissionTime;
    struct timeval retransmissionInterval;
    DENM_t* message;
} DENMRetransmission_t;

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
DENM_t* allocateDENMStruct(void);
void deallocateDENMStruct(DENM_t* denm);
void initializeDENMStruct(DENM_t* denm);

CAM_t* allocateCAMStruct(void);
void deallocateCAMStruct(CAM_t* cam);
void initializeCAMStruct(CAM_t* cam);

I32 generateCAMMessage(MONRType *MONRData, CAM_t* lastCam);
I32 generateDENMMessage(MONRType *MONRData, DENM_t* denm, int causeCode);
int updateDENMMessage(MONRType *MONRData, DENM_t* denm);

I32 sendCAM(CAM_t* lastCam);
I32 sendDENM(DENM_t* lastDENM);

void init_mqtt(char* ip_addr, char * clientid);
int connect_mqtt();
int publish_mqtt(char *payload, int payload_len, char *topic);
void delivered_mqtt(void *context, MQTTClient_deliveryToken dt);
int msgarrvd_mqtt(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void connlost_mqtt(void *context, char *cause);

bool validate_constraints(asn_TYPE_descriptor_t *type_descriptor, const void *struct_ptr) ;

void *encode_and_decode_object(asn_TYPE_descriptor_t *type_descriptor, void *struct_ptr) ;
int parseActionConfiguration(ACCMData config, uint16_t** actionIDs, int* nConfiguredActions);
int getActionIndex(EXACData exac, uint16_t* actionIDs, int nConfiguredActions);
int storeDENM(DENM_t denm);
static void signalHandler(int sig, siginfo_t *siginfo, void* uc);
void sendLastDENM(void);
ssize_t insertNewDENMIntoList(DENMRetransmission_t** list, size_t listSize);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

enum CITS_STATE {
    INIT,
    DISCONNECTED,
    CONNECTED,
    SENDING
};

struct Origo {
    double longitude;
    double latitude;
};


static int state = INIT;
static volatile int pending_state = INIT;
static volatile int iExit = 0;
static MQTTClient client;
static MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
static MQTTClient_message pubmsg = MQTTClient_message_initializer;
static volatile MQTTClient_deliveryToken deliveredtoken = 0;
static MQTTClient_deliveryToken sendtoken = 0;
static DENM_t *tempDENM;
static CAM_t* lastCAM, *tempCAM;
static struct Origo origin;
static DENMRetransmission_t* activeDENMs[MAX_SIMULTANEOUS_DENM];

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{
    origin.longitude = -200; //Initialize longitude to something outside -180 to 180
    origin.latitude = -100;  //Initialize latitude to something outside -90 to 90

    const struct timeval timeCycleCAM = {CAM_TIME_CYCLE_S,CAM_TIME_CYCLE_US};
    struct timeval nextCAMCycle;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;
    int mqtt_response_code = 0;
    MonitorDataType mqMONRdata;
    MONRType MONRMessage;
    MONRType LastMONRMessage;
    ACCMData actionConfig;
    EXACData actionData;
    uint16_t* storedActionIDs = NULL;
    int numberOfStoredActionIDs = 0;
    int actionIndex = -1;
    asn_enc_rval_t ec;

    LogInit(MODULE_NAME,logLevel);
    LogMessage(LOG_LEVEL_INFO, "C-ITS control running with PID: %i", getpid());


    lastCAM = allocateCAMStruct();
    tempCAM = allocateCAMStruct();
    if (lastCAM == NULL || tempCAM == NULL){
        exit(EXIT_FAILURE);
    }
    initializeCAMStruct(lastCAM);

    bzero(activeDENMs,sizeof (activeDENMs));

    tempDENM = allocateDENMStruct();
    if (tempDENM == NULL) {
        exit(EXIT_FAILURE);
    }

    TimeType time;

    I16 lastSpeed = 0;

    //! Timer for sending DENM at later time
    struct timeval systemTime, exacTime = {0,0};
    TimeSetToCurrentSystemTime(&nextCAMCycle);


    UtilGetMillisecond(&time);

    (void)init_mqtt(ERICSSON_MQTT_ADDRESS,DEFAULT_MQTT_CLIENTID);

    MQTTClient_setCallbacks(client, NULL, connlost_mqtt, msgarrvd_mqtt, delivered_mqtt);

    lastCAM->cam.generationDeltaTime = time.MillisecondU16;

    lastCAM->cam.camParameters.basicContainer.referencePosition.latitude = 0;
    lastCAM->cam.camParameters.basicContainer.referencePosition.longitude = 0;

    // TODO: Initialize signal handler

    if (iCommInit())
        util_error("Unable to connect to message bus");

    int monrCounter = 0;
    LogMessage(LOG_LEVEL_INFO,"Starting C-ITS control");
    while(!iExit)
    {
        TimeSetToCurrentSystemTime(&systemTime);

        // Handle states specific things
        state = pending_state;

        switch (state) {
        case INIT:
            if (!connect_mqtt()){
                LogMessage(LOG_LEVEL_INFO, "Connected to MQTT broker");
                MQTTClient_subscribe(client,DEFAULT_MQTT_TOPIC,DEFAULT_MQTT_QOS);
                pending_state = CONNECTED;
                LogMessage(LOG_LEVEL_DEBUG, "CITS state change from %d to %d",state,pending_state);
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
            // TODO: Implement
        }

        if(command == COMM_EXIT)
        {
            iExit = 1;
            LogMessage(LOG_LEVEL_INFO, "C-ITS control exiting");

            (void)iCommClose();
        }


        switch (command)
        {
        case COMM_INIT:
            free(storedActionIDs);
            break;
        case COMM_MONI:
            LogMessage(LOG_LEVEL_DEBUG, "Ignored old style MONR data");
            break;
        case COMM_OBJECTS_CONNECTED:
            TimeSetToCurrentSystemTime(&nextCAMCycle);
            break;
        case COMM_MONR:
            UtilPopulateMonitorDataStruct(busReceiveBuffer, sizeof(busReceiveBuffer),&mqMONRdata,0);
            MONRMessage = mqMONRdata.MONR;

            TimeSetToCurrentSystemTime(&systemTime);
            if(timercmp(&systemTime,&nextCAMCycle,>))
            {
                timeradd(&nextCAMCycle,&timeCycleCAM,&nextCAMCycle);

                I16 distanceDelta = 0;// UtilCoordinateDistance(LastMONRMessage.XPositionI32, LastMONRMessage.YPositionI32, MONRMessage.XPositionI32, MONRMessage.YPositionI32);
                I16 headingDelta = abs(LastMONRMessage.HeadingU16 - MONRMessage.HeadingU16);
                I16 speedDelta = abs((sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16))) - (lastSpeed));

                if(speedDelta >= S_THRESHOLD || distanceDelta >= D_THRESHOLD || headingDelta >= H_THRESHOLD){
                    generateCAMMessage(&MONRMessage, lastCAM);
                    sendCAM(lastCAM);

                    // Temporary
                    //generateDENMMessage(&MONRMessage, lastDENM, CauseCodeType_reserved);
                    //sendDENM(lastDENM);
                }
                lastSpeed = sqrt((MONRMessage.LateralSpeedI16*MONRMessage.LateralSpeedI16) + (MONRMessage.LongitudinalSpeedI16*MONRMessage.LongitudinalSpeedI16));
                LastMONRMessage = MONRMessage;
            }

            break;
        case COMM_ACCM:
            LogMessage(LOG_LEVEL_INFO,"Received action configuration");
            UtilPopulateACCMDataStructFromMQ(busReceiveBuffer, sizeof(busReceiveBuffer), &actionConfig);
            parseActionConfiguration(actionConfig, &storedActionIDs, &numberOfStoredActionIDs);
            break;
        case COMM_EXAC:
            UtilPopulateEXACDataStructFromMQ(busReceiveBuffer, sizeof(busReceiveBuffer), &actionData);
            actionIndex = getActionIndex(actionData, storedActionIDs, numberOfStoredActionIDs);
            if (actionIndex != -1 && state != DISCONNECTED && state != INIT)
            {
                LogMessage(LOG_LEVEL_INFO, "Received action execution request with ID %u", storedActionIDs[actionIndex]);

                if (timerisset(&exacTime))
                {
                    LogMessage(LOG_LEVEL_WARNING, "Another DENM send already queued");
                    break;
                }

                // Based on last MONR message, generate a DENM message to send
                ssize_t DENMindex = insertNewDENMIntoList(activeDENMs,MAX_SIMULTANEOUS_DENM);
                if (DENMindex == -1)
                {
                    LogMessage(LOG_LEVEL_ERROR,"Could not insert new DENM into list");
                    break;
                }
                generateDENMMessage(&MONRMessage, activeDENMs[DENMindex]->message, CauseCodeType_dangerousSituation);

                struct timeval validityDuration;
                if (actionData.executionTime_qmsoW == 0) // Immediate execution requested
                    activeDENMs[DENMindex]->nextRetransmissionTime = systemTime;
                else
                    TimeSetToGPStime(&activeDENMs[DENMindex]->nextRetransmissionTime, TimeGetAsGPSweek(&systemTime), actionData.executionTime_qmsoW);

                validityDuration.tv_sec = *activeDENMs[DENMindex]->message->denm.management.validityDuration;
                validityDuration.tv_usec = 0;

                timeradd(&activeDENMs[DENMindex]->nextRetransmissionTime, &validityDuration, &activeDENMs[DENMindex]->retransmissionEndTime);

                activeDENMs[DENMindex]->retransmissionInterval.tv_sec = DENM_RETRANSMIT_CYCLE_S;
                activeDENMs[DENMindex]->retransmissionInterval.tv_usec = DENM_RETRANSMIT_CYCLE_US;
            }
            else if (actionIndex == -1)
                LogMessage(LOG_LEVEL_DEBUG, "Received non-configured action execution request");
            else if (state == INIT || state == DISCONNECTED)
                LogMessage(LOG_LEVEL_WARNING, "Received action execution request while in non-connected state %u",(char)(state));
            break;
        case COMM_TRCM:
            // Future implementation
            break;
        case COMM_TREO:
            break;
        case COMM_OBC_STATE:
            break;
        case COMM_OSEM:
            // Save these values?
            break;
        case COMM_CONNECT:
            break;
        case COMM_LOG:
            break;
        case COMM_INV:
            break;
        case COMM_DATA_DICT:
            DataDictionaryGetOriginLatitudeDbl(GSD, &origin.latitude);
            DataDictionaryGetOriginLongitudeDbl(GSD, &origin.longitude);
            break;

        default:
            LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", command);
        }

        // Go through all stored DENMs, and see if any of them is up for retransmission
        TimeSetToCurrentSystemTime(&systemTime);
        for (int i = 0; i < MAX_SIMULTANEOUS_DENM; ++i)
        {
            if(activeDENMs[i] == NULL) continue;

            DENMRetransmission_t* aDENM = activeDENMs[i];

            if (timerisset(&aDENM->nextRetransmissionTime) && timercmp(&systemTime, &aDENM->nextRetransmissionTime, >))
            {
                updateDENMMessage(&MONRMessage,aDENM->message);
                sendDENM(aDENM->message);

                timeradd(&aDENM->nextRetransmissionTime,&aDENM->retransmissionInterval,&aDENM->nextRetransmissionTime);
                if (timercmp(&aDENM->nextRetransmissionTime,&aDENM->retransmissionEndTime,>))
                {
                    deallocateDENMStruct(aDENM->message);
                    free(aDENM);
                    activeDENMs[i] = NULL;
                }
            }
        }
    }

    for (int i = 0; i < MAX_SIMULTANEOUS_DENM; ++i)
    {
        deallocateDENMStruct(activeDENMs[i]->message);
        free(activeDENMs[i]);
    }
    free(storedActionIDs);
}


ssize_t insertNewDENMIntoList(DENMRetransmission_t** list, size_t listSize)
{
    for (unsigned int i = 0; i < listSize; ++i)
    {
        if (list[i] == NULL)
        {
            list[i] = calloc(1,sizeof(DENMRetransmission_t));
            list[i]->message = allocateDENMStruct();
            if (list[i]->message != NULL)
            {
                initializeDENMStruct(list[i]->message);
                return i;
            }
            else return -1;
        }
    }
    return -1;
}

void initializeDENMStruct(DENM_t* denm)
{
    denm->header.messageID = DENM_V_1_2_1_MESSAGE_ID;
    denm->header.protocolVersion = DENM_V_1_2_1_PROTOCOL_VERSION;
    denm->header.stationID = ITS_STATION_ID;

    /****** MANAGEMENT ******/
    denm->denm.management.actionID.originatingStationID = ITS_STATION_ID;
    denm->denm.management.actionID.sequenceNumber = 0;
    denm->denm.management.stationType = StationType_roadSideUnit;
    asn_long2INTEGER(&denm->denm.management.detectionTime,0); // TODO: double check
    asn_long2INTEGER(&denm->denm.management.referenceTime,0); // TODO: double check
    denm->denm.management.eventPosition.altitude.altitudeValue = 0;
    denm->denm.management.eventPosition.altitude.altitudeConfidence = 0;
    denm->denm.management.eventPosition.latitude = 0;
    denm->denm.management.eventPosition.longitude = 0;
    denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = 0;
    denm->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence = 0;
    denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 0;
    *denm->denm.management.validityDuration = DENM_VALIDITY_DURATION_S;
    *denm->denm.management.relevanceDistance = RelevanceDistance_over10km;
    *denm->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_allTrafficDirections;

    // Unused management optional fields (null their pointers to show unused)
    // TODO: Modify here to once relevant information can be used
    free(denm->denm.management.termination);
    denm->denm.management.termination = NULL;

    /****** SITUATION ******/
    denm->denm.situation->informationQuality = InformationQuality_highest;
    denm->denm.situation->eventType.causeCode = CauseCodeType_reserved;
    denm->denm.situation->eventType.subCauseCode = CauseCodeType_reserved;

    // Unused situation optional fields (null their pointers to show unused)
    // TODO: Modify here to once relevant information can be used
    free(denm->denm.situation->linkedCause);
    free(denm->denm.situation->eventHistory);
    denm->denm.situation->linkedCause = NULL;
    denm->denm.situation->eventHistory = NULL;

    /****** LOCATION ******/
    free(denm->denm.location);
    denm->denm.location = NULL;

    if (denm->denm.location != NULL)
    {
        denm->denm.location->eventSpeed->speedValue = 0;
        denm->denm.location->eventSpeed->speedValue = SpeedConfidence_unavailable;

        // Unused location optional fields (null their pointers to show unused)
        // TODO: Modify here to once relevant information can be used
        free(denm->denm.location->roadType);
        free(denm->denm.location->eventPositionHeading);
        denm->denm.location->roadType = NULL;
        denm->denm.location->eventPositionHeading = NULL;
    }

    /****** ALACARTE ******/

    // Unused alacarte optional fields (null their pointers to show unused)
    // TODO: Modify here to once relevant information can be used
    free(denm->denm.alacarte->lanePosition);
    free(denm->denm.alacarte->impactReduction);
    free(denm->denm.alacarte->externalTemperature);
    free(denm->denm.alacarte->roadWorks);
    free(denm->denm.alacarte->positioningSolution);
    free(denm->denm.alacarte->stationaryVehicle);
    denm->denm.alacarte->lanePosition = NULL;
    denm->denm.alacarte->impactReduction = NULL;
    denm->denm.alacarte->externalTemperature = NULL;
    denm->denm.alacarte->roadWorks = NULL;
    denm->denm.alacarte->positioningSolution = NULL;
    denm->denm.alacarte->stationaryVehicle = NULL;

    // TODO
}

DENM_t* allocateDENMStruct(void)
{
    // Allocate entire struct
    DENM_t* denm = calloc(1, sizeof(DENM_t));

    if (denm == NULL)
        return NULL;

    // Allocate management subcontainers
    denm->denm.management.termination = calloc(1, sizeof (Termination_t));
    denm->denm.management.relevanceDistance = calloc(1, sizeof (RelevanceDistance_t));
    denm->denm.management.relevanceTrafficDirection = calloc(1, sizeof (RelevanceTrafficDirection_t));
    denm->denm.management.validityDuration = calloc(1, sizeof (ValidityDuration_t));
    denm->denm.management.transmissionInterval = calloc(1, sizeof (TransmissionInterval_t));

    if (denm->denm.management.termination == NULL || denm->denm.management.relevanceDistance == NULL
            || denm->denm.management.relevanceTrafficDirection == NULL || denm->denm.management.validityDuration == NULL
            || denm->denm.management.transmissionInterval == NULL)
    {
        deallocateDENMStruct(denm);
        return NULL;
    }

    // Allocate subcontainers
    denm->denm.situation = calloc(1, sizeof(SituationContainer_t));
    denm->denm.location = calloc(1, sizeof(LocationContainer_t));
    denm->denm.alacarte = calloc(1, sizeof(AlacarteContainer_t));
    if (denm->denm.situation == NULL || denm->denm.location == NULL || denm->denm.alacarte == NULL)
    {
        deallocateDENMStruct(denm);
        return NULL;
    }
    else {
        // Initialize all pointers to null so that they can be passed to the deallocation
        // function if something goes wrong during initialization

        denm->denm.situation->linkedCause = NULL;
        denm->denm.situation->eventHistory = NULL;

        denm->denm.location->eventSpeed = NULL;
        denm->denm.location->eventPositionHeading = NULL;
        denm->denm.location->roadType = NULL;

        denm->denm.alacarte->lanePosition = NULL;
        denm->denm.alacarte->impactReduction = NULL;
        denm->denm.alacarte->externalTemperature = NULL;
        denm->denm.alacarte->roadWorks = NULL;
        denm->denm.alacarte->positioningSolution = NULL;
        denm->denm.alacarte->stationaryVehicle = NULL;
    }


    // Allocate situation subcontainers
    denm->denm.situation->linkedCause = calloc(1,sizeof (CauseCode_t));
    denm->denm.situation->eventHistory = NULL; // TODO: allocate memory for this
    if (denm->denm.situation->linkedCause == NULL /*|| denm->denm.situation->eventHistory == NULL*/)
    {
        deallocateDENMStruct(denm);
        return NULL;
    }

    // Allocate location subcontainers
    denm->denm.location->eventSpeed = calloc(1, sizeof (Speed_t));
    denm->denm.location->eventPositionHeading = calloc(1, sizeof (Heading_t));
    denm->denm.location->roadType = calloc(1, sizeof (RoadType_t));
    if (denm->denm.location->eventSpeed == NULL || denm->denm.location->eventPositionHeading == NULL
            || denm->denm.location->roadType == NULL)
    {
        deallocateDENMStruct(denm);
        return NULL;
    }

    // Allocate alacarte subcontainers
    denm->denm.alacarte->lanePosition = calloc(1, sizeof(LanePosition_t));
    denm->denm.alacarte->impactReduction = NULL; // TODO: allocate memory for this
    denm->denm.alacarte->externalTemperature = calloc(1, sizeof(Temperature_t));
    denm->denm.alacarte->roadWorks = NULL; // TODO: allocate memory for this
    denm->denm.alacarte->positioningSolution = calloc(1, sizeof (PositioningSolutionType_t));
    denm->denm.alacarte->stationaryVehicle = NULL; // TODO: allocate memory for this
    if (denm->denm.alacarte->lanePosition == NULL || denm->denm.alacarte->externalTemperature == NULL
            || denm->denm.alacarte->positioningSolution == NULL /*|| denm->denm.alacarte->impactReduction == NULL
            || denm->denm.alacarte->roadWorks == NULL || denm->denm.alacarte->stationaryVehicle == NULL*/)
    {
        deallocateDENMStruct(denm);
        return NULL;
    }

    return denm;
}

void deallocateDENMStruct(DENM_t* denm)
{
    if (denm == NULL) return;

    if (denm->denm.situation != NULL)
    {
        free(denm->denm.situation->linkedCause);
        free(denm->denm.situation->eventHistory); // TODO: deallocate possible submembers

        free(denm->denm.situation);
    }

    if (denm->denm.location != NULL)
    {
        free(denm->denm.location->eventSpeed);
        free(denm->denm.location->eventPositionHeading);
        free(denm->denm.location->roadType);

        free(denm->denm.location);
    }

    if (denm->denm.alacarte != NULL)
    {
        free(denm->denm.alacarte->lanePosition);
        free(denm->denm.alacarte->impactReduction); // TODO: deallocate possible submembers
        free(denm->denm.alacarte->externalTemperature);
        free(denm->denm.alacarte->roadWorks); // TODO: deallocate possible submembers
        free(denm->denm.alacarte->positioningSolution);
        free(denm->denm.alacarte->stationaryVehicle); // TODO: deallocate possible submembers

        free(denm->denm.alacarte);
    }

    free(denm);
}

CAM_t* allocateCAMStruct(void)
{
    CAM_t* cam = calloc(1,sizeof (CAM_t));
    const HighFrequencyContainer_PR chosenHFContainerType =  HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    if (cam == NULL)
    {
        LogMessage(LOG_LEVEL_ERROR, "Unable to allocate CAM struct");
        return NULL;
    }

    cam->cam.camParameters.highFrequencyContainer.present = chosenHFContainerType;
    switch (cam->cam.camParameters.highFrequencyContainer.present)
    {
    case HighFrequencyContainer_PR_NOTHING:
        util_error("Empty container unimplemented");
        break;
    case HighFrequencyContainer_PR_basicVehicleContainerHighFrequency:
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl = NULL; // TODO: Allocate memory for this
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition = calloc(1,sizeof(LanePosition_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle = calloc(1,sizeof(SteeringWheelAngle_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration = calloc(1,sizeof(LateralAcceleration_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration = calloc(1,sizeof(VerticalAcceleration_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass = calloc(1,sizeof(PerformanceClass_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone = calloc(1,sizeof(CenDsrcTollingZone_t));
        cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone->cenDsrcTollingZoneID = NULL; // TODO: Allocate memory for this

        BasicVehicleContainerHighFrequency_t* bvc = &cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
        if (bvc->lanePosition == NULL || bvc->steeringWheelAngle == NULL
                || bvc->lateralAcceleration == NULL || bvc->verticalAcceleration == NULL
                || bvc->performanceClass == NULL || bvc->cenDsrcTollingZone == NULL /*|| bvc->cenDsrcTollingZone->cenDsrcTollingZoneID == NULL || bvc->accelerationControl == NULL*/)
        {
            LogMessage(LOG_LEVEL_ERROR, "Unable to allocate CAM struct basic vehicle container");
            deallocateCAMStruct(cam);
            return NULL;
        }
        break;
    case HighFrequencyContainer_PR_rsuContainerHighFrequency:
        util_error("RSU container unimplemented");
        break;
    }
    return cam;
}

void initializeCAMStruct(CAM_t* cam)
{
    cam->header.stationID = ITS_STATION_ID;
    cam->header.messageID = CAM_V_1_3_1_MESSAGE_ID;
    cam->header.protocolVersion = CAM_V_1_3_1_PROTOCOL_VERSION;

    cam->cam.generationDeltaTime = 0;
    cam->cam.camParameters.basicContainer.stationType = StationType_roadSideUnit;
    cam->cam.camParameters.basicContainer.referencePosition.latitude = 0;
    cam->cam.camParameters.basicContainer.referencePosition.longitude = 0;
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = 0;
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 0;

    cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = HeadingValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = SpeedValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = SpeedConfidence_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = CurvatureValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = CurvatureCalculationMode_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = YawRateValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence = YawRateConfidence_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = LateralAccelerationValue_unavailable;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationConfidence = AccelerationConfidence_unavailable;

    // Unused highFrequencyContainer optional fields (null their pointers to show unused)
    // TODO: Modify here to once relevant information can be used
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl);
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition);
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle);
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration);
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass);
    free(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone);
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl = NULL;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition = NULL;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle = NULL;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration = NULL;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass = NULL;
    cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone = NULL;

    // Low frequency container and special vehicle containers unused for now (TODO)
    free(cam->cam.camParameters.lowFrequencyContainer);
    free(cam->cam.camParameters.specialVehicleContainer);
    cam->cam.camParameters.lowFrequencyContainer = NULL;
    cam->cam.camParameters.specialVehicleContainer = NULL;

    return;
}

void deallocateCAMStruct(CAM_t* cam)
{
    free(cam);
    // TODO: more
}

void signalHandler(int sig, siginfo_t* siginfo, void* uc)
{
    // TODO: check signal

    LogPrint("Caught %d", sig);

    //if (lastDENM != NULL)
    //    sendDENM(lastDENM);

    signal(sig, SIG_IGN);
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
        LogMessage(LOG_LEVEL_ERROR,"Failed to connect, return code %d", return_code);
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
    struct timeval tv, systemTime;

    initializeCAMStruct(tempCAM);

    tempCAM->header.protocolVersion = CAM_V_1_3_1_PROTOCOL_VERSION;
    tempCAM->header.messageID = CAM_V_1_3_1_MESSAGE_ID;
    tempCAM->header.stationID = ITS_STATION_ID;

    TimeSetToCurrentSystemTime(&systemTime);
    TimeSetToGPStime(&tv, TimeGetAsGPSweek(&systemTime), MONRData->GPSQmsOfWeekU32);
    tempCAM->cam.generationDeltaTime = TimeGetAsETSIms(&tv) % 65536; // Wrap to 65536 according to standard

    //LOG LAT from XY
    double x = MONRData->XPositionI32/1000.0;
    double y = MONRData->YPositionI32/1000.0;
    double z = MONRData->ZPositionI32/1000.0;
    double latitude, longitude, height;

    double distance=0;
    double azimuth1 = 0;
    double azimuth2 =0;
    int fail;

    if(origin.latitude < -90 || origin.latitude > 90 || origin.longitude < -180 || origin.longitude > 180)
    {
        LogMessage(LOG_LEVEL_WARNING, "Uninitialized origin: unable to write relevant data to CAM", origin.longitude, origin.latitude);
        return -1;
    }
    else
    {
        azimuth1 = UtilDegToRad(90)-atan2(y/1.00,x/1.00);
        distance = sqrt(pow(x/1.00,2)+pow(y/1.00,2));

        if(UtilVincentyDirect(origin.latitude, origin.longitude, azimuth1,distance ,&latitude,&longitude,&azimuth2) != -1)
        {
            // Convert to microdegrees
            tempCAM->cam.camParameters.basicContainer.referencePosition.latitude = (long)(latitude*1000000.0);
            tempCAM->cam.camParameters.basicContainer.referencePosition.longitude = (long)(longitude*1000000.0);

            tempCAM->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            tempCAM->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            tempCAM->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 0;

            tempCAM->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
            tempCAM->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
        }
        else LogMessage(LOG_LEVEL_ERROR, "Vincenty algorithm failed for CAM");
    }
    tempCAM->cam.camParameters.basicContainer.stationType = StationType_roadSideUnit;
    if (tempCAM->cam.camParameters.highFrequencyContainer.present == HighFrequencyContainer_PR_basicVehicleContainerHighFrequency)
    {
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = (long)(sqrt(pow((double)(MONRData->LongitudinalSpeedI16), 2) + pow((double)(MONRData->LateralSpeedI16), 2)));
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = SpeedConfidence_unavailable;

        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = MONRData->DriveDirectionU8;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = MONRData->HeadingU16 / 10;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_unavailable;

        if (MONRData->LongitudinalAccI16 == 32001)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
        else if (MONRData->LongitudinalAccI16 > 16000)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 160;
        else if (MONRData->LongitudinalAccI16 < -16000)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = -160;
        else
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = MONRData->LongitudinalAccI16 / 100;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;

        if (MONRData->LateralAccI16 == 32001)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = LateralAccelerationValue_unavailable;
        else if (MONRData->LateralAccI16 > 16000)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = 160;
        else if (MONRData->LateralAccI16 < -16000)
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = -160;
        else
            tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationValue = MONRData->LateralAccI16 / 100;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration->lateralAccelerationConfidence = AccelerationConfidence_unavailable;

        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = YawRateValue_unavailable;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence = YawRateConfidence_unavailable;

        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = CurvatureValue_unavailable;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = CurvatureCalculationMode_unavailable;

        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_unavailable;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
        tempCAM->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;


    } else LogMessage(LOG_LEVEL_ERROR, "Unimplemented CAM container %d", tempCAM->cam.camParameters.highFrequencyContainer.present);


    if(MONRData != NULL ){
        *cam = *tempCAM;
    }
    return 0;
}

/*!
 * \brief GenerateDENMMessage generates a DENM message to send on MQTT
 * \param MONRData MONR data struct
 * \param lastDENM struct to fill with DENM data if DENM should be sent, used as reference to calculate new DENM.
 * \param lastSpeed variable keeping track of last speed recorded.
 */
I32 generateDENMMessage(MONRType *MONRData, DENM_t* denm, int causeCode){
    struct timeval tv;
    long detectionTime;
    initializeDENMStruct(tempDENM);

    // Set reference time to current time, leave detection time as in previous message
    TimeSetToCurrentSystemTime(&tv);
    // TODO: make to work
    //asn_long2INTEGER(&tempDENM->denm.management.referenceTime,TimeGetAsETSIms(&tv));
    //asn_INTEGER2long(&lastDENM->denm.management.detectionTime,&detectionTime);
    //asn_INTEGER2long(&lastDENM->denm.management.referenceTime,&detectionTime);

    //LogPrint("rt: %ld, dt: %ld",TimeGetAsETSIms(&tv),detectionTime);

    //exit(EXIT_FAILURE);

    //asn_long2INTEGER(&tempDENM->denm.management.detectionTime,detectionTime);

    //LOG LAT from XY
    double x = MONRData->XPositionI32/1000.0;
    double y = MONRData->YPositionI32/1000.0;
    double z = MONRData->ZPositionI32/1000.0;
    double latitude, longitude, height;

    double distance = 0;
    double azimuth1 = 0;
    double azimuth2 = 0;

    if (origin.longitude > 180 || origin.longitude < -180 || origin.latitude > 90 || origin.latitude < -90)
    {
        LogMessage(LOG_LEVEL_WARNING, "Uninitialized origin: unable to write relevant data to DENM");

        tempDENM->denm.management.eventPosition.latitude = 0;
        tempDENM->denm.management.eventPosition.longitude = 0;

        tempDENM->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = 0;
        tempDENM->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 0;

        tempDENM->denm.management.eventPosition.altitude.altitudeValue = 0;
        tempDENM->denm.management.eventPosition.altitude.altitudeConfidence = 0;
    }
    else
    {
        azimuth1 = UtilDegToRad(90)-atan2(y/1.00,x/1.00);
        distance = sqrt(pow(x/1.00,2)+pow(y/1.00,2));

        if(UtilVincentyDirect(origin.latitude, origin.longitude,azimuth1,distance ,&latitude,&longitude,&azimuth2) != -1)
        {
            // Convert to microdegrees
            tempDENM->denm.management.eventPosition.latitude = (long)(latitude*1000000.0);
            tempDENM->denm.management.eventPosition.longitude = (long)(longitude*1000000.0);

            tempDENM->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            tempDENM->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            tempDENM->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 0;

            tempDENM->denm.management.eventPosition.altitude.altitudeValue = AltitudeValue_unavailable;
            tempDENM->denm.management.eventPosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
        }
        else LogMessage(LOG_LEVEL_ERROR, "Vincenty algorithm failed for DENM");
    }

    *tempDENM->denm.management.relevanceDistance = RelevanceDistance_lessThan500m;
    *tempDENM->denm.management.relevanceTrafficDirection = RelevanceTrafficDirection_upstreamTraffic;

    *tempDENM->denm.management.transmissionInterval = CAM_TIME_CYCLE_S*1000 + CAM_TIME_CYCLE_US/1000;

    tempDENM->denm.situation->eventType.causeCode = causeCode;

    if(causeCode == CauseCodeType_dangerousSituation){
        tempDENM->denm.situation->eventType.subCauseCode = 1;  // Emergency brake engaged
    }
    else{
        tempDENM->denm.situation->eventType.subCauseCode = 0;
    }

    // TODO: make to work
    //double lonSpeed_cm_s = (double)MONRData->LongitudinalSpeedI16;
    //double latSpeed_cm_s = (double)MONRData->LateralSpeedI16;
    //tempDENM->denm.location->eventSpeed->speedValue = (long)(sqrt(pow(lonSpeed_cm_s,2) + pow(latSpeed_cm_s,2)));
    //tempDENM->denm.location->eventSpeed->speedConfidence = SpeedConfidence_unavailable;


    if(MONRData != NULL ){
        *denm = *tempDENM;
    }
    return 0;
}

int updateDENMMessage(MONRType *MONRData, DENM_t* denm)
{
    struct timeval tv;
    double x, y, z, latitude, longitude, distance, azimuth1, azimuth2;

    if (MONRData == NULL || denm == NULL) return -1;

    // TODO: make to work
    //asn_long2INTEGER(&tempDENM->denm.management.referenceTime,TimeGetAsETSIms(&tv));

    //LOG LAT from XY
    x = MONRData->XPositionI32/1000.0;
    y = MONRData->YPositionI32/1000.0;
    z = MONRData->ZPositionI32/1000.0;

    distance = 0;
    azimuth1 = 0;
    azimuth2 = 0;

    if (origin.longitude > 180 || origin.longitude < -180 || origin.latitude > 90 || origin.latitude < -90)
    {
        LogMessage(LOG_LEVEL_WARNING, "Uninitialized origin: unable to write relevant data to DENM");

        denm->denm.management.eventPosition.latitude = 0;
        denm->denm.management.eventPosition.longitude = 0;

        denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = 0;
        denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 0;

        denm->denm.management.eventPosition.altitude.altitudeValue = 0;
        denm->denm.management.eventPosition.altitude.altitudeConfidence = 0;
        return -1;
    }
    else
    {
        azimuth1 = UtilDegToRad(90)-atan2(y/1.00,x/1.00);
        distance = sqrt(pow(x/1.00,2)+pow(y/1.00,2));

        if(UtilVincentyDirect(origin.latitude, origin.longitude,azimuth1,distance ,&latitude,&longitude,&azimuth2) != -1)
        {
            // Convert to microdegrees
            denm->denm.management.eventPosition.latitude = (long)(latitude*1000000.0);
            denm->denm.management.eventPosition.longitude = (long)(longitude*1000000.0);

            denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            denm->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation = 0;

            denm->denm.management.eventPosition.altitude.altitudeValue = AltitudeValue_unavailable;
            denm->denm.management.eventPosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
        }
        else LogMessage(LOG_LEVEL_ERROR, "Vincenty algorithm failed for DENM");
    }
    return 0;
}

/*!
 * \brief SendCam publishes a cam message on MQTT with hardcoded topic.
 * \param lastCam cam message struct
 * \return 1 if message sent succesfully
 */
I32 sendCAM(CAM_t* cam){
    if (cam == NULL)
    {
        LogMessage(LOG_LEVEL_ERROR, "Attempted to send null CAM");
        return -1;
    }

    void* buffer = NULL;
    struct asn_per_constraints_s* constraints = NULL;
    const ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_CAM, constraints, cam, &buffer);

    if (ec != -1)
    {

        FILE *fp = fopen("asn1test_cam.hx", "w");
        for (ssize_t i = 0; i < ec; ++i) {
            fprintf(fp,"%02X ",(((unsigned char*)buffer)[i]));
        }
        fclose(fp);

        LogMessage(LOG_LEVEL_INFO,"Sending CAM");
        publish_mqtt((char*)buffer, ec, "CLIENT/CAM/CS01/1/AZ12B");
        return 1;
    }
    else {
        LogMessage(LOG_LEVEL_ERROR,"Encoding of CAM message failed");
        return -1;
    }
}

/*!
 * \brief sendDENM publishes a cam message on MQTT with hardcoded topic.
 * \param denm cam message struct
 * \return 0 if message sent succesfully
 */
I32 sendDENM(DENM_t* denm){
    if  (denm == NULL)
    {
        LogMessage(LOG_LEVEL_ERROR, "Attempted to send null DENM");
        return -1;
    }

    void* buffer = NULL;
    struct asn_per_constraints_s* constraints = NULL;
    const ssize_t ec = uper_encode_to_new_buffer(&asn_DEF_DENM, constraints, denm, &buffer);

    if (ec != -1)
    {
        FILE *fp = fopen("asn1test_denm.hx", "w");
        for (ssize_t i = 0; i < ec; ++i) {
            fprintf(fp,"%02X ",(((unsigned char*)buffer)[i]));
        }
        fclose(fp);

        LogMessage(LOG_LEVEL_INFO,"Sending DENM");
        publish_mqtt((char*)buffer, ec, "CLIENT/DENM/CS01/1/AZ12B");
    }
    else
    {
        LogMessage(LOG_LEVEL_ERROR,"Encoding of DENM message failed");
        return -1;
    }
    return 0;
}

/*!
 * \brief parseActionConfiguration Parses an ACCM message into an action ID if it is of the infrastructure type
 * \param config ACCM data struct
 * \param actionIDs Pointer to a dynamically allocated array of actionIDs (may be null if empty)
 * \param nConfiguredActions Length of the actionID array
 * \return 0 if ACCM was decoded into an action, 1 if ACCM contained irrelevant action, -1 otherwise
 */
int parseActionConfiguration(ACCMData config, uint16_t** actionIDs, int* nConfiguredActions)
{
    int retval = -1;
    if (*nConfiguredActions < 0)
        return -1;

    if (config.actionType == ACTION_INFRASTRUCTURE)
    {
        switch (config.actionTypeParameter1)
        {
        case ACTION_PARAMETER_VS_BRAKE_WARNING:
            *actionIDs = (uint16_t*)realloc(*actionIDs, (unsigned int)(*nConfiguredActions+1)*sizeof(uint16_t));
            (*actionIDs)[*nConfiguredActions] = config.actionID;
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
 * \brief getActionIndex Checks if specified action is among the configured actions, and returns its index if so
 * \param exac Action to check
 * \param actionIDs Array of configured actions
 * \param nConfiguredActions Length of actionIDs list
 * \return Index of specified action, -1 if not found
 */
int getActionIndex(EXACData exac, uint16_t* actionIDs, int nConfiguredActions)
{
    if (actionIDs == NULL)
        return -1;

    for (int i = 0; i < nConfiguredActions; ++i)
    {
        if (exac.actionID == actionIDs[i])
            return i;
    }
    return -1;
}

