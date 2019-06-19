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

#include "MQTTClient.h"
#include "citscontrol.h"



#define CITS_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define CITS_CONTROL_BUFFER_SIZE_20 20
#define CITS_CONTROL_BUFFER_SIZE_52 52
#define CITS_CONTROL_TASK_PERIOD_MS 1

#define DEFAULT_MQTT_ADDRESS     "tcp://localhost:1883"
#define DEFAULT_MQTT_CLIENTID    "ExampleClientPub"
#define DEFAULT_MQTT_TOPIC       "MQTT Examples"
#define DEFAULT_MQTT_PAYLOAD     "Hello World!"
#define DEFAULT_MQTT_QOS         1
#define DEFAULT_MQTT_TIMEOUT     10000L

#define MODULE_NAME "CitsControl"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void init_mqtt(char* ip_addr, char * clientid);
int connect_mqtt();
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
static int pending_state = INIT;

static MQTTClient client;
static MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
static MQTTClient_message pubmsg = MQTTClient_message_initializer;
static MQTTClient_deliveryToken deliveredtoken;

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void citscontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{

    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;

    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());

    (void)iCommInit();

    (void)init_mqtt(DEFAULT_MQTT_ADDRESS,DEFAULT_MQTT_CLIENTID);

    MQTTClient_setCallbacks(client, NULL, connlost_mqtt, msgarrvd_mqtt, delivered_mqtt);


    LogMessage(LOG_LEVEL_INFO,"Starting cits control...\n");
    while(!iExit)
    {

        // Handle states specific things
        state = pending_state;
        switch (state) {
        case INIT:

            if (connect_mqtt()) pending_state = CONNECTED;

            break;
        case CONNECTED:
            break;
        }

        // Handle MQ messages

        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));
        (void)iCommRecv(&command,busReceiveBuffer, sizeof(busReceiveBuffer), NULL);

        if(command == COMM_EXIT)
        {
            iExit = 1;
            printf("citscontrol exiting.\n");
            (void)iCommClose();
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
        return 0;
    }
    return 1;
}

int msgarrvd_mqtt(void *context, char *topicName, int topicLen, MQTTClient_message *message){
    int i;
    char* payloadptr;
    printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: ");
    payloadptr = message->payload;
    for(i=0; i<message->payloadlen; i++)
    {
        putchar(*payloadptr++);
    }
    putchar('\n');
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void delivered_mqtt(void *context, MQTTClient_deliveryToken dt)
{
    LogMessage(LOG_LEVEL_DEBUG,"Message delivered with token: %d .", dt);
    deliveredtoken = dt;
    pending_state = CONNECTED;
}

void connlost_mqtt(void *context, char *cause){
    LogMessage(LOG_LEVEL_DEBUG,"Connection Lost.\n Cause: %s",cause);
    pending_state = DISCONNECTED;
}
