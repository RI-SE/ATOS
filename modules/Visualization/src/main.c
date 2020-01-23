#include <unistd.h>
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
#include "logging.h"
#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MODULE_NAME "Visualization"
#define RECV_MESSAGE_BUFFER 1024
#define VISUAL_SERVER_NAME  "localhost"
#define VISUAL_SERVER_PORT  53250
#define VISUAL_CONTROL_MODE 0
#define VISUAL_REPLAY_MODE 1
#define ENOUGH_BUFFER_SIZE 64

#define SMALL_ITEM_TEXT_BUFFER_SIZE 20
#define MAX_DATE_STRLEN 25		// Maximum string length of a time stamp on the format "2035;12;31;24;59;59;1000" is 25


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int *sockfd, struct sockaddr_in *addr);
static void vDisconnectVisualizationChannel(int *sockfd);
void vCreateVisualizationMessage(MonitorDataType * _monitorData, char *_visualizationMessage,
								 int _sizeOfVisualizationMessage, int _debug);
static void signalHandler(int signo);

I32 iExit = 0;

void vCreateVisualizationMessage(MonitorDataType * _monitorData, char *_visualizationMessage,
								 int _sizeOfVisualizationMessage, int _debug) {

	char ipStringBuffer[INET_ADDRSTRLEN];
    sprintf(ipStringBuffer, "%s",
            inet_ntop(AF_INET, &_monitorData->ClientIP, ipStringBuffer, sizeof (ipStringBuffer)));
    char GPSMsOfWeekString[ENOUGH_BUFFER_SIZE];
    sprintf(GPSMsOfWeekString, "%u", _monitorData->MONR.GPSQmsOfWeekU32);
    char xPosString[ENOUGH_BUFFER_SIZE];
    sprintf(xPosString, "%d", _monitorData->MONR.XPositionI32);
    char yPosString[ENOUGH_BUFFER_SIZE];
    sprintf(yPosString, "%d", _monitorData->MONR.YPositionI32);
    char zPosString[ENOUGH_BUFFER_SIZE];
    sprintf(zPosString, "%d", _monitorData->MONR.ZPositionI32);
    char headingString[ENOUGH_BUFFER_SIZE];
    sprintf(headingString, "%u", _monitorData->MONR.HeadingU16);
    char longSpeedString[ENOUGH_BUFFER_SIZE];
    sprintf(longSpeedString, "%d", _monitorData->MONR.LongitudinalSpeedI16);
    char stateString[ENOUGH_BUFFER_SIZE];
    sprintf(stateString, "%u", _monitorData->MONR.StateU8);


	//Build message from MonitorStruct
    snprintf(_visualizationMessage, _sizeOfVisualizationMessage, "%s;%s;%s;%s;%s;%s;%s;%s;",
			 ipStringBuffer,
             GPSMsOfWeekString,
             xPosString,
             yPosString,
             zPosString,
             headingString,
             longSpeedString,
             stateString);


	if (_debug) {
		//LogMessage(LOG_LEVEL_INFO, "%s", _visualizationMessage);
		LogMessage(LOG_LEVEL_INFO, "IP: %s", ipStringBuffer);
		LogMessage(LOG_LEVEL_INFO, "GPSQmsOfWeek: %u", _monitorData->MONR.GPSQmsOfWeekU32);
		LogMessage(LOG_LEVEL_INFO, "X: %d", _monitorData->MONR.XPositionI32);
		LogMessage(LOG_LEVEL_INFO, "Y: %d", _monitorData->MONR.YPositionI32);
		LogMessage(LOG_LEVEL_INFO, "Z: %d", _monitorData->MONR.ZPositionI32);
		LogMessage(LOG_LEVEL_INFO, "Heading: %u", _monitorData->MONR.HeadingU16);
		LogMessage(LOG_LEVEL_INFO, "LongSpeed: %d", _monitorData->MONR.LongitudinalSpeedI16);
		LogMessage(LOG_LEVEL_INFO, "State: %u", _monitorData->MONR.StateU8);
        LogMessage(LOG_LEVEL_INFO, "MESSAGE-SIZE = %d",_sizeOfVisualizationMessage);
	}
}

int main() {
	enum COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = { 0, 10000000 };
	const struct timespec abortWaitTime = { 1, 0 };
	struct timespec remTime;

	MonitorDataType monitorData;


	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	int visual_server;
	struct sockaddr_in visual_server_addr;

	vConnectVisualizationChannel(&visual_server, &visual_server_addr);

    //Setup signal handlers
    if (signal(SIGINT, signalHandler) == SIG_ERR)
        util_error("Unable to initialize signal handler");




	// Initialize message bus connection
	while (iCommInit()) {
		nanosleep(&sleepTimePeriod, &remTime);
	}

    while (!iExit) {


		if (iCommRecv(&command, mqRecvData, MQ_MSG_SIZE, NULL) < 0) {
			util_error("Message bus receive error");
		}


		if (command == COMM_ABORT) {
		}

		if (command == COMM_EXIT) {
			iExit = 1;
			LogMessage(LOG_LEVEL_INFO, "Visualization exiting");
			vDisconnectVisualizationChannel(&visual_server);
			(void)iCommClose();
		}


		switch (command) {
		case COMM_INIT:
			break;

		case COMM_MONR:
		{

			//Populate the monitorType
			UtilPopulateMonitorDataStruct(mqRecvData, (size_t) (sizeof (mqRecvData)), &monitorData, 0);

            char dummy[1];
            int sizeOfVisualizationMessage;

            //Calculate size of incoming buffer
            sizeOfVisualizationMessage = snprintf(dummy, sizeof(dummy),"%u;%d;%d;%d;%u;%d;%u;",
                                                  monitorData.MONR.GPSQmsOfWeekU32,
                                                  monitorData.MONR.XPositionI32,
                                                  monitorData.MONR.YPositionI32,
                                                  monitorData.MONR.ZPositionI32,
                                                  monitorData.MONR.HeadingU16,
                                                  monitorData.MONR.LongitudinalSpeedI16,
                                                  monitorData.MONR.StateU8
                                                  );
            sizeOfVisualizationMessage += INET_ADDRSTRLEN;
            sizeOfVisualizationMessage += 8; //(;)

			//Allocate memory
			char *visualizationMessage = malloc(sizeOfVisualizationMessage * sizeof (char));

			//Create visualization message and insert values from the monitor datastruct above
            vCreateVisualizationMessage(&monitorData, visualizationMessage, sizeOfVisualizationMessage, 0);

			//Send visualization message on the UDP socket
			UtilSendUDPData((const uint8_t *)"Visualization", &visual_server, &visual_server_addr,
                            visualizationMessage, strlen(visualizationMessage), 0);

			//Free memory used by malloc
			free(visualizationMessage);

		}
			break;
		case COMM_LOG:
			break;
		case COMM_INV:
			break;
		case COMM_OBC_STATE:
			break;
		case COMM_STRT:
			break;
		default:
			LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", (unsigned char)command);
		}
	}

    //Return MQBus to "stack"
    MQBusDisconnect();

	return 0;
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
void signalHandler(int signo) {
    if (signo == SIGINT) {
        LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
        iExit = 1;
    }
    else {
        LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
    }
}


static void vConnectVisualizationChannel(int *sockfd, struct sockaddr_in *addr) {
	struct hostent *server;
	char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];

	/* Setup connection to visualization */
	//DEBUG_LPRINT(DEBUG_LEVEL_LOW,"%s","INF: Creating visualization socket.\n");

	*sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (*sockfd < 0) {
		util_error("ERR: Failed to create visualization socket");
	}

	bzero((char *)addr, sizeof (*addr));

	bzero(pcTempBuffer, MAX_UTIL_VARIBLE_SIZE);
	if (!iUtilGetParaConfFile("VisualizationServerName", pcTempBuffer)) {
		strcat(pcTempBuffer, VISUAL_SERVER_NAME);
	}

	//DEBUG_LPRINT(DEBUG_LEVEL_LOW,"[Visualization] UDP visualization sending to %s %d\n",pcTempBuffer,VISUAL_SERVER_PORT);


	server = gethostbyname(pcTempBuffer);

	if (server == NULL) {
		util_error("ERR: Unkonown host\n");
	}
	bcopy((char *)server->h_addr, (char *)&addr->sin_addr.s_addr, server->h_length);

	addr->sin_family = AF_INET;
	addr->sin_port = htons(VISUAL_SERVER_PORT);
}

static void vDisconnectVisualizationChannel(int *sockfd) {
	close(*sockfd);
}
