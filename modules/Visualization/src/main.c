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
#include <systemd/sd-daemon.h>
#include "logging.h"
#include "maestroTime.h"
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
void vCreateVisualizationMessage(ObjectDataType * _monitorData, char *_visualizationMessage,
								 int _sizeOfVisualizationMessage, int _debug);
static void signalHandler(int signo);

I32 iExit = 0;

void vCreateVisualizationMessage(ObjectDataType * _monitorData, char *_visualizationMessage,
								 int _sizeOfVisualizationMessage, int _debug) {

	char ipStringBuffer[INET_ADDRSTRLEN];

	sprintf(ipStringBuffer, "%s",
			inet_ntop(AF_INET, &_monitorData->ClientIP, ipStringBuffer, sizeof (ipStringBuffer)));
	char GPSMsOfWeekString[ENOUGH_BUFFER_SIZE];

	sprintf(GPSMsOfWeekString, "%u", TimeGetAsGPSqmsOfWeek(&_monitorData->MonrData.timestamp));
	char xPosString[ENOUGH_BUFFER_SIZE];

	sprintf(xPosString, "%.3f", _monitorData->MonrData.position.xCoord_m);
	char yPosString[ENOUGH_BUFFER_SIZE];

	sprintf(yPosString, "%.3f", _monitorData->MonrData.position.yCoord_m);
	char zPosString[ENOUGH_BUFFER_SIZE];

	sprintf(zPosString, "%.3f", _monitorData->MonrData.position.zCoord_m);
	char headingString[ENOUGH_BUFFER_SIZE];

	sprintf(headingString, "%.2f", _monitorData->MonrData.position.heading_rad * 180.0 / M_PI);
	char longSpeedString[ENOUGH_BUFFER_SIZE];

	sprintf(longSpeedString, "%.3f", _monitorData->MonrData.speed.longitudinal_m_s);
	char stateString[ENOUGH_BUFFER_SIZE];

	sprintf(stateString, "%s", objectStateToASCII(_monitorData->MonrData.state));

	//Build message from MonitorStruct
	snprintf(_visualizationMessage, _sizeOfVisualizationMessage, "%s;%s;%s;%s;%s;%s;%s;%s;",
			 ipStringBuffer,
			 GPSMsOfWeekString,
			 xPosString, yPosString, zPosString, headingString, longSpeedString, stateString);


	if (_debug) {
		//LogMessage(LOG_LEVEL_INFO, "%s", _visualizationMessage);
		LogPrint("IP: %s", ipStringBuffer);
		LogPrint("GPSQmsOfWeek: %s", GPSMsOfWeekString);
		LogPrint("X: %s", xPosString);
		LogPrint("Y: %s", yPosString);
		LogPrint("Z: %s", zPosString);
		LogPrint("Heading: %s", headingString);
		LogPrint("LongSpeed: %s", longSpeedString);
		LogPrint("State: %s", stateString);
		LogPrint("MESSAGE-SIZE = %d", _sizeOfVisualizationMessage);
	}
}

int main() {
	enum COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	char mqSendData[MQ_MSG_SIZE];

	const struct timespec sleepTimePeriod = { 0, 10000000 };
	struct timespec remTime;

	ObjectDataType monitorData;


	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	int visual_server;
	struct sockaddr_in visual_server_addr;

	//Setup signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Initialize message bus connection
	while (iCommInit()) {
		nanosleep(&sleepTimePeriod, &remTime);
	}

	// Notify service handler that startup was successful
	sd_notify(0, "READY=1");

	vConnectVisualizationChannel(&visual_server, &visual_server_addr);

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
		case COMM_INV:
			break;
		case COMM_OBC_STATE:
			break;
		case COMM_STRT:
			break;
		case COMM_GETSTATUS:
			memset(mqSendData, 0, sizeof (mqSendData));
			sprintf(mqSendData, "%s", MODULE_NAME);
			if (iCommSend(COMM_GETSTATUS_OK, mqSendData, sizeof (mqSendData)) < 0) {
				LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending GETSTATUS.");
			}
			break;
		case COMM_GETSTATUS_OK:
			break;
		default:
			LogMessage(LOG_LEVEL_WARNING, "Unhandled message bus command: %u", (unsigned char)command);
		}
	}

	//Return MQBus to "stack"
	(void)iCommClose();

	LogMessage(LOG_LEVEL_INFO, "Visualization exiting...");

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
