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

#define SMALL_ITEM_TEXT_BUFFER_SIZE 20

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int *sockfd, struct sockaddr_in *addr);
static void vDisconnectVisualizationChannel(int *sockfd);
void vCreateVisualizationMessage(MonitorDataType *_monitorData, char *_visualizationMessage, int _debug);

void vCreateVisualizationMessage(MonitorDataType *_monitorData, char *_visualizationMessage, int _debug)
{
    sprintf(_visualizationMessage,"%d;%d;%d;%d", _monitorData->MONR.XPositionI32, _monitorData->MONR.YPositionI32, _monitorData->MONR.ZPositionI32, _monitorData->MONR.HeadingU16);

    if(_debug)
    {
        LogMessage(LOG_LEVEL_INFO, "X: %d", _monitorData->MONR.XPositionI32);
        LogMessage(LOG_LEVEL_INFO, "Y: %d", _monitorData->MONR.YPositionI32);
        LogMessage(LOG_LEVEL_INFO, "Z: %d", _monitorData->MONR.ZPositionI32);
        LogMessage(LOG_LEVEL_INFO, "Heading: %d", _monitorData->MONR.HeadingU16);
        LogMessage(LOG_LEVEL_INFO, "LatAcc: %d", _monitorData->MONR.LateralAccI16);
        LogMessage(LOG_LEVEL_INFO, "LongAcc: %d", _monitorData->MONR.LongitudinalAccI16);
    }
}

int main() {
	enum COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = { 0, 10000000 };
	const struct timespec abortWaitTime = { 1, 0 };
	struct timespec remTime;

    MonitorDataType monitorData;
    int sizeOfMessage = (sizeof (monitorData.MONR.XPositionI32) +
                         sizeof (monitorData.MONR.YPositionI32) +
                         sizeof (monitorData.MONR.ZPositionI32) +
                         sizeof (monitorData.MONR.HeadingU16));

	LogInit(MODULE_NAME, LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	int visual_server;
	struct sockaddr_in visual_server_addr;

	vConnectVisualizationChannel(&visual_server, &visual_server_addr);

    I32 iExit = 0;


	// Initialize message bus connection
	while (iCommInit()) {
		nanosleep(&sleepTimePeriod, &remTime);
	}

	while (true) {
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
		case COMM_MONI:
			// Ignore old style MONR data
			break;
        case COMM_MONR:
        {
            //Populate the monitorType
            UtilPopulateMonitorDataStruct(mqRecvData, (size_t) (sizeof (mqRecvData)), &monitorData, 0);

            //Create visualization message and insert values from the monitor datastruct above
            char visualizationMessage[42];
            vCreateVisualizationMessage(&monitorData, visualizationMessage, 0);

            //Send visualization message on the UDP socket
            UtilSendUDPData("Visualization", &visual_server, &visual_server_addr, visualizationMessage,
                            sizeof (visualizationMessage), 0);

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

	return 0;
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
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
