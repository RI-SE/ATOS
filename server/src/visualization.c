#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "visualization.h"
#include "util.h"


/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define RECV_MESSAGE_BUFFER 1024
#define VISUAL_SERVER_NAME  "localhost"
#define VISUAL_SERVER_PORT  53250
#define VISUAL_CONTROL_MODE 0
#define VISUAL_REPLAY_MODE 1

#define SMALL_ITEM_TEXT_BUFFER_SIZE 20
#define MODULE_NAME "VisualizationAdapter"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr);
static void vDisconnectVisualizationChannel(int* sockfd);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void visualization_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
{
    int visual_server;
    struct sockaddr_in visual_server_addr;
    vConnectVisualizationChannel(&visual_server,&visual_server_addr);

    int camTimeCycle = 0;
    I32 iExit = 0;
    char busReceiveBuffer[MBUS_MAX_DATALEN];               //!< Buffer for receiving from message bus
    enum COMMAND command;

    (void)iCommInit();
    LogInit(MODULE_NAME,LOG_LEVEL_INFO);
    LogMessage(LOG_LEVEL_INFO, "Visualization running with PID: %i", getpid());

    while(!iExit)
    {
        // Handle MQ messages
        bzero(busReceiveBuffer, sizeof(busReceiveBuffer));
               (void)iCommRecv(&command,busReceiveBuffer, sizeof(busReceiveBuffer), NULL);
               if (command == COMM_ABORT)
               {

               }

               if(command == COMM_EXIT)
               {
                   iExit = 1;
                   printf("Vizualisation exiting.\n");
                   vDisconnectVisualizationChannel(&visual_server);
                   (void)iCommClose();
               }
                //usleep(100000);
               switch (command)
               {
               case COMM_INIT:

                   break;
               case COMM_MONI:
                   // Ignore old style MONR data
                   break;
               case COMM_MONR:
                   printf("Vizualisation got MONR.\n");
                   //UtilSendUDPData("Visualization", &visual_server, &visual_server_addr, busReceiveBuffer, sizeof(busReceiveBuffer), 0);
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

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr)
{
    struct hostent *server;
    char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];

    /* Setup connection to visualization */
    //DEBUG_LPRINT(DEBUG_LEVEL_LOW,"%s","INF: Creating visualization socket.\n");

    *sockfd = socket ( AF_INET,
                       SOCK_DGRAM,
                       IPPROTO_UDP );

    if (*sockfd < 0)
    {
        util_error("ERR: Failed to create visualization socket");
    }

    bzero((char *)addr, sizeof(*addr));

    bzero(pcTempBuffer,MAX_UTIL_VARIBLE_SIZE);
    if(!iUtilGetParaConfFile("VisualizationServerName",pcTempBuffer))
    {
        strcat(pcTempBuffer,VISUAL_SERVER_NAME);
    }

    //DEBUG_LPRINT(DEBUG_LEVEL_LOW,"[Visualization] UDP visualization sending to %s %d\n",pcTempBuffer,VISUAL_SERVER_PORT);


    server = gethostbyname(pcTempBuffer);

    if (server == NULL)
    {
        util_error("ERR: Unkonown host\n");
    }
    bcopy((char *) server->h_addr,
          (char *)&addr->sin_addr.s_addr, server->h_length);

    addr->sin_family = AF_INET;
    addr->sin_port   = htons(VISUAL_SERVER_PORT);
}

static void vDisconnectVisualizationChannel(int* sockfd)
{
    close(*sockfd);
}


