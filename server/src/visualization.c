/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : visualization.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <errno.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

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
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr);
static void vDisconnectVisualizationChannel(int* sockfd);
static void vSendVisualization(int* sockfd, 
  struct sockaddr_in* addr,
  const char* message);

void vISOtoCHRONOSmsg(char* ISOmsg,char* tarCHRONOSmsg,int MSG_size);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
int main(int argc, char *argv[])
{
	int visual_server;
	struct sockaddr_in visual_server_addr;
    char cpBuffer[RECV_MESSAGE_BUFFER];
    char chronosbuff[RECV_MESSAGE_BUFFER];

	printf("[Visualization] DefaultVisualizationAdapter started\n");
	fflush(stdout);

	(void)iCommInit(IPC_RECV,MQ_VA,0);

	vConnectVisualizationChannel(&visual_server,&visual_server_addr);

	/* Listen for commands */
	int iExit = 0;
	int iCommand;

	/* Execution mode*/
	int VisualExecutionMode = VISUAL_CONTROL_MODE;

  while(!iExit)
  {
    bzero(cpBuffer,RECV_MESSAGE_BUFFER);
    (void)iCommRecv(&iCommand,cpBuffer,RECV_MESSAGE_BUFFER);
    
    #ifdef DEBUG
      printf("INF: VA received a command: %s\n",cpBuffer);
      fflush(stdout);
    #endif

    if(iCommand == COMM_MONI)
    {
      #ifdef DEBUG
        printf("INF: Recieved MONITOR message: %s\n",cpBuffer);
        fflush(stdout);
      #endif

      vISOtoCHRONOSmsg(cpBuffer,chronosbuff,RECV_MESSAGE_BUFFER);
      vSendVisualization(&visual_server,&visual_server_addr,chronosbuff);

    }
	else if(iCommand == COMM_REPLAY)
	{
		VisualExecutionMode = VISUAL_REPLAY_MODE;
    #ifdef DEBUG
    printf("Visualization in REPLAY mode: %s\n",cpBuffer);
	 #endif
  }
	else if(iCommand == COMM_CONTROL)
	{
		VisualExecutionMode = VISUAL_CONTROL_MODE;
		#ifdef DEBUG
     printf("Visualization in CONTROL mode: %s\n", cpBuffer);
    #endif
	}
    else if(iCommand == COMM_EXIT)
    {
      iExit = 1;  
    }
    else
    {
      #ifdef DEBUG
        printf("INF: Unhandled command in visualization adapter\n");
        fflush(stdout);
      #endif
    }
  }  

  /* Close visualization socket */
  vDisconnectVisualizationChannel(&visual_server);

  (void)iCommClose();
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr)
{
  struct hostent *server;
  char pcTempBuffer[MAX_UTIL_VARIBLE_SIZE];

  /* Setup connection to visualization */
  #ifdef DEBUG
    printf("INF: Creating visualization socket.\n");
  #endif

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

  printf("[Visualization] UDP visualization sending to %s %d\n",pcTempBuffer,VISUAL_SERVER_PORT);
  fflush(stdout);

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

static void vSendVisualization(int* sockfd, struct sockaddr_in* addr,const char* message)
{
  char buffer[1024];
  int result;

  #ifdef DEBUG
    printf("INF: Buffer to visualization: <%s>\n",message);
    fflush(stdout);
  #endif

  result = sendto(*sockfd,
    message,
    strlen (message),
    0,
    (const struct sockaddr *) addr,
    sizeof(struct sockaddr_in));

    if (result < 0)
    {
      util_error("ERR: Failed to send on monitor socket");
    }
}

void vISOtoCHRONOSmsg(char* ISOmsg,char* tarCHRONOSmsg, int MSG_size)
{
    char IP_address[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char ID[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Timestamp[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Latitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Longitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Altitude[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char Heading[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LonSpeed[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LatSpeed[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LonAcc[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char LatAcc[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char DriveDirection[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char StateFlag[SMALL_ITEM_TEXT_BUFFER_SIZE];
    char StatusFlag[SMALL_ITEM_TEXT_BUFFER_SIZE];

    bzero(tarCHRONOSmsg,MSG_size);

    char* item_p = strtok(ISOmsg,";");
    int item_num = 0; // The placement incoming message
    while(item_p != NULL)
    {
        // What to do with each item at the current placement
        switch (item_num) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 11:
        case 12:
            strcat(tarCHRONOSmsg,item_p); strcat(tarCHRONOSmsg,";");
            break;
        case 6:
            bzero(Heading,SMALL_ITEM_TEXT_BUFFER_SIZE);
            int mod_heading = atoi(item_p)/10;
            sprintf(Heading,"%d;",mod_heading);
            break;
        case 7:
            strcat(tarCHRONOSmsg,item_p); strcat(tarCHRONOSmsg,";");
            strcat(tarCHRONOSmsg,Heading);
            break;
        default:
            break;
        }
        item_num++;
        item_p = strtok(NULL,";");
    }

}

