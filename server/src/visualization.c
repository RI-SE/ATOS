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
#include "visualization.h"

#include <errno.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define RECV_MESSAGE_BUFFER 1024
#define VISUAL_SERVER_NAME  "localhost"
#define VISUAL_SERVER_PORT  53250

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void vConnectVisualizationChannel(int* sockfd, struct sockaddr_in* addr);
static void vDisconnectVisualizationChannel(int* sockfd);
static void vSendVisualization(int* sockfd, 
  struct sockaddr_in* addr,
  const char* message);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void visualization_task()
{
  int visual_server;
  struct sockaddr_in visual_server_addr;
  char cpBuffer[RECV_MESSAGE_BUFFER];

  (void)iCommInit(IPC_RECV,MQ_VA,0);

  vConnectVisualizationChannel(&visual_server,&visual_server_addr);

  /* Listen for commands */
  int iExit = 0;
  int iCommand;
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
      vSendVisualization(&visual_server,&visual_server_addr,cpBuffer);
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

  printf("INF: UDP visualization sending to %s %d.\n",pcTempBuffer,VISUAL_SERVER_PORT);
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
