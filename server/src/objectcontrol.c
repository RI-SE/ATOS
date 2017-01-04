/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include "objectcontrol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define RECV_MESSAGE_BUFFER 1024
#define OBJECT_MESS_BUFFER_SIZE 1024

/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static void vConnectObject(int* sockfd,const char* name,const uint32_t port);
static void vSendString(const char* command,int* sockfd);
static void vSendFile(const char* object_traj_file, int* sockfd);
static void vDisconnectObject(int* sockfd);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void objectcontrol_task()
{
  int socket_fd[MAX_OBJECTS];
  char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
  uint32_t object_port[MAX_OBJECTS];
  int nbr_objects=0;
  int iExit = 0;
  int iCommand;
  char pcRecvBuffer[RECV_MESSAGE_BUFFER];
  char pcBuffer[OBJECT_MESS_BUFFER_SIZE];
  char pcTempBuffer[512];
  int iIndex = 0;

  (void)iCommInit(IPC_RECV,MQ_OC,0);

  /* Get objects; name, port and drive file */
  vUtilFindObjectsInfo(object_traj_file,object_address_name,object_port,&nbr_objects,CONTROL_CHANNEL_PORT);

  bzero(pcBuffer,OBJECT_MESS_BUFFER_SIZE);
  strcat(pcBuffer,"OSEM;");
  if(iUtilGetParaConfFile("OrigoLatidude",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoLatidude=");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoLongitude",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoLongitude=");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoAltitude",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoAltitude=");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoHeading",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoHeading=");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  strcat(pcBuffer,"ENDOSEM;");

  #ifdef DEBUG
    printf("INF: Created ODSM string <%s>\n",pcBuffer);
    fflush(stdout);
  #endif

  /* Connect and send drive files */
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vConnectObject(&socket_fd[iIndex],object_address_name[iIndex],object_port[iIndex]);


    /* Send ODSM command */
    //vSendString(pcBuffer,&socket_fd[i]);

    /* Send DOPM command */
    vSendString("DOPM;",&socket_fd[iIndex]);
    vSendFile(object_traj_file[iIndex],&socket_fd[iIndex]);
    vSendString("ENDDOPM;",&socket_fd[iIndex]);
  }

  //#ifdef DEBUG
    printf("INF: Object control start waiting for command\n");
    fflush(stdout);
  //#endif

  /* Listen for commands */
  while(!iExit)
  {
    bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);

    (void)iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER);

    #ifdef DEBUG
      printf("INF: Object control command %d\n",iCommand);
      fflush(stdout);
    #endif

    if(iCommand == COMM_TRIG)
    {    
      //#ifdef DEBUG
        printf("INF: Object control TRIG recvied string <%s>\n",pcRecvBuffer);
        fflush(stdout);
      //#endif

      bzero(pcBuffer,OBJECT_MESS_BUFFER_SIZE);
      strncat(pcBuffer,"TRIG;",5);
      strncat(pcBuffer,pcRecvBuffer,OBJECT_MESS_BUFFER_SIZE-13);
      strncat(pcBuffer,"ENDTRIG;",8);

      #ifdef DEBUG
        printf("INF: Sending TRIG from object control <%s>\n",pcBuffer);
        fflush(stdout);
      #endif

      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
        vSendString(pcBuffer,&socket_fd[iIndex]);
      }
    }
    else if(iCommand == COMM_STOP)
    {
      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
        vSendString("STOP;ENDSTOP;",&socket_fd[iIndex]); 
      }
    }
    else if(iCommand == COMM_EXIT)
    {
      iExit = 1;  
    }
    else
    {
      #ifdef DEBUG
        printf("Unhandled command in object control\n");
        fflush(stdout);
      #endif
    }
  }
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vDisconnectObject(&socket_fd[iIndex]);
  }
  (void)iCommClose();
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
static void vConnectObject(int* sockfd, const char* name, const uint32_t port)
{
  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  char buffer[256];
  int iResult;

  *sockfd = socket(AF_INET, SOCK_STREAM, 0);
   
  if (*sockfd < 0) 
  {
    util_error("ERR: Failed to open control socket");
  }

  server = gethostbyname(name);
  if (server == NULL) 
  {
    util_error("ERR: Unknown host ");
  }
  
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  
  bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(port);
  
  //#ifdef DEBUG
    printf("INF: Connect to command socket: %s %i\n",name,port);
    fflush(stdout);
  //#endif
  
  do
  {
    iResult = connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

    if ( iResult < 0) 
    {
      if(errno == ECONNREFUSED)
      {
        printf("WAR: Was not able to connect to object, retry in 3 sec...\n");
        fflush(stdout);
        (void)sleep(3);
      }
      else
      {
        util_error("ERR: Failed to connect to control socket");
      }
    }
  } while(iResult < 0);

  //#ifdef DEBUG
    printf("INF: Connected to command socket: %s %i\n",name,port);
    fflush(stdout);
  //#endif

}

static void vDisconnectObject(int* sockfd)
{
  close(*sockfd);
}

static void vSendString(const char* command, int* sockfd)
{
  int n;

  #ifdef DEBUG
    printf("INF: Sending: <%s>\n",command);
    fflush(stdout);
  #endif

  n = write(*sockfd, command, strlen(command));
  if (n < 0)
  {
    util_error("ERR: Failed to send on control socket");
  }
}

static void vSendFile(const char* object_traj_file, int* sockfd)
{
  FILE *filefd;
  char buffer[1024];
  int n;
  size_t readBytes;

  #ifdef DEBUG
    printf("INF: Open file %s\n",object_traj_file);
  #endif

  filefd = fopen (object_traj_file, "rb");
  if (filefd == NULL)
  {
    util_error("ERR: Failed to open trajectory file");
  }

  do
  {
    readBytes = fread(buffer,1,1024,filefd);
    if(readBytes > 0)
    {
      #ifdef DEBUG
        printf("INF: Sending: <%s>\n",buffer);
        printf("INF: Try to sending nbr of bytes: <%lu>\n",readBytes);
        fflush(stdout);
      #endif
      n = write(*sockfd, buffer, readBytes);
      if (n < 0)
      {
        util_error("ERR: Failed to send on control socket");
      }
    }
  } while(readBytes > 0);

  fclose(filefd);
}
