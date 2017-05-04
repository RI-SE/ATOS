/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.c
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include "objectcontrol.h"

#include <arpa/inet.h>
#include <dirent.h>
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
#define LOCALHOST "127.0.0.1"

#define RECV_MESSAGE_BUFFER 1024
#define OBJECT_MESS_BUFFER_SIZE 1024

#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define OBJECT_CONTROL_CONTROL_MODE 0
#define OBJECT_CONTROL_REPLAY_MODE 1

typedef enum {
  COMMAND_HEARBEAT_GO,
  COMMAND_HEARBEAT_ABORT
} hearbeatCommand_t;

typedef struct {
  uint64_t timestamp;
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint16_t speed;
  uint16_t heading;
  uint8_t drivedirection;
} monitor_t;

/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static void vConnectObject(int* sockfd,const char* name,const uint32_t port);
static void vSendString(const char* command,int* sockfd);
static void vSendFile(const char* object_traj_file, int* sockfd);
static void vDisconnectObject(int* sockfd);

static void vCreateSafetyChannel(const char* name,const uint32_t port,
  int* sockfd, struct sockaddr_in* addr);
static void vCloseSafetyChannel(int* sockfd);
static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand);
static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData);

static void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  int* nbr_objects);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void objectcontrol_task()
{
  int safety_socket_fd[MAX_OBJECTS];
  struct sockaddr_in safety_object_addr[MAX_OBJECTS];
  int socket_fd[MAX_OBJECTS];
  char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
  uint32_t object_udp_port[MAX_OBJECTS];
  uint32_t object_tcp_port[MAX_OBJECTS];
  int nbr_objects=0;
  int iExit = 0;
  int iCommand;
  char pcRecvBuffer[RECV_MESSAGE_BUFFER];
  char pcBuffer[OBJECT_MESS_BUFFER_SIZE];
  char pcTempBuffer[512];
  int iIndex = 0;
  monitor_t ldm[MAX_OBJECTS];
  struct timespec sleep_time, ref_time;
  int iForceObjectToLocalhost = 0;
  hearbeatCommand_t tServerStatus = COMMAND_HEARBEAT_GO;

  (void)iCommInit(IPC_RECV_SEND,MQ_OC,1);

    /* Get objects; name and drive file */
  vFindObjectsInfo(object_traj_file,object_address_name,&nbr_objects);


  (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",&iForceObjectToLocalhost);

  
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {

    if(0 == iForceObjectToLocalhost)
    {
      object_udp_port[iIndex] = SAFETY_CHANNEL_PORT;
      object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT;
    }
    else
    {
      object_udp_port[iIndex] = SAFETY_CHANNEL_PORT + iIndex*2;
      object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT + iIndex*2;
    }
  }
  

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

  /* Set status to go */
  tServerStatus = COMMAND_HEARBEAT_GO;

  /* Connect and send drive files */
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vConnectObject(&socket_fd[iIndex],object_address_name[iIndex],object_tcp_port[iIndex]);


    /* Send ODSM command */
    //vSendString(pcBuffer,&socket_fd[i]);

    /* Send DOPM command */
    vSendString("DOPM;",&socket_fd[iIndex]);
    vSendFile(object_traj_file[iIndex],&socket_fd[iIndex]);
    vSendString("ENDDOPM;",&socket_fd[iIndex]);
  }

  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vCreateSafetyChannel(object_address_name[iIndex],object_udp_port[iIndex],
      &safety_socket_fd[iIndex],&safety_object_addr[iIndex]);
  }

  //#ifdef DEBUG
    printf("INF: Object control start waiting for command\n");
    fflush(stdout);
  //#endif

  uint8_t uiTimeCycle = 0;


  /* Execution mode*/
  int ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;


  /* Should we exit? */
  while(!iExit)
  {
    char buffer[RECV_MESSAGE_BUFFER];
    int recievedNewData = 0;

    #ifdef DEBUG
/*
      struct timeval tvTime;
      gettimeofday(&tvTime, NULL);
      uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - 
        MS_FROM_1970_TO_2004_NO_LEAP_SECS + 
        DIFF_LEAP_SECONDS_UTC_ETSI*1000;
      printf("INF: Time: %" PRIu64 "\n",uiTime);
/*
      struct timespec spec;
      clock_gettime(CLOCK_MONOTONIC, &spec);
      printf("INF: Time: %"PRIdMAX".%06ld \n",
        (intmax_t)spec.tv_sec, spec.tv_nsec);
*/
      fflush(stdout);
    #endif

    for(iIndex=0;iIndex<nbr_objects;++iIndex)
    {
      /* Should we send heart beat in this cycle */
      if(uiTimeCycle == 0)
      {
        vSendHeartbeat(&safety_socket_fd[iIndex],&safety_object_addr[iIndex],tServerStatus);
      }
    }

    for(iIndex=0;iIndex<nbr_objects;++iIndex)
    {
      bzero(buffer,RECV_MESSAGE_BUFFER);
      vRecvMonitor(&safety_socket_fd[iIndex],buffer, RECV_MESSAGE_BUFFER, &recievedNewData);

      #ifdef DEBUG
      printf("INF: Did we recieve new data from %s %d: %d\n",object_address_name[iIndex],object_udp_port[iIndex],recievedNewData);
       fflush(stdout);
      #endif

      if(recievedNewData)
      {
        /* Get monitor data */
        sscanf(buffer,"MONR;%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
          &ldm[iIndex].timestamp,&ldm[iIndex].latitude,&ldm[iIndex].longitude,
          &ldm[iIndex].altitude,&ldm[iIndex].speed,&ldm[iIndex].heading,&ldm[iIndex].drivedirection);

        bzero(buffer,RECV_MESSAGE_BUFFER);
        sprintf ( buffer,
          "%" PRIu16 ";0;%" PRIu64 ";%" PRId32 ";%" PRId32 ";%" PRId32 ";%" PRIu16 ";%" PRIu16 ";%" PRIu8 ";",
          iIndex,ldm[iIndex].timestamp,ldm[iIndex].latitude,ldm[iIndex].longitude,
          ldm[iIndex].altitude,ldm[iIndex].speed,ldm[iIndex].heading,ldm[iIndex].drivedirection);

        #ifdef DEBUG
/*          struct timeval tvTime;
          gettimeofday(&tvTime, NULL);
          uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - 
            MS_FROM_1970_TO_2004_NO_LEAP_SECS + 
            DIFF_LEAP_SECONDS_UTC_ETSI*1000;
          printf("INF: Time: %" PRIu64 " Send MONITOR message: %s\n",uiTime,buffer);
/*
          struct timespec spec;
          clock_gettime(CLOCK_MONOTONIC, &spec);
          printf("INF: Time: %"PRIdMAX".%06ld Send MONITOR message: %s\n",
            (intmax_t)spec.tv_sec, spec.tv_nsec,buffer);
*/
          fflush(stdout);
        #endif

        #ifdef DEBUG
          printf("INF: Send MONITOR message: %s\n",buffer);
          fflush(stdout);
        #endif

        if(ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) (void)iCommSend(COMM_MONI,buffer);
      }
    }

    bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);

    // Have we recieved a command?
    if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER))
    {

      #ifdef DEBUG
        printf("INF: Object control command %d\n",iCommand);
        fflush(stdout);
      #endif

      if(iCommand == COMM_ABORT)
      {
        printf("INF: Recieved abort command\n");
        fflush(stdout);

        tServerStatus = COMMAND_HEARBEAT_ABORT;
      }
      else if(iCommand == COMM_ARMD)
      {
        tServerStatus = COMMAND_HEARBEAT_GO;
        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          vSendString("AROM;ENDAROM;",&socket_fd[iIndex]); 
        }
      }
      else if(iCommand == COMM_TRIG)
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
    	else if(iCommand == COMM_REPLAY)
    	{
  			ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
        printf("INF: Object control REPLAY mode <%s>\n", pcRecvBuffer);
  			fflush(stdout);
  		}
      else if(iCommand == COMM_CONTROL)
      {
        ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
        printf("INF: Object control in CONTROL mode\n");     
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

    if(!iExit)
    {
      /* Make call periodic */
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = TASK_PERIOD_MS*1000000;

      ++uiTimeCycle;
      if(uiTimeCycle >= HEARTBEAT_TIME_MS/TASK_PERIOD_MS)
      {
        uiTimeCycle = 0;
      }

      (void)nanosleep(&sleep_time,&ref_time);
    }
  }

  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vDisconnectObject(&socket_fd[iIndex]);
  }

    /* Close safety socket */
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vCloseSafetyChannel(&safety_socket_fd[iIndex]);
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
    printf("INF: Try to connect to socket: %s %i\n",name,port);
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

static void vCreateSafetyChannel(const char* name,const uint32_t port,
  int* sockfd, struct sockaddr_in* addr)
{
  int result;
  struct hostent *object;

  /* Connect to object safety socket */
  #ifdef DEBUG
    printf("INF: Creating safety socket\n");
    fflush(stdout);
  #endif

  *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
  {
    util_error("ERR: Failed to connect to monitor socket");
  }

  /* Set address to object */
  object = gethostbyname(name);
  if (object==0)
  {
    util_error("ERR: Unknown host");
  }

  bcopy((char *) object->h_addr, 
    (char *)&addr->sin_addr.s_addr, object->h_length);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);

   /* set socket to non-blocking */
  result = fcntl(*sockfd, F_SETFL, 
    fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  if (result < 0)
  {
    util_error("ERR: calling fcntl");
  }

  #ifdef DEBUG
    printf("INF: Created socket and safety address: %s %d\n",name,port);
    fflush(stdout);
  #endif

}

static void vCloseSafetyChannel(int* sockfd)
{
  close(*sockfd);
}

static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand)
{
    int result;
    char pcCommand[10];

    bzero(pcCommand,10);

    #ifdef DEBUG
      //printf("INF: Sending: <HEBT>\n");
      //fflush(stdout);
    #endif

    if(COMMAND_HEARBEAT_GO == tCommand)
    {
      strcat(pcCommand,"HEBT;g;");
    }
    else
    {
      strcat(pcCommand,"HEBT;A;");
    }

    #ifdef DEBUG
     printf("INF: Sending: <%s>\n",pcCommand);
     fflush(stdout);
    #endif

    result = sendto(*sockfd,
      pcCommand,
      10,
      0,
      (const struct sockaddr *) addr,
      sizeof(struct sockaddr_in));

    if (result < 0)
    {
      util_error("ERR: Failed to send on monitor socket");
    }
}

static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData)
{
  int result;
  *recievedNewData = 0;
    do
    {
      result = recv(*sockfd, 
        buffer,
        length,
        0);
      
      if (result < 0)
      {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
        {
          util_error("ERR: Failed to receive from monitor socket");
        }
        else
        {
          #ifdef DEBUG
            //printf("INF: No data receive\n");
            //fflush(stdout);
          #endif
        }
      }
      else
      {
        *recievedNewData = 1;
        #ifdef DEBUG
          //printf("INF: Received: <%s>\n",buffer);
          //fflush(stdout);
        #endif
      }
    } while(result > 0 );
}

void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  int* nbr_objects)
{
  DIR* traj_directory;
  struct dirent *directory_entry;
  int iForceObjectToLocalhost;

  iForceObjectToLocalhost = 0;

  traj_directory = opendir(TRAJECTORY_PATH);
  if(traj_directory == NULL)
  {
    util_error("ERR: Failed to open trajectory directory");
  }

  (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",&iForceObjectToLocalhost);

  while ((directory_entry = readdir(traj_directory)) && ((*nbr_objects) < MAX_OBJECTS))
  {
    
    /* Check so it's not . or .. */
    if (strncmp(directory_entry->d_name,".",1))
    {
      bzero(object_address_name[(*nbr_objects)],MAX_FILE_PATH);

      bzero(object_traj_file[(*nbr_objects)],MAX_FILE_PATH);
      (void)strcat(object_traj_file[(*nbr_objects)],TRAJECTORY_PATH);
      (void)strcat(object_traj_file[(*nbr_objects)],directory_entry->d_name);

      if(0 == iForceObjectToLocalhost)
      {
        (void)strncat(object_address_name[(*nbr_objects)],directory_entry->d_name,strlen(directory_entry->d_name));
      }
      else
      {
        (void)strcat(object_address_name[(*nbr_objects)],LOCALHOST);
      }

      ++(*nbr_objects);
    }
  }
  (void)closedir(traj_directory);
}
