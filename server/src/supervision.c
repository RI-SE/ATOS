/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : supervision.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include "supervision.h"

#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
 #include <unistd.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LDM_SIZE            5
#define RECV_MESSAGE_BUFFER 1024

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
static void vCreateSafetyChannel(const char* name,const uint32_t port,
  int* sockfd, struct sockaddr_in* addr);
static void vCloseSafetyChannel(int* sockfd);
static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand);
static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void supervision_task()
{
  int safety_socket_fd[MAX_OBJECTS];
  struct sockaddr_in safety_object_addr[MAX_OBJECTS];
  char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
  uint32_t object_port[MAX_OBJECTS];
  int nbr_objects=0;

  monitor_t ldm[MAX_OBJECTS][LDM_SIZE];
  int ldm_act_step[MAX_OBJECTS];

  struct timespec sleep_time, ref_time;

  for(int i=0;i<nbr_objects;++i)
  {
    ldm_act_step[i] = 0;
  }

  (void)iCommInit(IPC_RECV_SEND,MQ_SV,1);

  /* Get objects; name, port and drive file */
  vUtilFindObjectsInfo(object_traj_file,object_address_name,object_port,&nbr_objects,SAFETY_CHANNEL_PORT);

  for(int i=0;i<nbr_objects;++i)
  {
    vCreateSafetyChannel(object_address_name[i],object_port[i],
      &safety_socket_fd[i],&safety_object_addr[i]);
  }

  /* Start sending and receiving HEAB, MONT and visualization */
  int iExit = 0;
  int iCommand;
  while(!iExit)
  {
    char buffer[RECV_MESSAGE_BUFFER];
    int recievedNewData = 0;

    #ifdef DEBUG
      struct timespec spec;
      clock_gettime(CLOCK_MONOTONIC, &spec);
      printf("INF: Time: %"PRIdMAX".%06ld \n",
        (intmax_t)spec.tv_sec, spec.tv_nsec);
      fflush(stdout);
    #endif

    for(int i=0;i<nbr_objects;++i)
    {
      bzero(buffer,RECV_MESSAGE_BUFFER);
      vSendHeartbeat(&safety_socket_fd[i],&safety_object_addr[i],COMMAND_HEARBEAT_GO);
      vRecvMonitor(&safety_socket_fd[i],buffer, RECV_MESSAGE_BUFFER, &recievedNewData);

      #ifdef DEBUG
        printf("INF: Did we recieve new data from %s %d: %d\n",object_address_name[i],object_port[i],recievedNewData);
        fflush(stdout);
      #endif

      if(recievedNewData)
      {
        /* Get monitor data */
        sscanf(buffer,"MONR;%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
          &ldm[i][ldm_act_step[i]].timestamp,&ldm[i][ldm_act_step[i]].latitude,&ldm[i][ldm_act_step[i]].longitude,
          &ldm[i][ldm_act_step[i]].altitude,&ldm[i][ldm_act_step[i]].speed,&ldm[i][ldm_act_step[i]].heading,&ldm[i][ldm_act_step[i]].drivedirection);

        bzero(buffer,RECV_MESSAGE_BUFFER);
        sprintf ( buffer,
          "%" PRIu16 ";0;%" PRIu64 ";%" PRId32 ";%" PRId32 ";%" PRId32 ";%" PRIu16 ";%" PRIu16 ";%" PRIu8 ";",
          i,ldm[i][ldm_act_step[i]].timestamp,ldm[i][ldm_act_step[i]].latitude,ldm[i][ldm_act_step[i]].longitude,
          ldm[i][ldm_act_step[i]].altitude,ldm[i][ldm_act_step[i]].speed,ldm[i][ldm_act_step[i]].heading,
          ldm[i][ldm_act_step[i]].drivedirection);
        #ifdef DEBUG
          printf("INF: Send MONITOR message: %s\n",buffer);
          fflush(stdout);
        #endif
        (void)iCommSend(COMM_MONI,buffer);

        ldm_act_step[i] = ++ldm_act_step[i] % LDM_SIZE;
      }
    }


    (void)iCommRecv(&iCommand,NULL,0);
	  if(iCommand == COMM_EXIT)
    {
      iExit = 1;  
    }
    else
    {
      #ifdef DEBUG
        printf("INF: Unhandled command in supervision\n");
        fflush(stdout);
      #endif
    }

    if(!iExit)
	  {
	    /* Make call periodic */
	    sleep_time.tv_sec = 0;
	    sleep_time.tv_nsec = 100000000;
	    (void)nanosleep(&sleep_time,&ref_time);
    }
  }

  /* Close safety socket */
  for(int i=0;i<nbr_objects;++i)
  {
    vCloseSafetyChannel(&safety_socket_fd[i]);
  }

  (void)iCommClose();

}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
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
      printf("INF: Sending: <HEBT>\n");
      fflush(stdout);
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
            printf("INF: No data receive\n");
            fflush(stdout);
          #endif
        }
      }
      else
      {
        *recievedNewData = 1;
        #ifdef DEBUG
          printf("INF: Received: <%s>\n",buffer);
          fflush(stdout);
        #endif
      }
    } while(result > 0 );
}
