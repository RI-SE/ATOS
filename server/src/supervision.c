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

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LDM_SIZE            5
#define RECV_MESSAGE_BUFFER 1024

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


/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void supervision_task()
{
  monitor_t ldm[MAX_OBJECTS][LDM_SIZE];
  int ldm_act_step[MAX_OBJECTS];
  char cpBuffer[RECV_MESSAGE_BUFFER];

  uint16_t iIndex = 0;

  uint64_t timestamp;
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint16_t speed;
  uint16_t heading;
  uint8_t drivedirection;

  struct timespec sleep_time, ref_time;

  for(iIndex=0;iIndex<MAX_OBJECTS;++iIndex)
  {
    ldm_act_step[iIndex] = 0;
  }

  (void)iCommInit(IPC_RECV,MQ_SV,1);

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

    int iResult = 1;
    while(iResult > 0 && iExit == 0)
    {
      bzero(cpBuffer,RECV_MESSAGE_BUFFER);
      iResult = iCommRecv(&iCommand,cpBuffer,RECV_MESSAGE_BUFFER);

      if(iCommand == COMM_MONI)
      {
        #ifdef DEBUG
          printf("INF: Recieved MONITOR message: %s\n",cpBuffer);
          fflush(stdout);
        #endif

        /* Start parsing messages */
        sscanf(cpBuffer,"%" SCNu16 ";0;%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
          &iIndex,&timestamp,&latitude,&longitude,&altitude,&speed,&heading,&drivedirection);

        ldm[iIndex][ldm_act_step[iIndex]].timestamp = timestamp;  
        ldm[iIndex][ldm_act_step[iIndex]].latitude = latitude;
        ldm[iIndex][ldm_act_step[iIndex]].longitude = longitude;
        ldm[iIndex][ldm_act_step[iIndex]].altitude = altitude;
        ldm[iIndex][ldm_act_step[iIndex]].speed = speed;
        ldm[iIndex][ldm_act_step[iIndex]].heading = heading;
        ldm[iIndex][ldm_act_step[iIndex]].drivedirection = drivedirection;

        ldm_act_step[iIndex] = ++ldm_act_step[iIndex] % LDM_SIZE;
      }
  	  else if(iCommand == COMM_EXIT)
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
    }

    if(!iExit)
	  {
	    /* Make call periodic */
	    sleep_time.tv_sec = 0;
	    sleep_time.tv_nsec = 100000000;
	    (void)nanosleep(&sleep_time,&ref_time);
    }
  }

  (void)iCommClose();

}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

