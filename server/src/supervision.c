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

  char pcTempBuffer[512];

  struct timespec sleep_time, ref_time;

  for(iIndex=0;iIndex<MAX_OBJECTS;++iIndex)
  {
    ldm_act_step[iIndex] = 0;
  }

  (void)iCommInit(IPC_RECV,MQ_SV,0);

  /* Start sending and receiving HEAB, MONT and visualization */
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

        /* Check if passing line */
        ObjectPosition tObjectPos;
        double dP1Lat = 0;
        double dP1Long = 0;
        double dP2Lat = 0;
        double dP2Long = 0;


        bzero(pcTempBuffer,512);
        (void)iUtilGetParaConfFile("OrigoLatidude=",pcTempBuffer);
        sscanf(pcTempBuffer, "%lf", &dP1Lat);

        bzero(pcTempBuffer,512);
        (void)iUtilGetParaConfFile("OrigoLongitude=",pcTempBuffer);
        sscanf(pcTempBuffer, "%lf", &dP1Long);

        dP2Lat = (double)latitude/10000000;
        dP2Long = (double)longitude/10000000;

        #ifdef DEBUG
          printf("INF: Origo position latitude: %lf longitude: %lf \n",dP1Lat, dP1Long);
          fflush(stdout);
        #endif

        #ifdef DEBUG
          printf("INF: Object position latitude: %lf longitude: %lf \n",dP2Lat, dP2Long);
          fflush(stdout);
        #endif

        //(void)UtilCalcPositionDelta(dP1Lat, dP1Long, dP2Lat, dP2Long, &tObjectPos);
        //UtilCalcPositionDelta(57.777360,12.780472, 57.777711,12.780829, &tObjectPos);
        
        //double r = 6378137.0;
        //double latmid=(dP1Lat+dP2Lat)/2;
        //tObjectPos.x =(dP2Long-dP1Long)*(M_PI/180)*r*cos(latmid*M_PI/180);
        //tObjectPos.y =(dP2Lat-dP1Lat)*(M_PI/180)*r;
      

        #ifdef DEBUG
          printf("INF: Calculate value x: %lf y: %lf \n",tObjectPos.x, tObjectPos.y);
          fflush(stdout);
        #endif

        //double dRes = (tObjectPos.x-41.2)*(26.1-55.2)-(tObjectPos.y-55.2)*(65.2-41.2);
        double dRes = 1.0;
        if(dRes < 0.0)
        {
          printf("INF: Sending ABORT from supervisor\n");
          fflush(stdout);
          (void)iCommSend(COMM_ABORT,NULL);
        }

    }
	if(iCommand == COMM_REPLAY)
	{
        printf("INF: Supervision received REPLAY message: %s\n",cpBuffer);
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

  (void)iCommClose();

}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

