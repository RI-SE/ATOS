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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "util.h"
#include "supervision.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define LDM_SIZE            5
#define RECV_MESSAGE_BUFFER 1024
#define SAFETY_MARGIN       0.5
#define DEBUG 1

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
  FILE*     fp              ;
  char      bFileName [100] ;
  char      bFileLine [100] ;
  char*     bFileLine_p     ;
  size_t    len             ;
  int       read            ;

  char*     bLine      ;
  float     time       ;
  double    x          ;
  double    y          ;
  double    z          ;
  float     hdg        ;
  float     vel        ;

  monitor_t ldm          [ MAX_OBJECTS ] [ LDM_SIZE ] ;
  int       ldm_act_step [ MAX_OBJECTS ]              ;
  char      cpBuffer     [ RECV_MESSAGE_BUFFER ]      ;

  uint16_t  iIndex = 0;

  uint64_t  timestamp      ;
  int32_t   latitude       ;
  int32_t   longitude      ;
  int32_t   altitude       ;
  uint16_t  speed          ;
  uint16_t  heading        ;
  uint8_t   drivedirection ;

  struct timespec sleep_time ;
  struct timespec ref_time   ;
  
  char pcTempBuffer[512];

  double dRes = 0.0;
  
#ifdef DEBUG
  printf ("--------------------------------------------------\n");
  printf ("DBG: SV Supervision started.\n");
  printf ("--------------------------------------------------\n");
  fflush(stdout);
#endif
  
  /*----------------------------------------------------------------------
  -- Init
  ----------------------------------------------------------------------*/

  for(iIndex = 0; iIndex < MAX_OBJECTS; ++iIndex)
    {
      ldm_act_step[iIndex] = 0;
    }

  /*----------------------------------------------------------------------
  -- Open a drive file
  ----------------------------------------------------------------------*/

  bzero ( &bFileName, sizeof(bFileName) );
  sprintf ( bFileName, "%s", "traj/127.0.0.1");
  printf("INF: SV fname %s.\n", bFileName);
  fp = fopen ( bFileName, "r");
  if (fp == NULL)
    {
      perror ("ERR: SV Failed to open trajectory drive file");
      exit(1);
    }
  printf("INF: SV fp %d.\n", fp);

  bFileLine_p = bFileLine;

  bzero ( &bFileLine, sizeof(bFileLine) );
  read = getline ( &bFileLine_p, &len, fp);
  printf ( "INF: SV line %s.\n", bFileLine );
  bzero ( &bFileLine, sizeof(bFileLine) );
  read = getline ( &bFileLine_p, &len, fp);
  printf ( "INF: SV line %s.\n", bFileLine );

  bLine = (char *) malloc (200);

  sscanf ( &bFileLine [5],
           "%f;%lf;%lf;%lf;%f;%f;",
           &time,
           &x,
           &y,
           &z,
           &hdg,
           &vel );

  printf ( "INF: SV t %f, x %lf, y %lf, z %lf, hdg %f, vel %f\n",
           time,
           x,
           y,
           z,
           hdg,
           vel );

#ifdef DEBUG
  printf ( "DBG: SV Supervision read trajectory file %s,  r %d, l %d.\n",
           bFileName,
           read,
           len );
  fflush(stdout);
#endif
  
  /*----------------------------------------------------------------------
  -- Listen loop.
  ----------------------------------------------------------------------*/
  
  (void) iCommInit(IPC_RECV, MQ_SV, 0);

  /* Start sending and receiving HEAB, MONR and visualization */
  int iExit = 0;
  int iCommand;
  
  while(!iExit)
  {
    bzero(cpBuffer, RECV_MESSAGE_BUFFER);
    (void) iCommRecv(&iCommand, cpBuffer, RECV_MESSAGE_BUFFER);
    
    #ifdef DEBUG
      printf("INF: SV received a command: %s\n", cpBuffer);
      fflush(stdout);
    #endif

    if(iCommand == COMM_MONI)
    {
      #ifdef DEBUG
        printf("INF: Recieved MONITOR message: %s\n", cpBuffer);
        fflush(stdout);
      #endif

        /* Start parsing messages */
        sscanf(cpBuffer,
	       "%" SCNu16 ";0;%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
	       &iIndex,
	       &timestamp,
	       &latitude,
	       &longitude,
	       &altitude,
	       &speed,
	       &heading,
	       &drivedirection );

        ldm[iIndex][ldm_act_step[iIndex]].timestamp = timestamp;  
        ldm[iIndex][ldm_act_step[iIndex]].latitude = latitude;
        ldm[iIndex][ldm_act_step[iIndex]].longitude = longitude;
        ldm[iIndex][ldm_act_step[iIndex]].altitude = altitude;
        ldm[iIndex][ldm_act_step[iIndex]].speed = speed;
        ldm[iIndex][ldm_act_step[iIndex]].heading = heading;
        ldm[iIndex][ldm_act_step[iIndex]].drivedirection = drivedirection;

#ifdef DEBUG
        printf ( "DBG: SV i %d, s %d: t %" PRIu64 ", lat %d, lon %d, a %d, s %d, h %d, d %d \n",
                 iIndex                                                   ,
                 ldm_act_step [ iIndex ]                                  ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].timestamp      ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].latitude       ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].longitude      ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].altitude       ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].speed          ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].heading        ,
                 ldm [ iIndex ] [ ldm_act_step [ iIndex ]].drivedirection );
        fflush(stdout);
#endif
	ldm_act_step[iIndex] = ++ldm_act_step[iIndex] % LDM_SIZE;

        /* Check if passing line */
	
        ObjectPosition tObjectPos;
        double dP1Lat = 0;
        double dP1Long = 0;
        double dP2Lat = 0;
        double dP2Long = 0;


        bzero(pcTempBuffer,512);
        (void) iUtilGetParaConfFile("OrigoLatidude=", pcTempBuffer);
        sscanf(pcTempBuffer, "%lf", &dP1Lat);

        bzero(pcTempBuffer,512);
        (void) iUtilGetParaConfFile("OrigoLongitude=", pcTempBuffer);
        sscanf(pcTempBuffer, "%lf", &dP1Long);

        dP2Lat = (double) latitude / 10000000;
        dP2Long = (double) longitude / 10000000;

        #ifdef DEBUG
          printf("INF: Origo position latitude: %lf longitude: %lf \n",dP1Lat, dP1Long);
          fflush(stdout);
        #endif

        #ifdef DEBUG
          printf("INF: Object position latitude: %lf longitude: %lf \n",dP2Lat, dP2Long);
          fflush(stdout);
        #endif

	  UtilCalcPositionDelta(dP1Lat, dP1Long, dP2Lat, dP2Long, &tObjectPos);

        //double r = 6378137.0;
        //double latmid=(dP1Lat+dP2Lat)/2;
        //tObjectPos.x =(dP2Long-dP1Long)*(M_PI/180)*r*cos(latmid*M_PI/180);
        //tObjectPos.y =(dP2Lat-dP1Lat)*(M_PI/180)*r;

        #ifdef DEBUG
          printf("INF: Calculated value x: %lf y: %lf \n",tObjectPos.x, tObjectPos.y);
          fflush(stdout);
        #endif

        //double dRes = (tObjectPos.x-41.2)*(26.1-55.2)-(tObjectPos.y-55.2)*(65.2-41.2);

	  
	  dRes = sqrt((x-tObjectPos.x)*(x-tObjectPos.x)+(y-tObjectPos.y)*(y-tObjectPos.y));
	  
	  dRes = 0.0;
	  if(dRes > SAFETY_MARGIN)
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
        printf("INF: Unhandled command in supervision\n");
        fflush(stdout);
#endif
      }
  }  
  
  (void) iCommClose();
  fclose (fp);
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

