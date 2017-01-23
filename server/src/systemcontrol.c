/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : systemcontrol.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include "systemcontrol.h"

#include <mqueue.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
typedef enum {
  SERVER_STATUS_INIT,
  SERVER_STATUS_OBJECT_CONNECTED,
  SERVER_STATUS_OBJECT_LOADED,
  SERVER_STATUS_ARMED,
  SERVER_STATUS_RUNNING,
  SERVER_STATUS_STOPPED,
  SERVER_STATUS_DONE
} state_t;

#define IPC_BUFFER_SIZE   256

/* Calculation: 
  34 years between 1970 and 2004 
  8 days for leap year between 1970 and 2004 
*/

/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Number of leap seconds since 1970 */
#define NBR_LEAP_SECONDS_FROM_1970 27


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void systemcontrol_task()
{
  state_t server_state = SERVER_STATUS_INIT;
  char pcBuffer[IPC_BUFFER_SIZE];
  struct timeval tvTime;
  int iExit = 0;

  (void)iCommInit(IPC_SEND,MQ_SC,0);

  while(!iExit)
  {
    bzero(pcBuffer,IPC_BUFFER_SIZE);
    scanf("%49s",pcBuffer);

    if(!strcmp(pcBuffer,"status"))
    {
      printf("Server status: %d\n",server_state);
    }
    else if(!strcmp(pcBuffer,"arm"))
    {
      (void)iCommSend(COMM_ARMD,NULL);
      server_state = SERVER_STATUS_ARMED;
    }
    else if(!strcmp(pcBuffer,"trig"))
    {
      bzero(pcBuffer, IPC_BUFFER_SIZE);

      gettimeofday(&tvTime, NULL);

      uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - 
        MS_FROM_1970_TO_2004_NO_LEAP_SECS + 
        NBR_LEAP_SECONDS_FROM_1970*1000;

      /* Add 5 seconds to get room for all objects to get command */
      uiTime += 5000;
    
      sprintf ( pcBuffer,"%" PRIu8 ";%" PRIu64 ";",0,uiTime);

      //#ifdef DEBUG
        printf("INF: System control Sending TRIG on IPC <%s>\n",pcBuffer);
        fflush(stdout);
      //#endif

      (void)iCommSend(COMM_TRIG,pcBuffer);
      server_state = SERVER_STATUS_RUNNING;
    }
    else if(!strcmp(pcBuffer,"stop"))
    {
      (void)iCommSend(COMM_STOP,NULL);
      server_state = SERVER_STATUS_STOPPED;
    }
    else if(!strcmp(pcBuffer,"exit"))
    {
      (void)iCommSend(COMM_EXIT,NULL);
      iExit = 1;  
    }
    else
    {
      printf("Unknown command\n");
    }
  }
  (void)iCommClose();
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
