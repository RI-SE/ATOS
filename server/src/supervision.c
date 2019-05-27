/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : supervision.c
  -- Author      :
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>

#include "supervision.h"



/* 34 years between 1970 and 2004, 8 days for leap year between 1970 and 2004      */
/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

#define MODULE_NAME "Supervisor"


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int supervision_task(TimeType *GPSTime, GSDType *GSD)
{

  I32 iExit = 0, iCommand;
  C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);


  LogMessage(LOG_LEVEL_INFO, "Supervision running with PID: %i", getpid());

  while(!iExit)
  {


    bzero(MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
    (void)iCommRecv(&iCommand,MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH, NULL);

    if(iCommand == COMM_EXIT)
    {
      iExit = 1;
      LogMessage(LOG_LEVEL_INFO, "Supervision exiting...");
      (void)iCommClose();
    }
  }


}
