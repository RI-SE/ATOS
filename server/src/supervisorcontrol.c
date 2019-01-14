/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : supervisorcontrol.c
  -- Author      : Sebastian Loh Lindholm
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

#include "util.h"
#include "logger.h"


/*------------------------------------------------------------
  -- Definition declarations.
  ------------------------------------------------------------*/


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/



/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int supervisorcontrol_task(TimeType *GPSTime, GSDType *GSD)
{

  I32 iExit = 0, iCommand;
  C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);


  printf("Starting supervisor control...\n");
  while(!iExit)
  {


    bzero(MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
    (void)iCommRecv(&iCommand,MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);

    if(iCommand == COMM_EXIT)
    {
      iExit = 1;
      printf("supervisor control exiting.\n");
      (void)iCommClose();
    }
    
  }


}



