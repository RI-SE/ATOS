/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.c
  -- Author      : Karl-Johan Ode
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "util.h"

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MQ_NBR_QUEUES 4

#define TEST_CONF_FILE "./conf/test.conf"

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/
static mqd_t tMQRecv;
static mqd_t ptMQSend[MQ_NBR_QUEUES];
static char pcMessageQueueName[1024];

/*---------------------------------------------s---------------
  -- Public functions
  ------------------------------------------------------------*/
void util_error(char* message)
{
  perror(message);
  exit(EXIT_FAILURE);
}

int iUtilGetParaConfFile(char* pcParameter, char* pcValue) 
{
  FILE *filefd;
  int iFindResult;
  char pcTemp[512];

  iFindResult = 0;

  filefd = fopen (TEST_CONF_FILE, "rb");

  if (filefd == NULL)
  {
    return 0;
  }
  
  while(fgets(pcTemp, 512, filefd) != NULL)
  {
    if((strstr(pcTemp, pcParameter)) != NULL) 
    {
      /* Does contain any value? */
      if(strlen(pcTemp) > (strlen(pcParameter)+1))
      {
        /* replace new line */
        if(pcTemp[strlen(pcTemp)-1] == '\n')
        {
          pcTemp[strlen(pcTemp)-1] = 0;
        }
        strcpy(pcValue,&pcTemp[strlen(pcParameter)+1]);
      }
      iFindResult = 1;
    }
  }
  
  if(filefd) 
  {
    fclose(filefd);
  }

  return 1;
}

int iUtilGetIntParaConfFile(char* pcParameter, int* iValue)
{
  int iResult;
  char pcValue[512];

  bzero(pcValue,512);
  iResult = iUtilGetParaConfFile(pcParameter,pcValue);

  if (iResult > 0)
  {
    *iValue = atoi(pcValue);
  }

  return iResult;
}

int iCommInit(const unsigned int uiMode, const char* name, const int iNonBlocking)
{
  struct mq_attr attr;
  int iResult;
  int iOFlag;
  unsigned int uiIndex;

  attr.mq_maxmsg = MQ_MAX_MSG;
  attr.mq_msgsize = MQ_MAX_MESSAGE_LENGTH;
  attr.mq_flags = 0;
  attr.mq_curmsgs = 0;

  tMQRecv = 0;

  for(uiIndex=0;uiIndex < MQ_NBR_QUEUES;++uiIndex)
  {
    ptMQSend[uiIndex] = 0;
  }

  strcpy(pcMessageQueueName,name);

  if(uiMode & IPC_RECV)
  {
    iOFlag = O_RDONLY | O_CREAT;
    if(iNonBlocking)
    {
      iOFlag |= O_NONBLOCK;
    }

    tMQRecv = mq_open(name, iOFlag, MQ_PERMISSION, &attr);
    if(tMQRecv < 0)
    {
      util_error("ERR: Failed to open receiving message queue");
    }
  }

  if(uiMode & IPC_SEND)
  {
    uiIndex = 0;

    if(strcmp(name,MQ_LG))
    {
      ptMQSend[uiIndex] = mq_open(MQ_LG, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_LG message queue");
      }
      ++uiIndex;
    }

    if(strcmp(name,MQ_OC))
    {
      ptMQSend[uiIndex] = mq_open(MQ_OC, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_OC message queue");
      }
      ++uiIndex;
    }

    if(strcmp(name,MQ_SV))
    {
      ptMQSend[uiIndex] = mq_open(MQ_SV, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_SV message queue");
      }
      ++uiIndex;
    }

    if(strcmp(name,MQ_VA))
    {
      ptMQSend[uiIndex] = mq_open(MQ_VA, O_WRONLY | O_NONBLOCK | O_CREAT, MQ_PERMISSION, &attr);
      if(ptMQSend[uiIndex] < 0)
      {
        util_error("ERR: Failed to open MQ_VA message queue");
      }
      ++uiIndex;
    }
  }

  return 1;
}

int iCommClose()
{
  int iIndex = 0;
  int iResult;

  if(tMQRecv != 0 && pcMessageQueueName != NULL)
  {
    iResult = mq_unlink(pcMessageQueueName);
    if(iResult < 0)
    {
      return 0;
    }
    iResult = mq_close(tMQRecv);
    if(iResult < 0)
    {
      return 0;
    }
  }

  for(iIndex = 0; iIndex < MQ_NBR_QUEUES; ++iIndex)
  {
    if(ptMQSend[iIndex] != 0)
    {
      iResult = mq_close(ptMQSend[iIndex]);
      if(iResult < 0)
      {
        return 0;
      }
    }
  }

  return 1;
}

int iCommRecv(int* iCommand, char* cpData, const int iMessageSize)
{
  int iResult;
  char cpMessage[MQ_MAX_MESSAGE_LENGTH];
  unsigned int prio;
  
  bzero(cpMessage,MQ_MAX_MESSAGE_LENGTH);

  iResult = mq_receive(tMQRecv, cpMessage, MQ_MAX_MESSAGE_LENGTH, &prio);
    
  if(iResult < 0 && errno != EAGAIN)
  {
    util_error ("ERR: Error when recieveing");
  }
  else if((iResult >= 0))
  {
    *iCommand = cpMessage[0];
    if((strlen(cpMessage) > 1) && (cpData != NULL))
    {
      if(iMessageSize <  iResult )
      {
        iResult = iMessageSize;
      }
      (void)strncat(cpData,&cpMessage[1],iResult);
    }
  }
  else
  {
    *iCommand = COMM_INV;
    iResult = 0;
  }

  return iResult;
}

int iCommSend(const int iCommand,const char* cpData)
{
  int iResult;
  unsigned int uiMessagePrio = 0;
  int iIndex = 0;
  char cpMessage[MQ_MAX_MESSAGE_LENGTH];

  bzero(cpMessage,MQ_MAX_MESSAGE_LENGTH);

  if(iCommand == COMM_TRIG)
  {
    uiMessagePrio = 100;
    cpMessage[0] = (char)COMM_TRIG;
  }
  else if(iCommand == COMM_STOP)
  {
    uiMessagePrio = 120;
    cpMessage[0] = (char)COMM_STOP;
  }
  else if(iCommand == COMM_MONI)
  {
    uiMessagePrio = 80;
    cpMessage[0] = (char)COMM_MONI;
  }
  else if(iCommand == COMM_EXIT)
  {
    uiMessagePrio = 140;
    cpMessage[0] = (char)COMM_EXIT;
  }
  else
  {
    util_error("ERR: Unknown command");
  }

  if(cpData != NULL)
  {
    (void)strncat(&cpMessage[1],cpData,strlen(cpData));
  }

  for(iIndex = 0; iIndex < MQ_NBR_QUEUES; ++iIndex)
  {
    if(ptMQSend[iIndex] != 0)
    {
      iResult = mq_send(ptMQSend[iIndex],cpMessage,strlen(cpMessage),uiMessagePrio);
      if(iResult < 0)
      {
        return 0;
      }
    }
  }
  return 1;
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/
