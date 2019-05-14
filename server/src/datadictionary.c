/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : datadictionary.c
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
#include "datadictionary.h"
#include "logging.h"


#define DD_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define DD_CONTROL_BUFFER_SIZE_20 20
#define DD_CONTROL_BUFFER_SIZE_52 52
#define DD_CONTROL_TASK_PERIOD_MS 1

// Parameters and variables
volatile U32 DataDictionaryExternalSupervisorIPU32 = 0; // The IP address of the external supervisor

I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/
void DataDictionaryConstructor(void)
{
  DataDictionaryInitExternalSupervisorIPU32();
}


/*External Supervisor IP*/
void DataDictionaryInitExternalSupervisorIPU32(void)
{
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(DataDictionarySearchParameter("SupervisorIP=", ResultBufferC8))
  {
    DataDictionaryExternalSupervisorIPU32 = UtilIPStringToInt(ResultBufferC8);
    LogMessage(LOG_LEVEL_INFO,"Supervisor IP: %s", ResultBufferC8);
  } else LogMessage(LOG_LEVEL_INFO,"Supervisor IP not found!");
}
void DataDictionarySetExternalSupervisorIPU32(U32 IP){  DataDictionaryExternalSupervisorIPU32 = IP;}
U32 DataDictionaryGetExternalSupervisorIPU32(void){  return DataDictionaryExternalSupervisorIPU32; }
/*end*/



I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer)
{
  bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(DD_CONTROL_CONF_FILE_PATH, ParameterName, "", ResultBuffer);
  return strlen(ResultBuffer);
}