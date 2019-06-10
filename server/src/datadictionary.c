/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
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
#include <pthread.h>

#include "util.h"
#include "logger.h"
#include "datadictionary.h"
#include "logging.h"


#define DD_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define DD_CONTROL_BUFFER_SIZE_1024 1024
#define DD_CONTROL_BUFFER_SIZE_20 20
#define DD_CONTROL_BUFFER_SIZE_52 52
#define DD_CONTROL_TASK_PERIOD_MS 1

// Parameters and variables
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/
void DataDictionaryConstructor(GSDType *GSD)
{
  DataDictionaryInitOriginLatitudeDbl(GSD);
  DataDictionaryInitOriginLongitudeDbl(GSD);
  DataDictionaryInitOriginAltitudeDbl(GSD);
  DataDictionaryInitVisualizationServerU32(GSD);
  DataDictionaryInitForceToLocalhostU8(GSD);
  DataDictionaryInitASPMaxTimeDiffDbl(GSD);
  DataDictionaryInitASPMaxTrajDiffDbl(GSD);
  DataDictionaryInitASPStepBackCountU32(GSD);
  DataDictionaryInitASPFilterLevelDbl(GSD);
  DataDictionaryInitASPMaxDeltaTimeDbl(GSD);
  DataDictionaryInitTimeServerIPU32(GSD);
  DataDictionaryInitTimeServerPortU16(GSD);
  DataDictionaryInitSimulatorIPU32(GSD);
  DataDictionaryInitSimulatorTCPPortU16(GSD);
  DataDictionaryInitSimulatorUDPPortU16(GSD);
  DataDictionaryInitSimulatorModeU8(GSD);
  DataDictionaryInitVOILReceiversC8(GSD);
  DataDictionaryInitDTMReceiversC8(GSD);
  DataDictionaryInitExternalSupervisorIPU32(GSD);
  DataDictionaryInitRVSSConfigU32(GSD);
  DataDictionaryInitRVSSRateU8(GSD);
  DataDictionaryInitSupervisorTCPPortU16(GSD);
  DataDictionaryInitMiscDataC8(GSD);
  DataDictionarySetOBCStateU8(GSD, OBC_STATE_IDLE);
}

//  I32 Return
//      1     = WRITE_OK
//      2     = READ_OK
//      3     = READ_WRITE_OK
//      4     = PARAMETER_NOTFOUND
//      5     = OUT_OF_RANGE


/*Origin Latitude*/
I32 DataDictionaryInitOriginLatitudeDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OrigoLatidude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginLatitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLatitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginLatitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLatitudeDbl(GSDType *GSD, C8 *Latitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoLatidude", Latitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginLatitudeDbl = atof(Latitude);
    bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLatitudeC8, Latitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLatitudeDbl(GSDType *GSD, dbl *Latitude)
{
  pthread_mutex_lock(&mutex);
  *Latitude = GSD->OriginLatitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginLatitudeC8(GSDType *GSD, C8 *Latitude)
{
  pthread_mutex_lock(&mutex);
  strcat(Latitude, GSD->OriginLatitudeC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

/*END of Origin Latitude*/

/*Origin Longitude*/
I32 DataDictionaryInitOriginLongitudeDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OrigoLongitude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginLongitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLongitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginLongitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLongitudeDbl(GSDType *GSD, C8 *Longitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoLongitude", Longitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginLongitudeDbl = atof(Longitude);
    bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLongitudeC8, Longitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLongitudeDbl(GSDType *GSD, dbl *Longitude)
{
  pthread_mutex_lock(&mutex);
  *Longitude = GSD->OriginLongitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginLongitudeC8(GSDType *GSD, C8 *Longitude)
{
  pthread_mutex_lock(&mutex);
  strcat(Longitude, GSD->OriginLongitudeC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Origin Longitude*/

/*Origin Altitude*/
I32 DataDictionaryInitOriginAltitudeDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OrigoAltitude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginAltitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginAltitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginAltitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginAltitudeDbl(GSDType *GSD, C8 *Altitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoAltitude", Altitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->OriginAltitudeDbl = atof(Altitude);
    bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginAltitudeC8, Altitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginAltitudeDbl(GSDType *GSD, dbl *Altitude)
{
  pthread_mutex_lock(&mutex);
  *Altitude = GSD->OriginAltitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginAltitudeC8(GSDType *GSD, C8 *Altitude)
{
  pthread_mutex_lock(&mutex);
  strcat(Altitude, GSD->OriginAltitudeC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

/*END of Origin Altitude*/

/*VisualizationServer*/
I32 DataDictionaryInitVisualizationServerU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("VisualizationServerName=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->VisualizationServerU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->VisualizationServerC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"VisualizationServerName not found!");
  }

  return Res;
}

I32 DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VisualizationServerName", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->VisualizationServerU32 = UtilIPStringToInt(IP);
    bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->VisualizationServerC8, IP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&mutex);
  *IP = GSD->VisualizationServerU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }


I32 DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&mutex);
  strcat(IP, GSD->VisualizationServerC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
}

/*END of VisualizationServer*/


/*ForceToLocalhost*/
I32 DataDictionaryInitForceToLocalhostU8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ForceObjectToLocalhost=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ForceObjectToLocalhostU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ForceObjectToLocalhost not found!");
  }

  return Res;
}

I32 DataDictionarySetForceToLocalhostU8(GSDType *GSD, C8 *ForceLocalhost)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ForceObjectToLocalhost", ForceLocalhost, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ForceObjectToLocalhostU8 = atoi(ForceLocalhost);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetForceToLocalhostU8(GSDType *GSD, U8 *ForceLocalhost)
{
  pthread_mutex_lock(&mutex);
  *ForceLocalhost = GSD->ForceObjectToLocalhostU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ForceToLocalhost*/

/*ASPMaxTimeDiff*/
I32 DataDictionaryInitASPMaxTimeDiffDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxTimeDiff=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxTimeDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxTimeDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTimeDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTimeDiff", ASPMaxTimeDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxTimeDiffDbl = atof(ASPMaxTimeDiff);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTimeDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxTimeDiff = GSD->ASPMaxTimeDiffDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPMaxTimeDiff*/

/*ASPMaxTrajDiff*/
I32 DataDictionaryInitASPMaxTrajDiffDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxTrajDiff=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxTrajDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxTrajDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTrajDiffDbl(GSDType *GSD, C8 *ASPMaxTrajDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTrajDiff", ASPMaxTrajDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxTrajDiffDbl = atof(ASPMaxTrajDiff);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTrajDiffDbl(GSDType *GSD, dbl *ASPMaxTrajDiff)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxTrajDiff = GSD->ASPMaxTrajDiffDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPMaxTrajDiff*/


/*ASPStepBackCount*/
I32 DataDictionaryInitASPStepBackCountU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPStepBackCount=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPStepBackCountU32 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPStepBackCount not found!");
  }

  return Res;
}

I32 DataDictionarySetASPStepBackCountU32(GSDType *GSD, C8 *ASPStepBackCount)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPStepBackCount", ASPStepBackCount, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPStepBackCountU32 = atoi(ASPStepBackCount);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPStepBackCountU32(GSDType *GSD, U32 *ASPStepBackCount)
{
  pthread_mutex_lock(&mutex);
  *ASPStepBackCount = GSD->ASPStepBackCountU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPStepBackCount*/

/*ASPFilterLevel*/
I32 DataDictionaryInitASPFilterLevelDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPFilterLevel=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPFilterLevelDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPFilterLevel not found!");
  }

  return Res;
}

I32 DataDictionarySetASPFilterLevelDbl(GSDType *GSD, C8 *ASPFilterLevel)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPFilterLevel", ASPFilterLevel, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPFilterLevelDbl = atof(ASPFilterLevel);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPFilterLevelDbl(GSDType *GSD, dbl *ASPFilterLevel)
{
  pthread_mutex_lock(&mutex);
  *ASPFilterLevel = GSD->ASPFilterLevelDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPFilterLevel*/

/*ASPMaxDeltaTime*/
I32 DataDictionaryInitASPMaxDeltaTimeDbl(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxDeltaTime=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxDeltaTimeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxDeltaTime not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxDeltaTimeDbl(GSDType *GSD, C8 *ASPMaxDeltaTime)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxDeltaTime", ASPMaxDeltaTime, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ASPMaxDeltaTimeDbl = atof(ASPMaxDeltaTime);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxDeltaTimeDbl(GSDType *GSD, dbl *ASPMaxDeltaTime)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxDeltaTime = GSD->ASPMaxDeltaTimeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPFilterLevel*/


/*TimeServerIP*/
I32 DataDictionaryInitTimeServerIPU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("TimeServerIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->TimeServerIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->TimeServerIPC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"TimeServerIP not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerIPU32(GSDType *GSD, C8 *TimeServerIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerIP", TimeServerIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->TimeServerIPU32 = UtilIPStringToInt(TimeServerIP);
    bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->TimeServerIPC8, TimeServerIP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerIPU32(GSDType *GSD, U32 *TimeServerIP)
{
  pthread_mutex_lock(&mutex);
  *TimeServerIP = GSD->TimeServerIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetTimeServerIPC8(GSDType *GSD, C8 *TimeServerIP)
{
  pthread_mutex_lock(&mutex);
  strcat(TimeServerIP, GSD->TimeServerIPC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

/*END of TimeServerIP*/


/*TimeServerPort*/
I32 DataDictionaryInitTimeServerPortU16(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("TimeServerPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->TimeServerPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"TimeServerPort not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerPortU16(GSDType *GSD, C8 *TimeServerPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerPort", TimeServerPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->TimeServerPortU16 = atoi(TimeServerPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerPortU16(GSDType *GSD, U16 *TimeServerPort)
{
  pthread_mutex_lock(&mutex);
  *TimeServerPort = GSD->TimeServerPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of TimeServerPort*/


/*SimulatorIP*/
I32 DataDictionaryInitSimulatorIPU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->SimulatorIPC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorIP not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorIPU32(GSDType *GSD, C8 *SimulatorIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorIP", SimulatorIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorIPU32 = UtilIPStringToInt(SimulatorIP);
    bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->SimulatorIPC8, SimulatorIP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorIPU32(GSDType *GSD, U32 *SimulatorIP)
{
  pthread_mutex_lock(&mutex);
  *SimulatorIP = GSD->SimulatorIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetSimulatorIPC8(GSDType *GSD, C8 *SimulatorIP)
{
  pthread_mutex_lock(&mutex);
  strcat(SimulatorIP, GSD->SimulatorIPC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorIP*/

/*SimulatorTCPPort*/
I32 DataDictionaryInitSimulatorTCPPortU16(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorTCPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorTCPPortU16(GSDType *GSD, C8 *SimulatorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorTCPPort", SimulatorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorTCPPortU16 = atoi(SimulatorTCPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorTCPPortU16(GSDType *GSD, U16 *SimulatorTCPPort)
{
  pthread_mutex_lock(&mutex);
  *SimulatorTCPPort = GSD->SimulatorTCPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorTCPPort*/

/*SimulatorUDPPort*/
I32 DataDictionaryInitSimulatorUDPPortU16(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorUDPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorUDPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorUDPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorUDPPortU16(GSDType *GSD, C8 *SimulatorUDPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorUDPPort", SimulatorUDPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorUDPPortU16 = atoi(SimulatorUDPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorUDPPortU16(GSDType *GSD, U16 *SimulatorUDPPort)
{
  pthread_mutex_lock(&mutex);
  *SimulatorUDPPort = GSD->SimulatorUDPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorUDPPort*/

/*SimulatorMode*/
I32 DataDictionaryInitSimulatorModeU8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorMode=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorModeU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorMode not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorModeU8(GSDType *GSD, C8 *SimulatorMode)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorMode", SimulatorMode, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->SimulatorModeU8 = atoi(SimulatorMode);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorModeU8(GSDType *GSD, U8 *SimulatorMode)
{
  pthread_mutex_lock(&mutex);
  *SimulatorMode = GSD->SimulatorModeU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorMode*/

/*VOILReceivers*/
I32 DataDictionaryInitVOILReceiversC8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_1024];
  if(Res = DataDictionarySearchParameter("VOILReceivers=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(GSD->VOILReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"VOILReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VOILReceivers", VOILReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    strcpy(GSD->VOILReceiversC8, VOILReceivers);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVOILReceiversC8(GSDType *GSD, U8 *VOILReceivers)
{
  pthread_mutex_lock(&mutex);
  strcpy(VOILReceivers, GSD->VOILReceiversC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of VOILReceivers*/

/*DTMReceivers*/
I32 DataDictionaryInitDTMReceiversC8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_1024];
  if(Res = DataDictionarySearchParameter("DTMReceivers=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(GSD->DTMReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"DTMReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("DTMReceivers", DTMReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    strcpy(GSD->DTMReceiversC8, DTMReceivers);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetDTMReceiversC8(GSDType *GSD, U8 *DTMReceivers)
{
  pthread_mutex_lock(&mutex);
  strcpy(DTMReceivers, GSD->DTMReceiversC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of DTMReceivers*/

/*External Supervisor IP*/
I32 DataDictionaryInitExternalSupervisorIPU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SupervisorIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->ExternalSupervisorIPC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
    //LogMessage(LOG_LEVEL_INFO,"Supervisor IP: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"Supervisor IP not found!");
  }

  return Res;
}

I32 DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorIP", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(IP);
    bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->ExternalSupervisorIPC8, IP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&mutex);
  *IP = GSD->ExternalSupervisorIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&mutex);
  strcat(IP, GSD->ExternalSupervisorIPC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of External Supervisor IP*/

/*External SupervisorTCPPort*/
I32 DataDictionaryInitSupervisorTCPPortU16(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SupervisorTCPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->SupervisorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SupervisorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorTCPPort", SupervisorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->SupervisorTCPPortU16 = atoi(SupervisorTCPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort)
{
  pthread_mutex_lock(&mutex);
  *SupervisorTCPPort = GSD->SupervisorTCPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of External SupervisorTCPPort*/




/*Runtime Variable Subscription Service (RVSS) Configuration*/
I32 DataDictionaryInitRVSSConfigU32(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("RVSSConfig=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->DataDictionaryRVSSConfigU32 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
    //LogMessage(LOG_LEVEL_INFO,"RVSSConfig: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"RVSSConfig not found!");
  }

  return Res;
}

I32 DataDictionarySetRVSSConfigU32(GSDType *GSD, U32 RVSSConfig)
{
  I32 Res;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
  sprintf(ResultBufferC8, "%" PRIu32, RVSSConfig);

  if(Res = UtilWriteConfigurationParameter("RVSSConfig", ResultBufferC8, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->DataDictionaryRVSSConfigU32 = RVSSConfig;
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSConfigU32(GSDType *GSD, U32 *RVSSConfig)
{
  pthread_mutex_lock(&mutex);
  *RVSSConfig = GSD->DataDictionaryRVSSConfigU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Configuration**/


/*Runtime Variable Subscription Service (RVSS) Rate*/
I32 DataDictionaryInitRVSSRateU8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("RVSSRate=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    GSD->DataDictionaryRVSSRateU8 = (U8)atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
    //LogMessage(LOG_LEVEL_INFO,"RVSSRate: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"RVSSRate not found!");
  }

  return Res;
}

I32 DataDictionarySetRVSSRateU8(GSDType *GSD, U8 RVSSRate)
{
  I32 Res;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
  sprintf(ResultBufferC8, "%" PRIu8, RVSSRate);

  if(Res = UtilWriteConfigurationParameter("RVSSRate", ResultBufferC8, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    GSD->DataDictionaryRVSSRateU8 = RVSSRate;
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSRateU8(GSDType *GSD, U8 *RVSSRate)
{
  pthread_mutex_lock(&mutex);
  *RVSSRate = GSD->DataDictionaryRVSSRateU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Rate**/


/*ASPDebug*/
I32 DataDictionarySetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&mutex);
  GSD->ASPData = *ASPD;
  pthread_mutex_unlock(&mutex);
  return (I32) WRITE_OK;
}

I32 DataDictionaryGetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&mutex);
  *ASPD = GSD->ASPData;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
}
/*END ASPDebug*/

/*MiscData*/
I32 DataDictionaryInitMiscDataC8(GSDType *GSD)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("MiscData=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(GSD->MiscDataC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"MiscData not found!");
  }

  return Res;
}

I32 DataDictionarySetMiscDataC8(GSDType *GSD, C8 *MiscData)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("MiscData", MiscData, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    bzero(GSD->MiscDataC8, DD_CONTROL_BUFFER_SIZE_1024);
    strcpy(GSD->MiscDataC8, MiscData);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetMiscDataC8(GSDType *GSD, U8 *MiscData)
{
  pthread_mutex_lock(&mutex);
  strcpy(MiscData, GSD->MiscDataC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of MiscData*/


/*OBCState*/
I32 DataDictionarySetOBCStateU8(GSDType *GSD, U8 OBCState)
{
  I32 Res;
  Res = WRITE_OK;
  pthread_mutex_lock(&mutex);
  GSD->OBCStateU8 = OBCState;
  pthread_mutex_unlock(&mutex);
  return Res; 
}

U8 DataDictionaryGetOBCStateU8(GSDType *GSD)
{
  return GSD->OBCStateU8;
 }
/*END OBCState*/

I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer)
{
  bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(DD_CONTROL_CONF_FILE_PATH, ParameterName, "", ResultBuffer);
  return strlen(ResultBuffer);
}