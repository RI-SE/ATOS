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
#include <stdlib.h>  
#include <pthread.h>

#include "datadictionary.h"
#include "logging.h"

// Parameters and variables
static pthread_mutex_t OriginLatitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t OriginLongitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t OriginAltitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t VisualizationServerMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ForceObjectToLocalhostMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxTimeDiffMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxTrajDiffMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPStepBackCountMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPFilterLevelMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxDeltaTimeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t TimeServerIPMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t TimeServerPortMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t SimulatorIPMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t SimulatorTCPPortMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t SimulatorUDPPortMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t SimulatorModeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t VOILReceiversMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t DTMReceiversMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ExternalSupervisorIPMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t SupervisorTCPPortMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t DataDictionaryRVSSConfigMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t DataDictionaryRVSSRateMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPDataMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t MiscDataMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t OBCStateMutex = PTHREAD_MUTEX_INITIALIZER;




I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/


/*
DataDictionaryConstructor initialize data hold by DataDictionary.
Initialization data that is configurable is stored in test.conf. 

- If return data (I32) not is READ_OK is a parameter missing 
- *GSD pointer to global data
*/
I32 DataDictionaryConstructor(GSDType *GSD)
{
  I32 Res;

  Res = DataDictionaryInitOriginLatitudeDbl(GSD);
  if(Res == READ_OK) Res = DataDictionaryInitOriginLongitudeDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitOriginAltitudeDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitVisualizationServerU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitForceToLocalhostU8(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitASPMaxTimeDiffDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitASPMaxTrajDiffDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitASPStepBackCountU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitASPFilterLevelDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitASPMaxDeltaTimeDbl(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitTimeServerIPU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitTimeServerPortU16(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitSimulatorIPU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitSimulatorTCPPortU16(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitSimulatorUDPPortU16(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitSimulatorModeU8(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitVOILReceiversC8(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitDTMReceiversC8(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitExternalSupervisorIPU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitRVSSConfigU32(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitRVSSRateU8(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitSupervisorTCPPortU16(GSD);
  else if(Res == READ_OK) Res = DataDictionaryInitMiscDataC8(GSD);

  DataDictionarySetOBCStateU8(GSD, OBC_STATE_UNDEFINED);
  
  return Res;
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
    pthread_mutex_lock(&OriginLatitudeMutex);
    GSD->OriginLatitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLatitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&OriginLatitudeMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"OriginLatitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLatitudeDbl(GSDType *GSD, C8 *Latitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoLatidude", Latitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&OriginLatitudeMutex);
    GSD->OriginLatitudeDbl = atof(Latitude);
    bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLatitudeC8, Latitude);
    pthread_mutex_unlock(&OriginLatitudeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLatitudeDbl(GSDType *GSD, dbl *Latitude)
{
  pthread_mutex_lock(&OriginLatitudeMutex);
  *Latitude = GSD->OriginLatitudeDbl;
  pthread_mutex_unlock(&OriginLatitudeMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginLatitudeC8(GSDType *GSD, C8 *Latitude)
{
  pthread_mutex_lock(&OriginLatitudeMutex);
  strcat(Latitude, GSD->OriginLatitudeC8);
  pthread_mutex_unlock(&OriginLatitudeMutex);
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
    pthread_mutex_lock(&OriginLongitudeMutex);
    GSD->OriginLongitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLongitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&OriginLongitudeMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"OriginLongitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLongitudeDbl(GSDType *GSD, C8 *Longitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoLongitude", Longitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&OriginLongitudeMutex);
    GSD->OriginLongitudeDbl = atof(Longitude);
    bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginLongitudeC8, Longitude);
    pthread_mutex_unlock(&OriginLongitudeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLongitudeDbl(GSDType *GSD, dbl *Longitude)
{
  pthread_mutex_lock(&OriginLongitudeMutex);
  *Longitude = GSD->OriginLongitudeDbl;
  pthread_mutex_unlock(&OriginLongitudeMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginLongitudeC8(GSDType *GSD, C8 *Longitude)
{
  pthread_mutex_lock(&OriginLongitudeMutex);
  strcat(Longitude, GSD->OriginLongitudeC8);
  pthread_mutex_unlock(&OriginLongitudeMutex);
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
    pthread_mutex_lock(&OriginAltitudeMutex);
    GSD->OriginAltitudeDbl = atof(ResultBufferC8);
    bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginAltitudeC8, ResultBufferC8);
    pthread_mutex_unlock(&OriginAltitudeMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"OriginAltitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginAltitudeDbl(GSDType *GSD, C8 *Altitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OrigoAltitude", Altitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&OriginAltitudeMutex);
    GSD->OriginAltitudeDbl = atof(Altitude);
    bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->OriginAltitudeC8, Altitude);
    pthread_mutex_unlock(&OriginAltitudeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginAltitudeDbl(GSDType *GSD, dbl *Altitude)
{
  pthread_mutex_lock(&OriginAltitudeMutex);
  *Altitude = GSD->OriginAltitudeDbl;
  pthread_mutex_unlock(&OriginAltitudeMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetOriginAltitudeC8(GSDType *GSD, C8 *Altitude)
{
  pthread_mutex_lock(&OriginAltitudeMutex);
  strcat(Altitude, GSD->OriginAltitudeC8);
  pthread_mutex_unlock(&OriginAltitudeMutex);
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
    pthread_mutex_lock(&VisualizationServerMutex);
    GSD->VisualizationServerU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->VisualizationServerC8, ResultBufferC8);
    pthread_mutex_unlock(&VisualizationServerMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"VisualizationServerName not found!");
  }

  return Res;
}

I32 DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VisualizationServerName", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&VisualizationServerMutex);
    GSD->VisualizationServerU32 = UtilIPStringToInt(IP);
    bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->VisualizationServerC8, IP);
    pthread_mutex_unlock(&VisualizationServerMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&VisualizationServerMutex);
  *IP = GSD->VisualizationServerU32;
  pthread_mutex_unlock(&VisualizationServerMutex);
  return (I32) READ_OK;
 }


I32 DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&VisualizationServerMutex);
  strcat(IP, GSD->VisualizationServerC8);
  pthread_mutex_unlock(&VisualizationServerMutex);
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
    pthread_mutex_lock(&ForceObjectToLocalhostMutex);
    GSD->ForceObjectToLocalhostU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ForceObjectToLocalhost not found!");
  }

  return Res;
}

I32 DataDictionarySetForceToLocalhostU8(GSDType *GSD, C8 *ForceLocalhost)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ForceObjectToLocalhost", ForceLocalhost, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ForceObjectToLocalhostMutex);
    GSD->ForceObjectToLocalhostU8 = atoi(ForceLocalhost);
    pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetForceToLocalhostU8(GSDType *GSD, U8 *ForceLocalhost)
{
  pthread_mutex_lock(&ForceObjectToLocalhostMutex);
  *ForceLocalhost = GSD->ForceObjectToLocalhostU8;
  pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
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
    pthread_mutex_lock(&ASPMaxTimeDiffMutex);
    GSD->ASPMaxTimeDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ASPMaxTimeDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTimeDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTimeDiff", ASPMaxTimeDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxTimeDiffMutex);
    GSD->ASPMaxTimeDiffDbl = atof(ASPMaxTimeDiff);
    pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTimeDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff)
{
  pthread_mutex_lock(&ASPMaxTimeDiffMutex);
  *ASPMaxTimeDiff = GSD->ASPMaxTimeDiffDbl;
  pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
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
    pthread_mutex_lock(&ASPMaxTrajDiffMutex);
    GSD->ASPMaxTrajDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ASPMaxTrajDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTrajDiffDbl(GSDType *GSD, C8 *ASPMaxTrajDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTrajDiff", ASPMaxTrajDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxTrajDiffMutex);
    GSD->ASPMaxTrajDiffDbl = atof(ASPMaxTrajDiff);
    pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTrajDiffDbl(GSDType *GSD, dbl *ASPMaxTrajDiff)
{
  pthread_mutex_lock(&ASPMaxTrajDiffMutex);
  *ASPMaxTrajDiff = GSD->ASPMaxTrajDiffDbl;
  pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
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
    pthread_mutex_lock(&ASPStepBackCountMutex);
    GSD->ASPStepBackCountU32 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&ASPStepBackCountMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ASPStepBackCount not found!");
  }

  return Res;
}

I32 DataDictionarySetASPStepBackCountU32(GSDType *GSD, C8 *ASPStepBackCount)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPStepBackCount", ASPStepBackCount, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPStepBackCountMutex);
    GSD->ASPStepBackCountU32 = atoi(ASPStepBackCount);
    pthread_mutex_unlock(&ASPStepBackCountMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPStepBackCountU32(GSDType *GSD, U32 *ASPStepBackCount)
{
  pthread_mutex_lock(&ASPStepBackCountMutex);
  *ASPStepBackCount = GSD->ASPStepBackCountU32;
  pthread_mutex_unlock(&ASPStepBackCountMutex);
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
    pthread_mutex_lock(&ASPFilterLevelMutex);
    GSD->ASPFilterLevelDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&ASPFilterLevelMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ASPFilterLevel not found!");
  }

  return Res;
}

I32 DataDictionarySetASPFilterLevelDbl(GSDType *GSD, C8 *ASPFilterLevel)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPFilterLevel", ASPFilterLevel, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPFilterLevelMutex);
    GSD->ASPFilterLevelDbl = atof(ASPFilterLevel);
    pthread_mutex_unlock(&ASPFilterLevelMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPFilterLevelDbl(GSDType *GSD, dbl *ASPFilterLevel)
{
  pthread_mutex_lock(&ASPFilterLevelMutex);
  *ASPFilterLevel = GSD->ASPFilterLevelDbl;
  pthread_mutex_unlock(&ASPFilterLevelMutex);
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
    pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
    GSD->ASPMaxDeltaTimeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"ASPMaxDeltaTime not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxDeltaTimeDbl(GSDType *GSD, C8 *ASPMaxDeltaTime)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxDeltaTime", ASPMaxDeltaTime, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
    GSD->ASPMaxDeltaTimeDbl = atof(ASPMaxDeltaTime);
    pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxDeltaTimeDbl(GSDType *GSD, dbl *ASPMaxDeltaTime)
{
  pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
  *ASPMaxDeltaTime = GSD->ASPMaxDeltaTimeDbl;
  pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
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
    pthread_mutex_lock(&TimeServerIPMutex);
    GSD->TimeServerIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->TimeServerIPC8, ResultBufferC8);
    pthread_mutex_unlock(&TimeServerIPMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"TimeServerIP not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerIPU32(GSDType *GSD, C8 *TimeServerIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerIP", TimeServerIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&TimeServerIPMutex);
    GSD->TimeServerIPU32 = UtilIPStringToInt(TimeServerIP);
    bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->TimeServerIPC8, TimeServerIP);
    pthread_mutex_unlock(&TimeServerIPMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerIPU32(GSDType *GSD, U32 *TimeServerIP)
{
  pthread_mutex_lock(&TimeServerIPMutex);
  *TimeServerIP = GSD->TimeServerIPU32;
  pthread_mutex_unlock(&TimeServerIPMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetTimeServerIPC8(GSDType *GSD, C8 *TimeServerIP)
{
  pthread_mutex_lock(&TimeServerIPMutex);
  strcat(TimeServerIP, GSD->TimeServerIPC8);
  pthread_mutex_unlock(&TimeServerIPMutex);
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
    pthread_mutex_lock(&TimeServerPortMutex);
    GSD->TimeServerPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&TimeServerPortMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"TimeServerPort not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerPortU16(GSDType *GSD, C8 *TimeServerPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerPort", TimeServerPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&TimeServerPortMutex);
    GSD->TimeServerPortU16 = atoi(TimeServerPort);
    pthread_mutex_unlock(&TimeServerPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerPortU16(GSDType *GSD, U16 *TimeServerPort)
{
  pthread_mutex_lock(&TimeServerPortMutex);
  *TimeServerPort = GSD->TimeServerPortU16;
  pthread_mutex_unlock(&TimeServerPortMutex);
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
    pthread_mutex_lock(&SimulatorIPMutex);
    GSD->SimulatorIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->SimulatorIPC8, ResultBufferC8);
    pthread_mutex_unlock(&SimulatorIPMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"SimulatorIP not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorIPU32(GSDType *GSD, C8 *SimulatorIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorIP", SimulatorIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorIPMutex);
    GSD->SimulatorIPU32 = UtilIPStringToInt(SimulatorIP);
    bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->SimulatorIPC8, SimulatorIP);
    pthread_mutex_unlock(&SimulatorIPMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorIPU32(GSDType *GSD, U32 *SimulatorIP)
{
  pthread_mutex_lock(&SimulatorIPMutex);
  *SimulatorIP = GSD->SimulatorIPU32;
  pthread_mutex_unlock(&SimulatorIPMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetSimulatorIPC8(GSDType *GSD, C8 *SimulatorIP)
{
  pthread_mutex_lock(&SimulatorIPMutex);
  strcat(SimulatorIP, GSD->SimulatorIPC8);
  pthread_mutex_unlock(&SimulatorIPMutex);
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
    pthread_mutex_lock(&SimulatorTCPPortMutex);
    GSD->SimulatorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&SimulatorTCPPortMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"SimulatorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorTCPPortU16(GSDType *GSD, C8 *SimulatorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorTCPPort", SimulatorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorTCPPortMutex);
    GSD->SimulatorTCPPortU16 = atoi(SimulatorTCPPort);
    pthread_mutex_unlock(&SimulatorTCPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorTCPPortU16(GSDType *GSD, U16 *SimulatorTCPPort)
{
  pthread_mutex_lock(&SimulatorTCPPortMutex);
  *SimulatorTCPPort = GSD->SimulatorTCPPortU16;
  pthread_mutex_unlock(&SimulatorTCPPortMutex);
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
    pthread_mutex_lock(&SimulatorUDPPortMutex);
    GSD->SimulatorUDPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&SimulatorUDPPortMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"SimulatorUDPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorUDPPortU16(GSDType *GSD, C8 *SimulatorUDPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorUDPPort", SimulatorUDPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorUDPPortMutex);
    GSD->SimulatorUDPPortU16 = atoi(SimulatorUDPPort);
    pthread_mutex_unlock(&SimulatorUDPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorUDPPortU16(GSDType *GSD, U16 *SimulatorUDPPort)
{
  pthread_mutex_lock(&SimulatorUDPPortMutex);
  *SimulatorUDPPort = GSD->SimulatorUDPPortU16;
  pthread_mutex_unlock(&SimulatorUDPPortMutex);
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
    pthread_mutex_lock(&SimulatorModeMutex);
    GSD->SimulatorModeU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&SimulatorModeMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"SimulatorMode not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorModeU8(GSDType *GSD, C8 *SimulatorMode)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorMode", SimulatorMode, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorModeMutex);
    GSD->SimulatorModeU8 = atoi(SimulatorMode);
    pthread_mutex_unlock(&SimulatorModeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorModeU8(GSDType *GSD, U8 *SimulatorMode)
{
  pthread_mutex_lock(&SimulatorModeMutex);
  *SimulatorMode = GSD->SimulatorModeU8;
  pthread_mutex_unlock(&SimulatorModeMutex);
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
    pthread_mutex_lock(&VOILReceiversMutex);
    strcpy(GSD->VOILReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&VOILReceiversMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"VOILReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VOILReceivers", VOILReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&VOILReceiversMutex);
    strcpy(GSD->VOILReceiversC8, VOILReceivers);
    pthread_mutex_unlock(&VOILReceiversMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVOILReceiversC8(GSDType *GSD, U8 *VOILReceivers)
{
  pthread_mutex_lock(&VOILReceiversMutex);
  strcpy(VOILReceivers, GSD->VOILReceiversC8);
  pthread_mutex_unlock(&VOILReceiversMutex);
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
    pthread_mutex_lock(&DTMReceiversMutex);
    strcpy(GSD->DTMReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&DTMReceiversMutex);
  }
else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"DTMReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("DTMReceivers", DTMReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&DTMReceiversMutex);
    strcpy(GSD->DTMReceiversC8, DTMReceivers);
    pthread_mutex_unlock(&DTMReceiversMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetDTMReceiversC8(GSDType *GSD, U8 *DTMReceivers)
{
  pthread_mutex_lock(&DTMReceiversMutex);
  strcpy(DTMReceivers, GSD->DTMReceiversC8);
  pthread_mutex_unlock(&DTMReceiversMutex);
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
    pthread_mutex_lock(&ExternalSupervisorIPMutex);
    GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(ResultBufferC8);
    bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->ExternalSupervisorIPC8, ResultBufferC8);
    pthread_mutex_unlock(&ExternalSupervisorIPMutex);
    //LogMessage(LOG_LEVEL_ERROR,"Supervisor IP: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"Supervisor IP not found!");
  }

  return Res;
}

I32 DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorIP", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ExternalSupervisorIPMutex);
    GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(IP);
    bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
    strcat(GSD->ExternalSupervisorIPC8, IP);
    pthread_mutex_unlock(&ExternalSupervisorIPMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&ExternalSupervisorIPMutex);
  *IP = GSD->ExternalSupervisorIPU32;
  pthread_mutex_unlock(&ExternalSupervisorIPMutex);
  return (I32) READ_OK;
 }

I32 DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&ExternalSupervisorIPMutex);
  strcat(IP, GSD->ExternalSupervisorIPC8);
  pthread_mutex_unlock(&ExternalSupervisorIPMutex);
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
    pthread_mutex_lock(&SupervisorTCPPortMutex);
    GSD->SupervisorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&SupervisorTCPPortMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"SupervisorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorTCPPort", SupervisorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SupervisorTCPPortMutex);
    GSD->SupervisorTCPPortU16 = atoi(SupervisorTCPPort);
    pthread_mutex_unlock(&SupervisorTCPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort)
{
  pthread_mutex_lock(&SupervisorTCPPortMutex);
  *SupervisorTCPPort = GSD->SupervisorTCPPortU16;
  pthread_mutex_unlock(&SupervisorTCPPortMutex);
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
    pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
    GSD->DataDictionaryRVSSConfigU32 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
    //LogMessage(LOG_LEVEL_ERROR,"RVSSConfig: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"RVSSConfig not found!");
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
    pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
    GSD->DataDictionaryRVSSConfigU32 = RVSSConfig;
    pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSConfigU32(GSDType *GSD, U32 *RVSSConfig)
{
  pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
  *RVSSConfig = GSD->DataDictionaryRVSSConfigU32;
  pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
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
    pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
    GSD->DataDictionaryRVSSRateU8 = (U8)atoi(ResultBufferC8);
    pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
    //LogMessage(LOG_LEVEL_ERROR,"RVSSRate: %s", ResultBufferC8);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"RVSSRate not found!");
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
    pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
    GSD->DataDictionaryRVSSRateU8 = RVSSRate;
    pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSRateU8(GSDType *GSD, U8 *RVSSRate)
{
  pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
  *RVSSRate = GSD->DataDictionaryRVSSRateU8;
  pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
  return (I32) READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Rate**/


/*ASPDebug*/
I32 DataDictionarySetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&ASPDataMutex);
  GSD->ASPData = *ASPD;
  pthread_mutex_unlock(&ASPDataMutex);
  return (I32) WRITE_OK;
}

I32 DataDictionaryGetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&ASPDataMutex);
  *ASPD = GSD->ASPData;
  pthread_mutex_unlock(&ASPDataMutex);
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
    pthread_mutex_lock(&MiscDataMutex);
    strcpy(GSD->MiscDataC8, ResultBufferC8);
    pthread_mutex_unlock(&MiscDataMutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_ERROR,"MiscData not found!");
  }

  return Res;
}

I32 DataDictionarySetMiscDataC8(GSDType *GSD, C8 *MiscData)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("MiscData", MiscData, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&MiscDataMutex);
    bzero(GSD->MiscDataC8, DD_CONTROL_BUFFER_SIZE_1024);
    strcpy(GSD->MiscDataC8, MiscData);
    pthread_mutex_unlock(&MiscDataMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetMiscDataC8(GSDType *GSD, U8 *MiscData)
{
  pthread_mutex_lock(&MiscDataMutex);
  strcpy(MiscData, GSD->MiscDataC8);
  pthread_mutex_unlock(&MiscDataMutex);
  return (I32) READ_OK;
 }
/*END of MiscData*/


/*OBCState*/
I32 DataDictionarySetOBCStateU8(GSDType *GSD, U8 OBCState)
{
  I32 Res;
  Res = WRITE_OK;
  pthread_mutex_lock(&OBCStateMutex);
  GSD->OBCStateU8 = OBCState;
  pthread_mutex_unlock(&OBCStateMutex);
  return Res; 
}

U8 DataDictionaryGetOBCStateU8(GSDType *GSD)
{
  U8 Ret;
  pthread_mutex_lock(&OBCStateMutex);
  Ret = GSD->OBCStateU8;
  pthread_mutex_unlock(&OBCStateMutex);

  return Ret;
 }
/*END OBCState*/


/*
DataDictionarySearchParameter searches for parameters in the file test.conf and returns
the parameter value.

- *ParameterName the name of the parameter in test.conf
- *ResultBuffer the value of the parameter.
*/
I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer)
{
  bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, ParameterName, "", ResultBuffer);
  return strlen(ResultBuffer);
}
