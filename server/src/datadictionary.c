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




U64 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/


/*!
 * \brief DataDictionaryConstructor Initialize data held by DataDictionary.
Initialization data that is configurable is stored in test.conf.
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryConstructor(GSDType *GSD)
{
  ReadWriteAccess_t Res;

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


/*Origin Latitude*/
ReadWriteAccess_t DataDictionaryInitOriginLatitudeDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetOriginLatitudeDbl(GSDType *GSD, C8 *Latitude)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetOriginLatitudeDbl(GSDType *GSD, dbl *Latitude)
{
  pthread_mutex_lock(&OriginLatitudeMutex);
  *Latitude = GSD->OriginLatitudeDbl;
  pthread_mutex_unlock(&OriginLatitudeMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetOriginLatitudeC8(GSDType *GSD, C8 *Latitude)
{
  pthread_mutex_lock(&OriginLatitudeMutex);
  strcat(Latitude, GSD->OriginLatitudeC8);
  pthread_mutex_unlock(&OriginLatitudeMutex);
  return READ_OK;
 }

/*END of Origin Latitude*/

/*Origin Longitude*/
ReadWriteAccess_t DataDictionaryInitOriginLongitudeDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetOriginLongitudeDbl(GSDType *GSD, C8 *Longitude)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetOriginLongitudeDbl(GSDType *GSD, dbl *Longitude)
{
  pthread_mutex_lock(&OriginLongitudeMutex);
  *Longitude = GSD->OriginLongitudeDbl;
  pthread_mutex_unlock(&OriginLongitudeMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetOriginLongitudeC8(GSDType *GSD, C8 *Longitude)
{
  pthread_mutex_lock(&OriginLongitudeMutex);
  strcat(Longitude, GSD->OriginLongitudeC8);
  pthread_mutex_unlock(&OriginLongitudeMutex);
  return READ_OK;
 }
/*END of Origin Longitude*/

/*Origin Altitude*/
ReadWriteAccess_t DataDictionaryInitOriginAltitudeDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetOriginAltitudeDbl(GSDType *GSD, C8 *Altitude)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetOriginAltitudeDbl(GSDType *GSD, dbl *Altitude)
{
  pthread_mutex_lock(&OriginAltitudeMutex);
  *Altitude = GSD->OriginAltitudeDbl;
  pthread_mutex_unlock(&OriginAltitudeMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetOriginAltitudeC8(GSDType *GSD, C8 *Altitude)
{
  pthread_mutex_lock(&OriginAltitudeMutex);
  strcat(Altitude, GSD->OriginAltitudeC8);
  pthread_mutex_unlock(&OriginAltitudeMutex);
  return READ_OK;
 }

/*END of Origin Altitude*/

/*VisualizationServer*/
ReadWriteAccess_t DataDictionaryInitVisualizationServerU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&VisualizationServerMutex);
  *IP = GSD->VisualizationServerU32;
  pthread_mutex_unlock(&VisualizationServerMutex);
  return READ_OK;
 }


ReadWriteAccess_t DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&VisualizationServerMutex);
  strcat(IP, GSD->VisualizationServerC8);
  pthread_mutex_unlock(&VisualizationServerMutex);
  return READ_OK;
}

/*END of VisualizationServer*/


/*ForceToLocalhost*/
ReadWriteAccess_t DataDictionaryInitForceToLocalhostU8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetForceToLocalhostU8(GSDType *GSD, C8 *ForceLocalhost)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ForceObjectToLocalhost", ForceLocalhost, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ForceObjectToLocalhostMutex);
    GSD->ForceObjectToLocalhostU8 = atoi(ForceLocalhost);
    pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetForceToLocalhostU8(GSDType *GSD, U8 *ForceLocalhost)
{
  pthread_mutex_lock(&ForceObjectToLocalhostMutex);
  *ForceLocalhost = GSD->ForceObjectToLocalhostU8;
  pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
  return READ_OK;
 }
/*END of ForceToLocalhost*/

/*ASPMaxTimeDiff*/
ReadWriteAccess_t DataDictionaryInitASPMaxTimeDiffDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetASPMaxTimeDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTimeDiff", ASPMaxTimeDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxTimeDiffMutex);
    GSD->ASPMaxTimeDiffDbl = atof(ASPMaxTimeDiff);
    pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetASPMaxTimeDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff)
{
  pthread_mutex_lock(&ASPMaxTimeDiffMutex);
  *ASPMaxTimeDiff = GSD->ASPMaxTimeDiffDbl;
  pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
  return READ_OK;
 }
/*END of ASPMaxTimeDiff*/

/*ASPMaxTrajDiff*/
ReadWriteAccess_t DataDictionaryInitASPMaxTrajDiffDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetASPMaxTrajDiffDbl(GSDType *GSD, C8 *ASPMaxTrajDiff)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTrajDiff", ASPMaxTrajDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxTrajDiffMutex);
    GSD->ASPMaxTrajDiffDbl = atof(ASPMaxTrajDiff);
    pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetASPMaxTrajDiffDbl(GSDType *GSD, dbl *ASPMaxTrajDiff)
{
  pthread_mutex_lock(&ASPMaxTrajDiffMutex);
  *ASPMaxTrajDiff = GSD->ASPMaxTrajDiffDbl;
  pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
  return READ_OK;
 }
/*END of ASPMaxTrajDiff*/


/*ASPStepBackCount*/
ReadWriteAccess_t DataDictionaryInitASPStepBackCountU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetASPStepBackCountU32(GSDType *GSD, C8 *ASPStepBackCount)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ASPStepBackCount", ASPStepBackCount, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPStepBackCountMutex);
    GSD->ASPStepBackCountU32 = atoi(ASPStepBackCount);
    pthread_mutex_unlock(&ASPStepBackCountMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetASPStepBackCountU32(GSDType *GSD, U32 *ASPStepBackCount)
{
  pthread_mutex_lock(&ASPStepBackCountMutex);
  *ASPStepBackCount = GSD->ASPStepBackCountU32;
  pthread_mutex_unlock(&ASPStepBackCountMutex);
  return READ_OK;
 }
/*END of ASPStepBackCount*/

/*ASPFilterLevel*/
ReadWriteAccess_t DataDictionaryInitASPFilterLevelDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetASPFilterLevelDbl(GSDType *GSD, C8 *ASPFilterLevel)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ASPFilterLevel", ASPFilterLevel, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPFilterLevelMutex);
    GSD->ASPFilterLevelDbl = atof(ASPFilterLevel);
    pthread_mutex_unlock(&ASPFilterLevelMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetASPFilterLevelDbl(GSDType *GSD, dbl *ASPFilterLevel)
{
  pthread_mutex_lock(&ASPFilterLevelMutex);
  *ASPFilterLevel = GSD->ASPFilterLevelDbl;
  pthread_mutex_unlock(&ASPFilterLevelMutex);
  return READ_OK;
 }
/*END of ASPFilterLevel*/

/*ASPMaxDeltaTime*/
ReadWriteAccess_t DataDictionaryInitASPMaxDeltaTimeDbl(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetASPMaxDeltaTimeDbl(GSDType *GSD, C8 *ASPMaxDeltaTime)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxDeltaTime", ASPMaxDeltaTime, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
    GSD->ASPMaxDeltaTimeDbl = atof(ASPMaxDeltaTime);
    pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetASPMaxDeltaTimeDbl(GSDType *GSD, dbl *ASPMaxDeltaTime)
{
  pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
  *ASPMaxDeltaTime = GSD->ASPMaxDeltaTimeDbl;
  pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
  return READ_OK;
 }
/*END of ASPFilterLevel*/


/*TimeServerIP*/
ReadWriteAccess_t DataDictionaryInitTimeServerIPU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetTimeServerIPU32(GSDType *GSD, C8 *TimeServerIP)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(GSDType *GSD, U32 *TimeServerIP)
{
  pthread_mutex_lock(&TimeServerIPMutex);
  *TimeServerIP = GSD->TimeServerIPU32;
  pthread_mutex_unlock(&TimeServerIPMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetTimeServerIPC8(GSDType *GSD, C8 *TimeServerIP)
{
  pthread_mutex_lock(&TimeServerIPMutex);
  strcat(TimeServerIP, GSD->TimeServerIPC8);
  pthread_mutex_unlock(&TimeServerIPMutex);
  return READ_OK;
 }

/*END of TimeServerIP*/


/*TimeServerPort*/
ReadWriteAccess_t DataDictionaryInitTimeServerPortU16(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetTimeServerPortU16(GSDType *GSD, C8 *TimeServerPort)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerPort", TimeServerPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&TimeServerPortMutex);
    GSD->TimeServerPortU16 = atoi(TimeServerPort);
    pthread_mutex_unlock(&TimeServerPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(GSDType *GSD, U16 *TimeServerPort)
{
  pthread_mutex_lock(&TimeServerPortMutex);
  *TimeServerPort = GSD->TimeServerPortU16;
  pthread_mutex_unlock(&TimeServerPortMutex);
  return READ_OK;
 }
/*END of TimeServerPort*/


/*SimulatorIP*/
ReadWriteAccess_t DataDictionaryInitSimulatorIPU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetSimulatorIPU32(GSDType *GSD, C8 *SimulatorIP)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetSimulatorIPU32(GSDType *GSD, U32 *SimulatorIP)
{
  pthread_mutex_lock(&SimulatorIPMutex);
  *SimulatorIP = GSD->SimulatorIPU32;
  pthread_mutex_unlock(&SimulatorIPMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetSimulatorIPC8(GSDType *GSD, C8 *SimulatorIP)
{
  pthread_mutex_lock(&SimulatorIPMutex);
  strcat(SimulatorIP, GSD->SimulatorIPC8);
  pthread_mutex_unlock(&SimulatorIPMutex);
  return READ_OK;
 }
/*END of SimulatorIP*/

/*SimulatorTCPPort*/
ReadWriteAccess_t DataDictionaryInitSimulatorTCPPortU16(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetSimulatorTCPPortU16(GSDType *GSD, C8 *SimulatorTCPPort)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorTCPPort", SimulatorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorTCPPortMutex);
    GSD->SimulatorTCPPortU16 = atoi(SimulatorTCPPort);
    pthread_mutex_unlock(&SimulatorTCPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetSimulatorTCPPortU16(GSDType *GSD, U16 *SimulatorTCPPort)
{
  pthread_mutex_lock(&SimulatorTCPPortMutex);
  *SimulatorTCPPort = GSD->SimulatorTCPPortU16;
  pthread_mutex_unlock(&SimulatorTCPPortMutex);
  return READ_OK;
 }
/*END of SimulatorTCPPort*/

/*SimulatorUDPPort*/
ReadWriteAccess_t DataDictionaryInitSimulatorUDPPortU16(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetSimulatorUDPPortU16(GSDType *GSD, C8 *SimulatorUDPPort)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorUDPPort", SimulatorUDPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorUDPPortMutex);
    GSD->SimulatorUDPPortU16 = atoi(SimulatorUDPPort);
    pthread_mutex_unlock(&SimulatorUDPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetSimulatorUDPPortU16(GSDType *GSD, U16 *SimulatorUDPPort)
{
  pthread_mutex_lock(&SimulatorUDPPortMutex);
  *SimulatorUDPPort = GSD->SimulatorUDPPortU16;
  pthread_mutex_unlock(&SimulatorUDPPortMutex);
  return READ_OK;
 }
/*END of SimulatorUDPPort*/

/*SimulatorMode*/
ReadWriteAccess_t DataDictionaryInitSimulatorModeU8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetSimulatorModeU8(GSDType *GSD, C8 *SimulatorMode)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorMode", SimulatorMode, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SimulatorModeMutex);
    GSD->SimulatorModeU8 = atoi(SimulatorMode);
    pthread_mutex_unlock(&SimulatorModeMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetSimulatorModeU8(GSDType *GSD, U8 *SimulatorMode)
{
  pthread_mutex_lock(&SimulatorModeMutex);
  *SimulatorMode = GSD->SimulatorModeU8;
  pthread_mutex_unlock(&SimulatorModeMutex);
  return READ_OK;
 }
/*END of SimulatorMode*/

/*VOILReceivers*/
ReadWriteAccess_t DataDictionaryInitVOILReceiversC8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("VOILReceivers", VOILReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&VOILReceiversMutex);
    strcpy(GSD->VOILReceiversC8, VOILReceivers);
    pthread_mutex_unlock(&VOILReceiversMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetVOILReceiversC8(GSDType *GSD, U8 *VOILReceivers)
{
  pthread_mutex_lock(&VOILReceiversMutex);
  strcpy(VOILReceivers, GSD->VOILReceiversC8);
  pthread_mutex_unlock(&VOILReceiversMutex);
  return READ_OK;
 }
/*END of VOILReceivers*/

/*DTMReceivers*/
ReadWriteAccess_t DataDictionaryInitDTMReceiversC8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("DTMReceivers", DTMReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&DTMReceiversMutex);
    strcpy(GSD->DTMReceiversC8, DTMReceivers);
    pthread_mutex_unlock(&DTMReceiversMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetDTMReceiversC8(GSDType *GSD, U8 *DTMReceivers)
{
  pthread_mutex_lock(&DTMReceiversMutex);
  strcpy(DTMReceivers, GSD->DTMReceiversC8);
  pthread_mutex_unlock(&DTMReceiversMutex);
  return READ_OK;
 }
/*END of DTMReceivers*/

/*External Supervisor IP*/
ReadWriteAccess_t DataDictionaryInitExternalSupervisorIPU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP)
{
  pthread_mutex_lock(&ExternalSupervisorIPMutex);
  *IP = GSD->ExternalSupervisorIPU32;
  pthread_mutex_unlock(&ExternalSupervisorIPMutex);
  return READ_OK;
 }

ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP)
{
  pthread_mutex_lock(&ExternalSupervisorIPMutex);
  strcat(IP, GSD->ExternalSupervisorIPC8);
  pthread_mutex_unlock(&ExternalSupervisorIPMutex);
  return READ_OK;
 }
/*END of External Supervisor IP*/

/*External SupervisorTCPPort*/
ReadWriteAccess_t DataDictionaryInitSupervisorTCPPortU16(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort)
{
  ReadWriteAccess_t Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorTCPPort", SupervisorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&SupervisorTCPPortMutex);
    GSD->SupervisorTCPPortU16 = atoi(SupervisorTCPPort);
    pthread_mutex_unlock(&SupervisorTCPPortMutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

ReadWriteAccess_t DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort)
{
  pthread_mutex_lock(&SupervisorTCPPortMutex);
  *SupervisorTCPPort = GSD->SupervisorTCPPortU16;
  pthread_mutex_unlock(&SupervisorTCPPortMutex);
  return READ_OK;
 }
/*END of External SupervisorTCPPort*/

/*Runtime Variable Subscription Service (RVSS) Configuration*/
ReadWriteAccess_t DataDictionaryInitRVSSConfigU32(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetRVSSConfigU32(GSDType *GSD, U32 RVSSConfig)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetRVSSConfigU32(GSDType *GSD, U32 *RVSSConfig)
{
  pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
  *RVSSConfig = GSD->DataDictionaryRVSSConfigU32;
  pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
  return READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Configuration**/


/*Runtime Variable Subscription Service (RVSS) Rate*/
ReadWriteAccess_t DataDictionaryInitRVSSRateU8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetRVSSRateU8(GSDType *GSD, U8 RVSSRate)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetRVSSRateU8(GSDType *GSD, U8 *RVSSRate)
{
  pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
  *RVSSRate = GSD->DataDictionaryRVSSRateU8;
  pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
  return READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Rate**/


/*ASPDebug*/
ReadWriteAccess_t DataDictionarySetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&ASPDataMutex);
  GSD->ASPData = *ASPD;
  pthread_mutex_unlock(&ASPDataMutex);
  return WRITE_OK;
}

ReadWriteAccess_t DataDictionaryGetRVSSAsp(GSDType *GSD, ASPType *ASPD)
{
  pthread_mutex_lock(&ASPDataMutex);
  *ASPD = GSD->ASPData;
  pthread_mutex_unlock(&ASPDataMutex);
  return READ_OK;
}
/*END ASPDebug*/

/*MiscData*/
ReadWriteAccess_t DataDictionaryInitMiscDataC8(GSDType *GSD)
{
  ReadWriteAccess_t Res = UNDEFINED;
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

ReadWriteAccess_t DataDictionarySetMiscDataC8(GSDType *GSD, C8 *MiscData)
{
  ReadWriteAccess_t Res;
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

ReadWriteAccess_t DataDictionaryGetMiscDataC8(GSDType *GSD, U8 *MiscData)
{
  pthread_mutex_lock(&MiscDataMutex);
  strcpy(MiscData, GSD->MiscDataC8);
  pthread_mutex_unlock(&MiscDataMutex);
  return READ_OK;
 }
/*END of MiscData*/


/*OBCState*/
ReadWriteAccess_t DataDictionarySetOBCStateU8(GSDType *GSD, U8 OBCState)
{
  ReadWriteAccess_t Res;
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
U64 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer)
{
  bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, ParameterName, "", ResultBuffer);
  return strlen(ResultBuffer);
}
