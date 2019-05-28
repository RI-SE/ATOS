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
#define DD_CONTROL_BUFFER_SIZE_20 20
#define DD_CONTROL_BUFFER_SIZE_52 52
#define DD_CONTROL_TASK_PERIOD_MS 1

// Parameters and variables
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
volatile dbl OriginLatitudeDbl = 0;
volatile dbl OriginLongitudeDbl = 0;
volatile dbl OriginAltitudeDbl = 0;
volatile U32 VisualizationServerU32 = 0;
volatile U8 ForceObjectToLocalhostU8 = 0;
volatile dbl ASPMaxTimeDiffDbl = 0;
volatile dbl ASPMaxTrajDiffDbl = 0;
volatile U32 ASPStepBackCountU32 = 0;
volatile U16 ASPFilterLevelU16 = 0;
volatile dbl ASPMaxDeltaTimeDbl = 0;
volatile U32 TimeServerIPU32 = 0;
volatile U16 TimeServerPortU16 = 0;
volatile U32 SimulatorIPU32 = 0;
volatile U16 SimulatorTCPPortU16 = 0;
volatile U16 SimulatorUDPPortU16 = 0;
volatile U8 SimulatorModeU8 = 0;
C8 VOILReceiversC8[1024];
C8 DTMReceiversC8[1024];
volatile U32 ExternalSupervisorIPU32 = 0;
volatile U16 SupervisorTCPPortU16 = 0;
volatile U32 DataDictionaryRVSSConfigU32 = 0;
volatile U32 DataDictionaryRVSSRateU8 = 0;
volatile ASPType ASPData;
C8 MiscDataC8[1024];

I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/
void DataDictionaryConstructor(void)
{
  DataDictionaryInitOriginLatitudeDbl();
  DataDictionaryInitOriginLongitudeDbl();
  DataDictionaryInitOriginAltitudeDbl();
  DataDictionaryInitVisualizationServerU32();
  DataDictionaryInitForceToLocalhostU8();
  DataDictionaryInitASPMaxTimeDiffDbl();
  DataDictionaryInitASPMaxTrajDiffDbl();
  DataDictionaryInitASPStepBackCountU32();
  DataDictionaryInitASPFilterLevelU16();
  DataDictionaryInitASPMaxDeltaTimeDbl();
  DataDictionaryInitTimeServerIPU32();
  DataDictionaryInitTimeServerPortU16();
  DataDictionaryInitSimulatorIPU32();
  DataDictionaryInitSimulatorTCPPortU16();
  DataDictionaryInitSimulatorUDPPortU16();
  DataDictionaryInitSimulatorModeU8();
  DataDictionaryInitVOILReceiversC8();
  DataDictionaryInitDTMReceiversC8();
  DataDictionaryInitExternalSupervisorIPU32();
  DataDictionaryInitRVSSConfigU32();
  DataDictionaryInitRVSSRateU8();
  DataDictionaryInitSupervisorTCPPortU16();
  DataDictionaryInitMiscDataC8();

}

//  I32 Return
//      1     = WRITE_OK
//      2     = READ_OK
//      3     = READ_WRITE_OK
//      4     = PARAMETER_NOTFOUND
//      5     = OUT_OF_RANGE


/*Origin Latitude*/
I32 DataDictionaryInitOriginLatitudeDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OriginLatidude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    OriginLatitudeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginLatitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLatitudeDbl(C8 *Latitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OriginLatidude", Latitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    OriginLatitudeDbl = atof(Latitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLatitudeDbl(dbl *Latitude)
{
  pthread_mutex_lock(&mutex);
  *Latitude = OriginLatitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Origin Latitude*/

/*Origin Longitude*/
I32 DataDictionaryInitOriginLongitudeDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OriginLongitude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    OriginLongitudeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginLongitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginLongitudeDbl(C8 *Longitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OriginLongitude", Longitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    OriginLongitudeDbl = atof(Longitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginLongitudeDbl(dbl *Longitude)
{
  pthread_mutex_lock(&mutex);
  *Longitude = OriginLatitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Origin Longitude*/

/*Origin Altitude*/
I32 DataDictionaryInitOriginAltitudeDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("OriginAltitude=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    OriginAltitudeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"OriginAltitude not found!");
  }

  return Res;
}

I32 DataDictionarySetOriginAltitudeDbl(C8 *Altitude)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("OriginAltitude", Altitude, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    OriginAltitudeDbl = atof(Altitude);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetOriginAltitudeDbl(dbl *Altitude)
{
  pthread_mutex_lock(&mutex);
  *Altitude = OriginAltitudeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Origin Altitude*/

/*VisualizationServer*/
I32 DataDictionaryInitVisualizationServerU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("VisualizationServer=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    VisualizationServerU32 = UtilIPStringToInt(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"VisualizationServer not found!");
  }

  return Res;
}

I32 DataDictionarySetVisualizationServerU32(C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VisualizationServer", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    VisualizationServerU32 = UtilIPStringToInt(IP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVisualizationServer(U32 *IP)
{
  pthread_mutex_lock(&mutex);
  *IP = VisualizationServerU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of VisualizationServer*/


/*ForceToLocalhost*/
I32 DataDictionaryInitForceToLocalhostU8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ForceObjectToLocalhost=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ForceObjectToLocalhostU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ForceObjectToLocalhost not found!");
  }

  return Res;
}

I32 DataDictionarySetForceToLocalhostU8(C8 *ForceLocalhost)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ForceObjectToLocalhost", ForceLocalhost, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ForceObjectToLocalhostU8 = atoi(ForceLocalhost);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetForceToLocalhostU8(U8 *ForceLocalhost)
{
  pthread_mutex_lock(&mutex);
  *ForceLocalhost = ForceObjectToLocalhostU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ForceToLocalhost*/

/*ASPMaxTimeDiff*/
I32 DataDictionaryInitASPMaxTimeDiffDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxTimeDiff=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxTimeDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxTimeDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTimeDiffDbl(C8 *ASPMaxTimeDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTimeDiff", ASPMaxTimeDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxTimeDiffDbl = atof(ASPMaxTimeDiff);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTimeDiffDbl(dbl *ASPMaxTimeDiff)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxTimeDiff = ASPMaxTimeDiffDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPMaxTimeDiff*/

/*ASPMaxTrajDiff*/
I32 DataDictionaryInitASPMaxTrajDiffDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxTrajDiff=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxTrajDiffDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxTrajDiff not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxTrajDiffDbl(C8 *ASPMaxTrajDiff)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxTrajDiff", ASPMaxTrajDiff, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxTrajDiffDbl = atof(ASPMaxTrajDiff);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxTrajDiffDbl(dbl *ASPMaxTrajDiff)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxTrajDiff = ASPMaxTrajDiffDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPMaxTrajDiff*/


/*ASPStepBackCount*/
I32 DataDictionaryInitASPStepBackCountU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPStepBackCount=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ASPStepBackCountU32 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPStepBackCount not found!");
  }

  return Res;
}

I32 DataDictionarySetASPStepBackCountU32(C8 *ASPStepBackCount)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPStepBackCount", ASPStepBackCount, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ASPStepBackCountU32 = atoi(ASPStepBackCount);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPStepBackCountU32(U32 *ASPStepBackCount)
{
  pthread_mutex_lock(&mutex);
  *ASPStepBackCount = ASPStepBackCountU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPStepBackCount*/

/*ASPFilterLevel*/
I32 DataDictionaryInitASPFilterLevelU16(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPFilterLevel=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ASPFilterLevelU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPFilterLevel not found!");
  }

  return Res;
}

I32 DataDictionarySetASPFilterLevelU16(C8 *ASPFilterLevel)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPFilterLevel", ASPFilterLevel, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ASPFilterLevelU16 = atoi(ASPFilterLevel);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPFilterLevelU16(U16 *ASPFilterLevel)
{
  pthread_mutex_lock(&mutex);
  *ASPFilterLevel = ASPFilterLevelU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPFilterLevel*/

/*ASPMaxDeltaTime*/
I32 DataDictionaryInitASPMaxDeltaTimeDbl(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("ASPMaxDeltaTime=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxDeltaTimeDbl = atof(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"ASPMaxDeltaTime not found!");
  }

  return Res;
}

I32 DataDictionarySetASPMaxDeltaTimeDbl(C8 *ASPMaxDeltaTime)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("ASPMaxDeltaTime", ASPMaxDeltaTime, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ASPMaxDeltaTimeDbl = atof(ASPMaxDeltaTime);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetASPMaxDeltaTimeDbl(U16 *ASPMaxDeltaTime)
{
  pthread_mutex_lock(&mutex);
  *ASPMaxDeltaTime = ASPMaxDeltaTimeDbl;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of ASPFilterLevel*/


/*TimeServerIP*/
I32 DataDictionaryInitTimeServerIPU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("TimeServerIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    TimeServerIPU32 = UtilIPStringToInt(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"TimeServerIP not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerIPU32(C8 *TimeServerIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerIP", TimeServerIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    TimeServerIPU32 = UtilIPStringToInt(TimeServerIP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerIPU32(U32 *TimeServerIP)
{
  pthread_mutex_lock(&mutex);
  *TimeServerIP = TimeServerIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of TimeServerIP*/


/*TimeServerPort*/
I32 DataDictionaryInitTimeServerPortU16(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("TimeServerPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    TimeServerPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"TimeServerPort not found!");
  }

  return Res;
}

I32 DataDictionarySetTimeServerPortU16(C8 *TimeServerPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("TimeServerPort", TimeServerPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    TimeServerPortU16 = atoi(TimeServerPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetTimeServerPortU16(U16 *TimeServerPort)
{
  pthread_mutex_lock(&mutex);
  *TimeServerPort = TimeServerPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of TimeServerPort*/


/*SimulatorIP*/
I32 DataDictionaryInitSimulatorIPU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    SimulatorIPU32 = UtilIPStringToInt(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorIP not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorIPU32(C8 *SimulatorIP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorIP", SimulatorIP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    SimulatorIPU32 = UtilIPStringToInt(SimulatorIP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorIPU32(U32 *SimulatorIP)
{
  pthread_mutex_lock(&mutex);
  *SimulatorIP = SimulatorIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorIP*/

/*SimulatorTCPPort*/
I32 DataDictionaryInitSimulatorTCPPortU16(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorTCPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    SimulatorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorTCPPortU16(C8 *SimulatorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorTCPPort", SimulatorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    SimulatorTCPPortU16 = atoi(SimulatorTCPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorTCPPortU16(U16 *SimulatorTCPPort)
{
  pthread_mutex_lock(&mutex);
  *SimulatorTCPPort = SimulatorTCPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorTCPPort*/

/*SimulatorUDPPort*/
I32 DataDictionaryInitSimulatorUDPPortU16(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorUDPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    SimulatorUDPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorUDPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorUDPPortU16(C8 *SimulatorUDPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorUDPPort", SimulatorUDPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    SimulatorUDPPortU16 = atoi(SimulatorUDPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorUDPPortU16(U16 *SimulatorUDPPort)
{
  pthread_mutex_lock(&mutex);
  *SimulatorUDPPort = SimulatorUDPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorUDPPort*/

/*SimulatorMode*/
I32 DataDictionaryInitSimulatorModeU8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SimulatorMode=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    SimulatorModeU8 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SimulatorMode not found!");
  }

  return Res;
}

I32 DataDictionarySetSimulatorModeU8(C8 *SimulatorMode)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SimulatorMode", SimulatorMode, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    SimulatorModeU8 = atoi(SimulatorMode);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSimulatorModeU8(U8 *SimulatorMode)
{
  pthread_mutex_lock(&mutex);
  *SimulatorMode = SimulatorModeU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of SimulatorMode*/

/*VOILReceivers*/
I32 DataDictionaryInitVOILReceiversC8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("VOILReceivers=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(VOILReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"VOILReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetVOILReceiversC8(C8 *VOILReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("VOILReceivers", VOILReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    strcpy(VOILReceiversC8, VOILReceivers);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetVOILReceiversC8(U8 *VOILReceivers)
{
  pthread_mutex_lock(&mutex);
  strcpy(VOILReceivers, VOILReceiversC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of VOILReceivers*/

/*DTMReceivers*/
I32 DataDictionaryInitDTMReceiversC8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("DTMReceivers=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(DTMReceiversC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"DTMReceivers not found!");
  }

  return Res;
}

I32 DataDictionarySetDTMReceiversC8(C8 *DTMReceivers)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("DTMReceivers", DTMReceivers, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    strcpy(DTMReceiversC8, DTMReceivers);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetDTMReceiversC8(U8 *DTMReceivers)
{
  pthread_mutex_lock(&mutex);
  strcpy(DTMReceivers, DTMReceiversC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of DTMReceivers*/

/*External Supervisor IP*/
I32 DataDictionaryInitExternalSupervisorIPU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SupervisorIP=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    ExternalSupervisorIPU32 = UtilIPStringToInt(ResultBufferC8);
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

I32 DataDictionarySetExternalSupervisorIPU32(C8 *IP)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorIP", IP, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    ExternalSupervisorIPU32 = UtilIPStringToInt(IP);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetExternalSupervisorIPU32(U32 *IP)
{
  pthread_mutex_lock(&mutex);
  *IP = ExternalSupervisorIPU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of External Supervisor IP*/

/*External SupervisorTCPPort*/
I32 DataDictionaryInitSupervisorTCPPortU16(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("SupervisorTCPPort=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    SupervisorTCPPortU16 = atoi(ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"SupervisorTCPPort not found!");
  }

  return Res;
}

I32 DataDictionarySetSupervisorTCPPortU16(C8 *SupervisorTCPPort)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("SupervisorTCPPort", SupervisorTCPPort, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    SupervisorTCPPortU16 = atoi(SupervisorTCPPort);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetSupervisorTCPPortU16(U16 *SupervisorTCPPort)
{
  pthread_mutex_lock(&mutex);
  *SupervisorTCPPort = SupervisorTCPPortU16;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of External SupervisorTCPPort*/




/*Runtime Variable Subscription Service (RVSS) Configuration*/
I32 DataDictionaryInitRVSSConfigU32(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("RVSSConfig=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    DataDictionaryRVSSConfigU32 = atoi(ResultBufferC8);
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

I32 DataDictionarySetRVSSConfigU32(U32 RVSSConfig)
{
  I32 Res;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
  sprintf(ResultBufferC8, "%" PRIu32, RVSSConfig);

  if(Res = UtilWriteConfigurationParameter("RVSSConfig", ResultBufferC8, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    DataDictionaryRVSSConfigU32 = RVSSConfig;
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSConfigU32(U32 *RVSSConfig)
{
  pthread_mutex_lock(&mutex);
  *RVSSConfig = DataDictionaryRVSSConfigU32;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Configuration**/


/*Runtime Variable Subscription Service (RVSS) Rate*/
I32 DataDictionaryInitRVSSRateU8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("RVSSRate=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    DataDictionaryRVSSRateU8 = (U8)atoi(ResultBufferC8);
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

I32 DataDictionarySetRVSSRateU8(U8 RVSSRate)
{
  I32 Res;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
  sprintf(ResultBufferC8, "%" PRIu8, RVSSRate);

  if(Res = UtilWriteConfigurationParameter("RVSSRate", ResultBufferC8, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    DataDictionaryRVSSRateU8 = RVSSRate;
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetRVSSRateU8(U8 *RVSSRate)
{
  pthread_mutex_lock(&mutex);
  *RVSSRate = DataDictionaryRVSSRateU8;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of Runtime Variable Subscription Service (RVSS) Rate**/


/*ASPDebug*/
I32 DataDictionarySetRVSSAsp(ASPType *ASPD)
{
  pthread_mutex_lock(&mutex);
  ASPData = *ASPD;
  pthread_mutex_unlock(&mutex);
  return (I32) WRITE_OK;
}

I32 DataDictionaryGetRVSSAsp(ASPType *ASPD)
{
  pthread_mutex_lock(&mutex);
  *ASPD = ASPData;
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
}
/*END ASPDebug*/

/*MiscData*/
I32 DataDictionaryInitMiscDataC8(void)
{
  I32 Res = UNDEFINED;
  C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];
  if(Res = DataDictionarySearchParameter("MiscData=", ResultBufferC8))
  {
    Res = READ_OK;
    pthread_mutex_lock(&mutex);
    strcpy(MiscDataC8, ResultBufferC8);
    pthread_mutex_unlock(&mutex);
  }
  else
  {
    Res = PARAMETER_NOTFOUND;
    LogMessage(LOG_LEVEL_INFO,"MiscData not found!");
  }

  return Res;
}

I32 DataDictionarySetMiscDataC8(C8 *MiscData)
{
  I32 Res;
  if(Res = UtilWriteConfigurationParameter("MiscData", MiscData, 0))
  {
    Res = WRITE_OK;
    pthread_mutex_lock(&mutex);
    strcpy(MiscDataC8, MiscData);
    pthread_mutex_unlock(&mutex);
  } else Res = PARAMETER_NOTFOUND; 
  return Res; 
}

I32 DataDictionaryGetMiscDataC8(U8 *MiscData)
{
  pthread_mutex_lock(&mutex);
  strcpy(MiscData, MiscDataC8);
  pthread_mutex_unlock(&mutex);
  return (I32) READ_OK;
 }
/*END of MiscData*/








I32 DataDictionarySearchParameter(C8 *ParameterName, C8 *ResultBuffer)
{
  bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(DD_CONTROL_CONF_FILE_PATH, ParameterName, "", ResultBuffer);
  return strlen(ResultBuffer);
}