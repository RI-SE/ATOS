/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : datadictionary.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/
#ifndef __DATADICTIONARY_H_INCLUDED__
#define __DATADICTIONARY_H_INCLUDED__

//#include "util.h"
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
I32 DataDictionaryInitOriginLatitudeDbl(GSDType *GSD);
I32 DataDictionarySetOriginLatitudeDbl(GSDType *GSD, C8 *Latitude);
I32 DataDictionaryGetOriginLatitudeDbl(GSDType *GSD, dbl *Latitude);
I32 DataDictionaryGetOriginLatitudeC8(GSDType *GSD, C8 *Latitude);

I32 DataDictionaryInitOriginLongitudeDbl(GSDType *GSD);
I32 DataDictionarySetOriginLongitudeDbl(GSDType *GSD, C8 *Longitude);
I32 DataDictionaryGetOriginLongitudeDbl(GSDType *GSD, dbl *Longitude);
I32 DataDictionaryGetOriginLongitudeC8(GSDType *GSD, C8 *Longitude);

I32 DataDictionaryInitOriginAltitudeDbl(GSDType *GSD);
I32 DataDictionarySetOriginAltitudeDbl(GSDType *GSD, C8 *Altitude);
I32 DataDictionaryGetOriginAltitudeDbl(GSDType *GSD, dbl *Altitude);
I32 DataDictionaryGetOriginAltitudeC8(GSDType *GSD, C8 *Altitude);

I32 DataDictionaryInitVisualizationServerU32(GSDType *GSD);
I32 DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP);
I32 DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP);
I32 DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP);

I32 DataDictionaryInitForceToLocalhostU8(GSDType *GSD);
I32 DataDictionarySetForceToLocalhostU8(GSDType *GSD, C8 *ForceLocalhost);
I32 DataDictionaryGetForceToLocalhostU8(GSDType *GSD, U8 *ForceLocalhost);

I32 DataDictionaryInitASPMaxTimeDiffDbl(GSDType *GSD);
I32 DataDictionarySetASPMaxTimeDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff);
I32 DataDictionaryGetASPMaxTimeDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff);

I32 DataDictionaryInitASPMaxTrajDiffDbl(GSDType *GSD);
I32 DataDictionarySetASPMaxTrajDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff);
I32 DataDictionaryGetASPMaxTrajDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff);

I32 DataDictionaryInitASPStepBackCountU32(GSDType *GSD);
I32 DataDictionarySetASPStepBackCountU32(GSDType *GSD, C8 *ASPStepBackCount);
I32 DataDictionaryGetASPStepBackCountU32(GSDType *GSD, U32 *ASPStepBackCount);

I32 DataDictionaryInitASPFilterLevelDbl(GSDType *GSD);
I32 DataDictionarySetASPFilterLevelDbl(GSDType *GSD, C8 *ASPFilterLevel);
I32 DataDictionaryGetASPFilterLevelDbl(GSDType *GSD, dbl *ASPFilterLevel);

I32 DataDictionaryInitASPMaxDeltaTimeDbl(GSDType *GSD);
I32 DataDictionarySetASPMaxDeltaTimeDbl(GSDType *GSD, C8 *ASPMaxDeltaTime);
I32 DataDictionaryGetASPMaxDeltaTimeDbl(GSDType *GSD, dbl *ASPMaxDeltaTime);

I32 DataDictionaryInitTimeServerIPU32(GSDType *GSD);
I32 DataDictionarySetTimeServerIPU32(GSDType *GSD, C8 *TimeServerIP);
I32 DataDictionaryGetTimeServerIPU32(GSDType *GSD, U32 *TimeServerIP);
I32 DataDictionaryGetTimeServerIPC8(GSDType *GSD, C8 *TimeServerIP);

I32 DataDictionaryInitTimeServerPortU16(GSDType *GSD);
I32 DataDictionarySetTimeServerPortU16(GSDType *GSD, C8 *TimeServerPort);
I32 DataDictionaryGetTimeServerPortU16(GSDType *GSD, U16 *TimeServerPort);

I32 DataDictionaryInitSimulatorIPU32(GSDType *GSD);
I32 DataDictionarySetSimulatorIPU32(GSDType *GSD, C8 *SimulatorIP);
I32 DataDictionaryGetSimulatorIPU32(GSDType *GSD, U32 *SimulatorIP);
I32 DataDictionaryGetSimulatorIPC8(GSDType *GSD, C8 *SimulatorIP);

I32 DataDictionaryInitSimulatorTCPPortU16(GSDType *GSD);
I32 DataDictionarySetSimulatorTCPPortU16(GSDType *GSD, C8 *SimulatorTCPPort);
I32 DataDictionaryGetSimulatorTCPPortU16(GSDType *GSD, U16 *SimulatorTCPPort);

I32 DataDictionaryInitSimulatorUDPPortU16(GSDType *GSD);
I32 DataDictionarySetSimulatorUDPPortU16(GSDType *GSD, C8 *SimulatorUDPPort);
I32 DataDictionaryGetSimulatorUDPPortU16(GSDType *GSD, U16 *SimulatorUDPPort);

I32 DataDictionaryInitSimulatorModeU8(GSDType *GSD);
I32 DataDictionarySetSimulatorModeU8(GSDType *GSD, C8 *SimulatorMode);
I32 DataDictionaryGetSimulatorModeU8(GSDType *GSD, U8 *SimulatorMode);

I32 DataDictionaryInitVOILReceiversC8(GSDType *GSD);
I32 DataDictionarySetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers);
I32 DataDictionaryGetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers);

I32 DataDictionaryInitDTMReceiversC8(GSDType *GSD);
I32 DataDictionarySetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers);
I32 DataDictionaryGetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers);

I32 DataDictionaryInitExternalSupervisorIPU32(GSDType *GSD);
I32 DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP);
I32 DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP);
I32 DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP);

I32 DataDictionaryInitSupervisorTCPPortU16(GSDType *GSD);
I32 DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort);
I32 DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort);

I32 DataDictionaryInitRVSSConfigU32(GSDType *GSD);
I32 DataDictionarySetRVSSConfigU32(GSDType *GSD, U32 RVSSConfig);
I32 DataDictionaryGetRVSSConfigU32(GSDType *GSD, U32 *RVSSConfig);

I32 DataDictionaryInitRVSSRateU8(GSDType *GSD);
I32 DataDictionarySetRVSSRateU8(GSDType *GSD, U8 RVSSRate);
I32 DataDictionaryGetRVSSRateU8(GSDType *GSD, U8 *RVSSRate);

I32 DataDictionarySetRVSSAsp(GSDType *GSD, ASPType *ASPData);
I32 DataDictionaryGetRVSSAsp(GSDType *GSD, ASPType *ASPData);

I32 DataDictionaryInitMiscDataC8(GSDType *GSD);
I32 DataDictionarySetMiscDataC8(GSDType *GSD, C8 *MiscData);
I32 DataDictionaryGetMiscDataC8(GSDType *GSD, C8 *MiscData);

I32 DataDictionarySetOBCStateU8(GSDType *GSD, U8 OBCState);
U8 DataDictionaryGetOBCStateU8(GSDType *GSD);

void DataDictionaryConstructor(GSDType *GSD);

#endif 

