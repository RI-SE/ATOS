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

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
I32 DataDictionaryInitOriginLatitudeDbl(void);
I32 DataDictionarySetOriginLatitudeDbl(C8 *Latitude);
I32 DataDictionaryGetOriginLatitudeDbl(dbl *Latitude);

I32 DataDictionaryInitOriginLongitudeDbl(void);
I32 DataDictionarySetOriginLongitudeDbl(C8 *Longitude);
I32 DataDictionaryGetOriginLongitudeDbl(dbl *Longitude);

I32 DataDictionaryInitOriginAltitudeDbl(void);
I32 DataDictionarySetOriginAltitudeDbl(C8 *Altitude);
I32 DataDictionaryGetOriginAltitudeDbl(dbl *Altitude);

I32 DataDictionaryInitVisualizationServerU32(void);
I32 DataDictionarySetVisualizationServerU32(C8 *IP);
I32 DataDictionaryGetVisualizationServer(U32 *IP);

I32 DataDictionaryInitForceToLocalhostU8(void);
I32 DataDictionarySetForceToLocalhostU8(C8 *ForceLocalhost);
I32 DataDictionaryGetForceToLocalhostU8(U8 *ForceLocalhost);

I32 DataDictionaryInitASPMaxTimeDiffDbl(void);
I32 DataDictionarySetASPMaxTimeDiffDbl(C8 *ASPMaxTimeDiff);
I32 DataDictionaryGetASPMaxTimeDiffDbl(dbl *ASPMaxTimeDiff);

I32 DataDictionaryInitASPMaxTrajDiffDbl(void);
I32 DataDictionarySetASPMaxTrajDiffDbl(C8 *ASPMaxTimeDiff);
I32 DataDictionaryGetASPMaxTrajDiffDbl(dbl *ASPMaxTimeDiff);

I32 DataDictionaryInitASPStepBackCountU32(void);
I32 DataDictionarySetASPStepBackCountU32(C8 *ASPStepBackCount);
I32 DataDictionaryGetASPStepBackCountU32(U32 *ASPStepBackCount);

I32 DataDictionaryInitASPFilterLevelU16(void);
I32 DataDictionarySetASPFilterLevelU16(C8 *ASPFilterLevel);
I32 DataDictionaryGetASPFilterLevelU16(U16 *ASPFilterLevel);

I32 DataDictionaryInitASPMaxDeltaTimeDbl(void);
I32 DataDictionarySetASPMaxDeltaTimeDbl(C8 *ASPMaxDeltaTime);
I32 DataDictionaryGetASPMaxDeltaTimeDbl(U16 *ASPMaxDeltaTime);

I32 DataDictionaryInitTimeServerIPU32(void);
I32 DataDictionarySetTimeServerIPU32(C8 *TimeServerIP);
I32 DataDictionaryGetTimeServerIPU32(U32 *TimeServerIP);

I32 DataDictionaryInitTimeServerPortU16(void);
I32 DataDictionarySetTimeServerPortU16(C8 *TimeServerPort);
I32 DataDictionaryGetTimeServerPortU16(U16 *TimeServerPort);

I32 DataDictionaryInitSimulatorIPU32(void);
I32 DataDictionarySetSimulatorIPU32(C8 *SimulatorIP);
I32 DataDictionaryGetSimulatorIPU32(U32 *SimulatorIP);

I32 DataDictionaryInitSimulatorTCPPortU16(void);
I32 DataDictionarySetSimulatorTCPPortU16(C8 *SimulatorTCPPort);
I32 DataDictionaryGetSimulatorTCPPortU16(U16 *SimulatorTCPPort);

I32 DataDictionaryInitSimulatorUDPPortU16(void);
I32 DataDictionarySetSimulatorUDPPortU16(C8 *SimulatorUDPPort);
I32 DataDictionaryGetSimulatorUDPPortU16(U16 *SimulatorUDPPort);

I32 DataDictionaryInitSimulatorModeU8(void);
I32 DataDictionarySetSimulatorModeU8(C8 *SimulatorMode);
I32 DataDictionaryGetSimulatorModeU8(U8 *SimulatorMode);

I32 DataDictionaryInitVOILReceiversC8(void);
I32 DataDictionarySetVOILReceiversC8(C8 *VOILReceivers);
I32 DataDictionaryGetVOILReceiversC8(C8 *VOILReceivers);

I32 DataDictionaryInitDTMReceiversC8(void);
I32 DataDictionarySetDTMReceiversC8(C8 *DTMReceivers);
I32 DataDictionaryGetDTMReceiversC8(C8 *DTMReceivers);

I32 DataDictionaryInitExternalSupervisorIPU32(void);
I32 DataDictionarySetExternalSupervisorIPU32(C8 *IP);
I32 DataDictionaryGetExternalSupervisorIPU32(U32 *IP);

I32 DataDictionaryInitSupervisorTCPPortU16(void);
I32 DataDictionarySetSupervisorTCPPortU16(C8 *SupervisorTCPPort);
I32 DataDictionaryGetSupervisorTCPPortU16(U16 *SupervisorTCPPort);

I32 DataDictionaryInitRVSSConfigU32(void);
I32 DataDictionarySetRVSSConfigU32(U32 RVSSConfig);
I32 DataDictionaryGetRVSSConfigU32(U32 *RVSSConfig);

I32 DataDictionaryInitRVSSRateU8(void);
I32 DataDictionarySetRVSSRateU8(U8 RVSSRate);
I32 DataDictionaryGetRVSSRateU8(U8 *RVSSRate);

I32 DataDictionarySetRVSSAsp(ASPType *ASPData);
I32 DataDictionaryGetRVSSAsp(ASPType *ASPData);

I32 DataDictionaryInitMiscDataC8(void);
I32 DataDictionarySetMiscDataC8(C8 *MiscData);
I32 DataDictionaryGetMiscDataC8(C8 *MiscData);


void DataDictionaryConstructor(void);

#endif 

