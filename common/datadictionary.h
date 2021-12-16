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

#define SHARED_MEMORY_PATH "/dev/shm/maestro/"
#define MISC_DATA_MAX_SIZE 1024

#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
ReadWriteAccess_t DataDictionaryInitScenarioName();
ReadWriteAccess_t DataDictionarySetScenarioName(const char* name, const size_t nameLength);
ReadWriteAccess_t DataDictionaryGetScenarioName(char* name, const size_t nameLength);

//TODO: We should have one call for the origin, this in order to make sure that all the three parts of the position are updated at the same time, otherwise there is a small risk of us using old and new values for origin thereby we might get a completely messed up origin
ReadWriteAccess_t DataDictionaryInitOriginLatitudeDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetOriginLatitudeDbl(GSDType *GSD, C8 *Latitude);
ReadWriteAccess_t DataDictionaryGetOriginLatitudeDbl(GSDType *GSD, dbl *Latitude);
ReadWriteAccess_t DataDictionaryGetOriginLatitudeC8(GSDType *GSD, C8 *Latitude, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitOriginLongitudeDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetOriginLongitudeDbl(GSDType *GSD, C8 *Longitude);
ReadWriteAccess_t DataDictionaryGetOriginLongitudeDbl(GSDType *GSD, dbl *Longitude);
ReadWriteAccess_t DataDictionaryGetOriginLongitudeC8(GSDType *GSD, C8 *Longitude, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitOriginAltitudeDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetOriginAltitudeDbl(GSDType *GSD, C8 *Altitude);
ReadWriteAccess_t DataDictionaryGetOriginAltitudeDbl(GSDType *GSD, dbl *Altitude);
ReadWriteAccess_t DataDictionaryGetOriginAltitudeC8(GSDType *GSD, C8 *Altitude, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitVisualizationServerU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP);
ReadWriteAccess_t DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP);
ReadWriteAccess_t DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitASPMaxTimeDiffDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetASPMaxTimeDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff);
ReadWriteAccess_t DataDictionaryGetASPMaxTimeDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff);

ReadWriteAccess_t DataDictionaryInitASPMaxTrajDiffDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetASPMaxTrajDiffDbl(GSDType *GSD, C8 *ASPMaxTimeDiff);
ReadWriteAccess_t DataDictionaryGetASPMaxTrajDiffDbl(GSDType *GSD, dbl *ASPMaxTimeDiff);

ReadWriteAccess_t DataDictionaryInitASPStepBackCountU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetASPStepBackCountU32(GSDType *GSD, C8 *ASPStepBackCount);
ReadWriteAccess_t DataDictionaryGetASPStepBackCountU32(GSDType *GSD, U32 *ASPStepBackCount);

ReadWriteAccess_t DataDictionaryInitASPFilterLevelDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetASPFilterLevelDbl(GSDType *GSD, C8 *ASPFilterLevel);
ReadWriteAccess_t DataDictionaryGetASPFilterLevelDbl(GSDType *GSD, dbl *ASPFilterLevel);

ReadWriteAccess_t DataDictionaryInitASPMaxDeltaTimeDbl(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetASPMaxDeltaTimeDbl(GSDType *GSD, C8 *ASPMaxDeltaTime);
ReadWriteAccess_t DataDictionaryGetASPMaxDeltaTimeDbl(GSDType *GSD, dbl *ASPMaxDeltaTime);

ReadWriteAccess_t DataDictionarySetTimeServerIPU32(const char* timeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(in_addr_t* timeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPC8(char* timeServerIP, size_t bufferLength);

ReadWriteAccess_t DataDictionarySetTimeServerPortU16(const char* timeServerPort);
ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(uint16_t* timeServerPort);

ReadWriteAccess_t DataDictionaryInitSimulatorIPU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSimulatorIPU32(GSDType *GSD, C8 *SimulatorIP);
ReadWriteAccess_t DataDictionaryGetSimulatorIPU32(GSDType *GSD, U32 *SimulatorIP);
ReadWriteAccess_t DataDictionaryGetSimulatorIPC8(GSDType *GSD, C8 *SimulatorIP, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitSimulatorTCPPortU16(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSimulatorTCPPortU16(GSDType *GSD, C8 *SimulatorTCPPort);
ReadWriteAccess_t DataDictionaryGetSimulatorTCPPortU16(GSDType *GSD, U16 *SimulatorTCPPort);

ReadWriteAccess_t DataDictionaryInitSimulatorUDPPortU16(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSimulatorUDPPortU16(GSDType *GSD, C8 *SimulatorUDPPort);
ReadWriteAccess_t DataDictionaryGetSimulatorUDPPortU16(GSDType *GSD, U16 *SimulatorUDPPort);

ReadWriteAccess_t DataDictionaryInitSimulatorModeU8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSimulatorModeU8(GSDType *GSD, C8 *SimulatorMode);
ReadWriteAccess_t DataDictionaryGetSimulatorModeU8(GSDType *GSD, U8 *SimulatorMode);

ReadWriteAccess_t DataDictionaryInitVOILReceiversC8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers);
ReadWriteAccess_t DataDictionaryGetVOILReceiversC8(GSDType *GSD, C8 *VOILReceivers, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitDTMReceiversC8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers);
ReadWriteAccess_t DataDictionaryGetDTMReceiversC8(GSDType *GSD, C8 *DTMReceivers, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitExternalSupervisorIPU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP);
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP);
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitSupervisorTCPPortU16(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort);
ReadWriteAccess_t DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort);

ReadWriteAccess_t DataDictionaryInitRVSSConfigU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetRVSSConfigU32(GSDType *GSD, U32 RVSSConfig);
ReadWriteAccess_t DataDictionaryGetRVSSConfigU32(GSDType *GSD, U32 *RVSSConfig);

ReadWriteAccess_t DataDictionarySetRVSSRateU8(uint8_t RVSSRate);
ReadWriteAccess_t DataDictionaryGetRVSSRateU8(uint8_t *RVSSRate);

ReadWriteAccess_t DataDictionaryInitRVSSAsp();
ReadWriteAccess_t DataDictionarySetRVSSAsp(ASPType *ASPData);
ReadWriteAccess_t DataDictionaryGetRVSSAsp(ASPType *ASPData);

ReadWriteAccess_t DataDictionaryInitMiscData(void);
ReadWriteAccess_t DataDictionarySetMiscData(const char * miscData, const size_t datalen);
ReadWriteAccess_t DataDictionaryGetMiscData(char* miscData, const size_t buflen);

ReadWriteAccess_t DataDictionaryInitStateData();
ReadWriteAccess_t DataDictionarySetOBCState(const OBCState_t OBCState);
ReadWriteAccess_t DataDictionaryGetOBCState(OBCState_t* OBCState);
ReadWriteAccess_t DataDictionaryFreeStateData();

ReadWriteAccess_t DataDictionaryConstructor(GSDType *GSD);
ReadWriteAccess_t DataDictionaryDestructor();

ReadWriteAccess_t DataDictionaryInitMaxPacketsLost(void);
ReadWriteAccess_t DataDictionarySetMaxPacketsLost(uint8_t maxPacketsLostSetting);
ReadWriteAccess_t DataDictionaryGetMaxPacketsLost(uint8_t * maxPacketsLostSetting);

ReadWriteAccess_t DataDictionaryInitTransmitterID(void);
ReadWriteAccess_t DataDictionaryGetTransmitterID(uint32_t * transmitterID);
ReadWriteAccess_t DataDictionarySetTransmitterID(const uint32_t transmitterID);

ReadWriteAccess_t DataDictionarySetObjectData(const ObjectDataType * objectData);
ReadWriteAccess_t DataDictionaryClearObjectData(const uint32_t transmitterID);
ReadWriteAccess_t DataDictionarySetNumberOfObjects(const uint32_t newNumberOfObjects);
ReadWriteAccess_t DataDictionaryGetNumberOfObjects(uint32_t *numberOfObjects);
ReadWriteAccess_t DataDictionaryFreeObjectData();
ReadWriteAccess_t DataDictionaryInitObjectData();

ReadWriteAccess_t DataDictionaryGetObjectTransmitterIDs(uint32_t transmitterIDs[], const uint32_t arraySize);
ReadWriteAccess_t DataDictionaryGetObjectTransmitterIDByIP(const in_addr_t ClientIP, uint32_t *transmitterID);

ReadWriteAccess_t DataDictionaryGetObjectIPByTransmitterID(const uint32_t transmitterID, in_addr_t * ClientIP);
ReadWriteAccess_t DataDictionaryModifyTransmitterID(const uint32_t oldTransmitterID, const uint32_t newTransmitterID);
ReadWriteAccess_t DataDictionaryModifyTransmitterIDByIP(const in_addr_t ipKey, const uint32_t newTransmitterID);

ReadWriteAccess_t DataDictionarySetObjectEnableStatus(const uint32_t transmitterID, ObjectEnabledType enabledStatus);
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusById(const uint32_t transmitterID, ObjectEnabledType *enabledStatus);
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusByIp(const in_addr_t ClientIP, ObjectEnabledType *enabledStatus);

ReadWriteAccess_t DataDictionaryGetMonitorData(const uint32_t transmitterID, ObjectMonitorType * monitorData);
ReadWriteAccess_t DataDictionarySetMonitorData(const uint32_t transmitterID, const ObjectMonitorType * monitorData, const struct timeval * receiveTime);
ReadWriteAccess_t DataDictionaryGetMonitorDataReceiveTime(const uint32_t transmitterID, struct timeval * lastDataUpdate);
ReadWriteAccess_t DataDictionarySetMonitorDataReceiveTime(const uint32_t transmitterID, const struct timeval * lastDataUpdate);

ReadWriteAccess_t DataDictionarySetObjectProperties(const uint32_t transmitterID, const ObjectPropertiesType* objectProperties);
ReadWriteAccess_t DataDictionaryGetObjectProperties(const uint32_t transmitterID, ObjectPropertiesType* objectProperties);
ReadWriteAccess_t DataDictionaryClearObjectProperties(const uint32_t transmitterID);

ReadWriteAccess_t DataDictionaryGetOrigin(const uint32_t transmitterID, GeoPosition * origin);
ReadWriteAccess_t DataDictionarySetOrigin(const uint32_t* transmitterID, const GeoPosition * origin);
ReadWriteAccess_t DataDictionaryInitOrigin();

ReadWriteAccess_t DataDictionarySetRequestedControlAction(const uint32_t transmitterID, const RequestControlActionType* reqCtrlAction);
ReadWriteAccess_t DataDictionaryGetRequestedControlAction(const uint32_t transmitterID, RequestControlActionType* reqCtrlAction);
ReadWriteAccess_t DataDictionaryResetRequestedControlAction(const uint32_t transmitterID);


#ifdef __cplusplus
}
#endif
#endif
