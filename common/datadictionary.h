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
ReadWriteAccess_t DataDictionarySetOriginLatitudeDbl(const char* latitude);
ReadWriteAccess_t DataDictionaryGetOriginLatitudeDbl(double_t* latitude);
ReadWriteAccess_t DataDictionaryGetOriginLatitudeString(char* latitude, const size_t bufferLength);

ReadWriteAccess_t DataDictionarySetOriginLongitudeDbl(const char* longitude);
ReadWriteAccess_t DataDictionaryGetOriginLongitudeDbl(double_t* longitude);
ReadWriteAccess_t DataDictionaryGetOriginLongitudeString(char* longitude, const size_t bufferLength);

ReadWriteAccess_t DataDictionarySetOriginAltitudeDbl(const char*  altitude);
ReadWriteAccess_t DataDictionaryGetOriginAltitudeDbl(double_t* altitude);
ReadWriteAccess_t DataDictionaryGetOriginAltitudeString(char* altitude, const size_t bufferLength);

ReadWriteAccess_t DataDictionaryInitVisualizationServerU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetVisualizationServerU32(GSDType *GSD, C8 *IP);
ReadWriteAccess_t DataDictionaryGetVisualizationServerU32(GSDType *GSD, U32 *IP);
ReadWriteAccess_t DataDictionaryGetVisualizationServerC8(GSDType *GSD, C8 *IP, U32 BuffLen);

ReadWriteAccess_t DataDictionarySetASPMaxTimeDiffDbl(const char *ASPMaxTimeDiff);
ReadWriteAccess_t DataDictionaryGetASPMaxTimeDiffDbl(double_t *ASPMaxTimeDiff);

ReadWriteAccess_t DataDictionarySetASPMaxTrajDiffDbl(const char *ASPMaxTimeDiff);
ReadWriteAccess_t DataDictionaryGetASPMaxTrajDiffDbl(double_t *ASPMaxTimeDiff);

ReadWriteAccess_t DataDictionarySetASPStepBackCountU32(const char *ASPStepBackCount);
ReadWriteAccess_t DataDictionaryGetASPStepBackCountU32(uint32_t *ASPStepBackCount);

ReadWriteAccess_t DataDictionarySetASPFilterLevelDbl(const char *ASPFilterLevel);
ReadWriteAccess_t DataDictionaryGetASPFilterLevelDbl(double_t *ASPFilterLevel);

ReadWriteAccess_t DataDictionarySetASPMaxDeltaTimeDbl(const char *ASPMaxDeltaTime);
ReadWriteAccess_t DataDictionaryGetASPMaxDeltaTimeDbl(double_t *ASPMaxDeltaTime);

ReadWriteAccess_t DataDictionarySetTimeServerIPU32(const char* timeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(in_addr_t* timeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPString(char* timeServerIP, size_t bufferLength);

ReadWriteAccess_t DataDictionarySetTimeServerPortU16(const char* timeServerPort);
ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(uint16_t* timeServerPort);

ReadWriteAccess_t DataDictionarySetSimulatorIPU32(const char* simulatorIP);
ReadWriteAccess_t DataDictionaryGetSimulatorIPU32(in_addr_t* simulatorIP);
ReadWriteAccess_t DataDictionaryGetSimulatorIPString(char* simulatorIP, const size_t bufferLength);

ReadWriteAccess_t DataDictionarySetSimulatorTCPPortU16(const char* simulatorTCPPort);
ReadWriteAccess_t DataDictionaryGetSimulatorTCPPortU16(uint16_t* simulatorTCPPort);

ReadWriteAccess_t DataDictionarySetSimulatorUDPPortU16(const char* simulatorUDPPort);
ReadWriteAccess_t DataDictionaryGetSimulatorUDPPortU16(uint16_t* simulatorUDPPort);

ReadWriteAccess_t DataDictionarySetSimulatorModeU8(const char* simulatorMode);
ReadWriteAccess_t DataDictionaryGetSimulatorModeU8(uint8_t* simulatorMode);

ReadWriteAccess_t DataDictionarySetVOILReceiversStr(const char* VOILReceivers);
ReadWriteAccess_t DataDictionaryGetVOILReceiversStr(char* VOILReceivers, uint32_t buflen);

ReadWriteAccess_t DataDictionarySetDTMReceiversStr(const char *DTMReceivers);
ReadWriteAccess_t DataDictionaryGetDTMReceiversStr(char *DTMReceivers, uint32_t buflen);

ReadWriteAccess_t DataDictionaryInitExternalSupervisorIPU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetExternalSupervisorIPU32(GSDType *GSD, C8 *IP);
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPU32(GSDType *GSD, U32 *IP);
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPC8(GSDType *GSD, C8 *IP, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitSupervisorTCPPortU16(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetSupervisorTCPPortU16(GSDType *GSD, C8 *SupervisorTCPPort);
ReadWriteAccess_t DataDictionaryGetSupervisorTCPPortU16(GSDType *GSD, U16 *SupervisorTCPPort);

ReadWriteAccess_t DataDictionarySetRVSSConfigU32(uint32_t RVSSConfig);
ReadWriteAccess_t DataDictionaryGetRVSSConfigU32(uint32_t *RVSSConfig);

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
