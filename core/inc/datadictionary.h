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

#include "util.h"
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
enum DataDictionaryParameter {
	DD_SCENARIO_NAME = CONFIGURATION_PARAMETER_SCENARIO_NAME,							//! [string]
	DD_ORIGIN_LATITUDE = CONFIGURATION_PARAMETER_ORIGIN_LATITUDE,						//! [double,string]
	DD_ORIGIN_LONGITUDE = CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE,						//! [double,string]
	DD_ORIGIN_ALTITUDE = CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE,						//! [double,string]
	DD_VISUALIZATION_SERVER_NAME = CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME,	//!	[in_addr_t,string]
	DD_FORCE_OBJECT_TO_LOCALHOST = CONFIGURATION_PARAMETER_FORCE_OBJECT_TO_LOCALHOST,	//!	[uint]
	DD_ASP_MAX_TIME_DIFF = CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF,					//! [double,string]
	DD_ASP_MAX_TRAJ_DIFF = CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF,					//! [double,string]
	DD_ASP_STEP_BACK_COUNT = CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT,				//!	[uint]
	DD_ASP_FILTER_LEVEL = CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL,						//! [double,string]
	DD_ASP_MAX_DELTA_TIME = CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME,					//! [double,string]
	DD_TIME_SERVER_IP = CONFIGURATION_PARAMETER_TIME_SERVER_IP,							//!	[in_addr_t,string]
	DD_TIME_SERVER_PORT = CONFIGURATION_PARAMETER_TIME_SERVER_PORT,						//!	[uint]
	DD_SIMULATOR_IP = CONFIGURATION_PARAMETER_SIMULATOR_IP,								//!	[in_addr_t,string]
	DD_SIMULATOR_PORT_TCP = CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP,					//!	[uint]
	DD_SIMULATOR_PORT_UDP = CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP,					//!	[uint]
	DD_SIMULATOR_MODE = CONFIGURATION_PARAMETER_SIMULATOR_MODE,							//!	[uint]
	DD_VOIL_RECEIVERS = CONFIGURATION_PARAMETER_VOIL_RECEIVERS,							//! [string]
	DD_DTM_RECEIVERS = CONFIGURATION_PARAMETER_DTM_RECEIVERS,							//! [string]
	DD_EXTERNAL_SUPERVISOR_IP = CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP,			//!	[in_addr_t,string]
	DD_EXTERNAL_SUPERVISOR_PORT_TCP =
	CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP,								//! [uint]
	DD_RVSS_CONFIG = CONFIGURATION_PARAMETER_RVSS_CONFIG,								//!	[uint]
	DD_RVSS_RATE = CONFIGURATION_PARAMETER_RVSS_RATE,									//!	[uint]
	DD_MAX_PACKETS_LOST = CONFIGURATION_PARAMETER_MAX_PACKETS_LOST,						//!	[uint]
	DD_MISC_DATA = CONFIGURATION_PARAMETER_MISC_DATA,									//! [string]
	DD_PARAMETER_INVALID = CONFIGURATION_PARAMETER_INVALID,								//!
	DD_RVSS_ASP,																		//!
	DD_OBC_STATE,																		//!
	DD_OBJECT_DATA,																		//!
	DD_NUMBER_OF_OBJECTS,																//!
	DD_OBJECT_TRANSMITTER_IDS,															//!
	DD_OBJECT_TRANSMITTER_ID,															//!
	DD_OBJECT_TRANSMITTER_IP,															//!
	DD_OBJECT_ENABLE_STATUS,															//!
	DD_MONITOR_DATA,																	//!
	DD_MONITOR_DATA_RECEIVE_TIME														//!
};

ReadWriteAccess_t DataDictionaryGet(const enum DataDictionaryParameter param, void *result, const size_t resultSize);

ReadWriteAccess_t DataDictionaryInitScenarioName();
ReadWriteAccess_t DataDictionaryGetScenarioName(char* name, const size_t nameLength);

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

ReadWriteAccess_t DataDictionaryInitForceToLocalhostU8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetForceToLocalhostU8(GSDType *GSD, C8 *ForceLocalhost);
ReadWriteAccess_t DataDictionaryGetForceToLocalhostU8(GSDType *GSD, U8 *ForceLocalhost);

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

ReadWriteAccess_t DataDictionaryInitTimeServerIPU32(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetTimeServerIPU32(GSDType *GSD, C8 *TimeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(GSDType *GSD, U32 *TimeServerIP);
ReadWriteAccess_t DataDictionaryGetTimeServerIPC8(GSDType *GSD, C8 *TimeServerIP, U32 BuffLen);

ReadWriteAccess_t DataDictionaryInitTimeServerPortU16(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetTimeServerPortU16(GSDType *GSD, C8 *TimeServerPort);
ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(GSDType *GSD, U16 *TimeServerPort);

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

ReadWriteAccess_t DataDictionaryInitRVSSRateU8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetRVSSRateU8(GSDType *GSD, U8 RVSSRate);
ReadWriteAccess_t DataDictionaryGetRVSSRateU8(GSDType *GSD, U8 *RVSSRate);

ReadWriteAccess_t DataDictionarySetRVSSAsp(GSDType *GSD, ASPType *ASPData);
ReadWriteAccess_t DataDictionaryGetRVSSAsp(GSDType *GSD, ASPType *ASPData);

ReadWriteAccess_t DataDictionaryInitMiscDataC8(GSDType *GSD);
ReadWriteAccess_t DataDictionarySetMiscDataC8(GSDType *GSD, C8 *MiscData);
ReadWriteAccess_t DataDictionaryGetMiscDataC8(GSDType *GSD, C8 *MiscData, U32 BuffLen);

ReadWriteAccess_t DataDictionarySetOBCStateU8(GSDType *GSD, OBCState_t OBCState);
OBCState_t DataDictionaryGetOBCStateU8(GSDType *GSD);

ReadWriteAccess_t DataDictionaryInitMaxPacketsLost(void);
ReadWriteAccess_t DataDictionarySetMaxPacketsLost(uint8_t maxPacketsLostSetting);
ReadWriteAccess_t DataDictionaryGetMaxPacketsLost(uint8_t * maxPacketsLostSetting);

ReadWriteAccess_t DataDictionaryConstructor(GSDType *GSD);
ReadWriteAccess_t DataDictionaryDestructor(GSDType *GSD);

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

ReadWriteAccess_t DataDictionarySetObjectEnableStatus(const uint32_t transmitterID, ObjectEnabledType enabledStatus);
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusById(const uint32_t transmitterID, ObjectEnabledType *enabledStatus);
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusByIp(const in_addr_t ClientIP, ObjectEnabledType *enabledStatus);

ReadWriteAccess_t DataDictionaryGetMonitorData(const uint32_t TransmitterID, ObjectMonitorType * monitorData);
ReadWriteAccess_t DataDictionarySetMonitorData(const uint32_t transmitterID, const ObjectMonitorType * monitorData, const struct timeval * receiveTime);
ReadWriteAccess_t DataDictionaryGetMonitorDataReceiveTime(const uint32_t transmitterID, struct timeval * lastDataUpdate);
ReadWriteAccess_t DataDictionarySetMonitorDataReceiveTime(const uint32_t transmitterID, const struct timeval * lastDataUpdate);

#endif 

