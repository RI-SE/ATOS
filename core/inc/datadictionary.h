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

#ifdef __cplusplus
extern "C" {
#endif
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
	DD_RVSS_ASP,																		//! [ASPType]
	DD_OBC_STATE,																		//!	[OBCState_t]
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
ReadWriteAccess_t DataDictionarySet(const enum DataDictionaryParameter param, const void *newValue, const size_t newValueSize);


ReadWriteAccess_t DataDictionaryConstructor(void);
ReadWriteAccess_t DataDictionaryDestructor(void);

ReadWriteAccess_t DataDictionarySetObjectData(const ObjectDataType * objectData);
ReadWriteAccess_t DataDictionaryClearObjectData(const uint32_t transmitterID);
ReadWriteAccess_t DataDictionaryFreeObjectData();

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

#ifdef __cplusplus
}
#endif
#endif

