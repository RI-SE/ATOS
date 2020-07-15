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

#include <string>
#include <algorithm>
#include <stdlib.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "datadictionary.h"
#include "logging.h"
#include "shmem.h"


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
static pthread_mutex_t ObjectStatusMutex = PTHREAD_MUTEX_INITIALIZER;



#define MONR_DATA_FILENAME "MonitorData"

static volatile ObjectDataType *objectDataMemory = nullptr;
typedef ReadWriteAccess_t (*GetterFunctionReference)(
		const enum DataDictionaryParameter,
		void*,
		const size_t
);
typedef ReadWriteAccess_t (*SetterFunctionReference)(
		const enum DataDictionaryParameter,
		const void*,
		const size_t
);


/*------------------------------------------------------------
  -- Static function definitions
  ------------------------------------------------------------*/
static ReadWriteAccess_t getConfigAsUnsignedInteger(
		const enum DataDictionaryParameter,
		void*,
		const size_t);
static ReadWriteAccess_t getConfigAsDouble(
		const enum DataDictionaryParameter,
		void*,
		const size_t);
static ReadWriteAccess_t getConfigAsString(
		const enum DataDictionaryParameter,
		void*,
		const size_t);
static ReadWriteAccess_t getAsIPAddress(
		const enum DataDictionaryParameter,
		void*,
		const size_t);
static ReadWriteAccess_t defaultGetter(
		const enum DataDictionaryParameter,
		void*,
		const size_t);

static ReadWriteAccess_t setConfigToUnsignedInteger(
		const enum DataDictionaryParameter,
		const void*,
		const size_t);
static ReadWriteAccess_t setConfigToDouble(
		const enum DataDictionaryParameter,
		const void*,
		const size_t);
static ReadWriteAccess_t setConfigToString(
		const enum DataDictionaryParameter,
		const void*,
		const size_t);
static ReadWriteAccess_t setToIPAddress(
		const enum DataDictionaryParameter,
		const void*,
		const size_t);
static ReadWriteAccess_t defaultSetter(
		const enum DataDictionaryParameter,
		const void*,
		const size_t);

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/


/*!
 * \brief DataDictionaryConstructor Initialize data held by DataDictionary.
Initialization data that is configurable is stored in test.conf.
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryConstructor() {
	ReadWriteAccess_t Res = READ_OK;

	char resultChar[DD_CONTROL_BUFFER_SIZE_1024];
	double resultDouble;
	uint64_t resultInteger;
	in_addr_t resultAddress;
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_SCENARIO_NAME, resultChar, sizeof (resultChar));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ORIGIN_LATITUDE, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ORIGIN_LONGITUDE, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ORIGIN_ALTITUDE, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_VISUALIZATION_SERVER_NAME, resultChar, sizeof (resultChar));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_FORCE_OBJECT_TO_LOCALHOST, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ASP_MAX_TIME_DIFF, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ASP_MAX_TRAJ_DIFF, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ASP_STEP_BACK_COUNT, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ASP_FILTER_LEVEL, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_ASP_MAX_DELTA_TIME, &resultDouble, sizeof (resultDouble));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_TIME_SERVER_IP, &resultAddress, sizeof (resultAddress));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_TIME_SERVER_PORT, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_SIMULATOR_IP, &resultAddress, sizeof (resultAddress));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_SIMULATOR_PORT_TCP, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_SIMULATOR_PORT_UDP, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_SIMULATOR_MODE, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_VOIL_RECEIVERS, resultChar, sizeof (resultChar));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_DTM_RECEIVERS, resultChar, sizeof (resultChar));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_EXTERNAL_SUPERVISOR_IP, &resultAddress, sizeof (resultAddress));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_EXTERNAL_SUPERVISOR_PORT_TCP, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_RVSS_CONFIG, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_RVSS_RATE, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_MAX_PACKETS_LOST, &resultInteger, sizeof (resultInteger));
	Res = Res != READ_OK ? Res : DataDictionaryGet(DD_MISC_DATA, &resultChar, sizeof (resultChar));


	Res = Res == READ_OK ? DataDictionaryInitObjectData() : Res;
	if (Res != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting monitor data memory found");
	}

	OBCState_t initState = OBC_STATE_UNDEFINED;
	DataDictionarySet(DD_OBC_STATE, &initState, sizeof (initState));

	return Res;
}


/*!
 * \brief DataDictionaryDestructor Deallocate data held by DataDictionary.
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryDestructor(GSDType * GSD) {
	ReadWriteAccess_t result = WRITE_OK;

	result = result == WRITE_OK ? DataDictionaryFreeObjectData() : result;

	return result;
}

ReadWriteAccess_t DataDictionarySet(
		const enum DataDictionaryParameter param,
		const void* newValue,
		const size_t newValueSize) {
	SetterFunctionReference setterFunction = &defaultSetter;

	switch (param) {
	case DD_SCENARIO_NAME:
	case DD_VOIL_RECEIVERS:
	case DD_DTM_RECEIVERS:
	case DD_MISC_DATA:
		setterFunction = &setConfigToString;
		break;
	case DD_ORIGIN_LATITUDE:
	case DD_ORIGIN_LONGITUDE:
	case DD_ORIGIN_ALTITUDE:
	case DD_ASP_MAX_TIME_DIFF:
	case DD_ASP_MAX_TRAJ_DIFF:
	case DD_ASP_FILTER_LEVEL:
	case DD_ASP_MAX_DELTA_TIME:
	{
		std::string str(static_cast<const char*>(newValue), newValueSize);
		size_t nonNumericPos = str.find_first_not_of("0123456789+-,.");
		bool isNumericString = nonNumericPos == std::string::npos || str[nonNumericPos] == '\0';
		setterFunction = isNumericString ? &setConfigToString
										 : &setConfigToDouble;
		break;
	}
	case DD_FORCE_OBJECT_TO_LOCALHOST:
	case DD_ASP_STEP_BACK_COUNT:
	case DD_TIME_SERVER_PORT:
	case DD_SIMULATOR_PORT_TCP:
	case DD_SIMULATOR_PORT_UDP:
	case DD_SIMULATOR_MODE:
	case DD_EXTERNAL_SUPERVISOR_PORT_TCP:
	case DD_RVSS_CONFIG:
	case DD_RVSS_RATE:
	case DD_MAX_PACKETS_LOST:
	{
		std::string str(static_cast<const char*>(newValue), newValueSize);
		size_t nonNumericPos = str.find_first_not_of("0123456789+-");
		bool isNumericString = nonNumericPos == std::string::npos || str[nonNumericPos] == '\0';
		setterFunction = isNumericString ? &setConfigToString
										 : &setConfigToUnsignedInteger;
		break;
	}
	case DD_VISUALIZATION_SERVER_NAME:
	case DD_TIME_SERVER_IP:
	case DD_SIMULATOR_IP:
	case DD_EXTERNAL_SUPERVISOR_IP:
		setterFunction = newValueSize == sizeof (in_addr_t) ? &setToIPAddress
															: &setConfigToString;
		break;

	}

	return setterFunction(param, newValue, newValueSize);
}

ReadWriteAccess_t DataDictionaryGet(const enum DataDictionaryParameter param,
									void* result,
									const size_t resultSize) {
	GetterFunctionReference getterFunction = &defaultGetter;

	switch (param) {
	case DD_SCENARIO_NAME:
	case DD_VOIL_RECEIVERS:
	case DD_DTM_RECEIVERS:
	case DD_MISC_DATA:
		getterFunction = &getConfigAsString;
		break;
	case DD_ORIGIN_LATITUDE:
	case DD_ORIGIN_LONGITUDE:
	case DD_ORIGIN_ALTITUDE:
	case DD_ASP_MAX_TIME_DIFF:
	case DD_ASP_MAX_TRAJ_DIFF:
	case DD_ASP_FILTER_LEVEL:
	case DD_ASP_MAX_DELTA_TIME:
		getterFunction = resultSize == sizeof (double) ? &getConfigAsDouble
													   : &getConfigAsString;
		break;
	case DD_FORCE_OBJECT_TO_LOCALHOST:
	case DD_ASP_STEP_BACK_COUNT:
	case DD_TIME_SERVER_PORT:
	case DD_SIMULATOR_PORT_TCP:
	case DD_SIMULATOR_PORT_UDP:
	case DD_SIMULATOR_MODE:
	case DD_EXTERNAL_SUPERVISOR_PORT_TCP:
	case DD_RVSS_CONFIG:
	case DD_RVSS_RATE:
	case DD_MAX_PACKETS_LOST:
		getterFunction = &getConfigAsUnsignedInteger;
		break;
	case DD_VISUALIZATION_SERVER_NAME:
	case DD_TIME_SERVER_IP:
	case DD_SIMULATOR_IP:
	case DD_EXTERNAL_SUPERVISOR_IP:
		getterFunction = resultSize == sizeof (in_addr_t) ? &getAsIPAddress
														  : &getConfigAsString;
		break;
	}
	return getterFunction(param, result, resultSize);
}

ReadWriteAccess_t getConfigAsDouble(const enum DataDictionaryParameter param,
										void *result,
										const size_t resultSize) {
	ReadWriteAccess_t retval = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endptr = nullptr;
	if (resultSize != sizeof (double)) {
		LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", resultSize);
		return OUT_OF_RANGE;
	}
	if (UtilReadConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param),
									   resultBuffer, sizeof (resultBuffer))) {
		double r = std::strtod(resultBuffer, &endptr);
		if (endptr != resultBuffer) {
			retval = READ_OK;
			*static_cast<double*>(result) = r;
		}
		else {
			retval = PARAMETER_NOTFOUND;
			LogMessage(LOG_LEVEL_ERROR, "Cannot cast parameter value <%s> to double",
					   resultBuffer);
		}
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						resultBuffer, sizeof (resultBuffer)));
	}
	return retval;
}

ReadWriteAccess_t getConfigAsUnsignedInteger(
		const enum DataDictionaryParameter param,
		void *result,
		const size_t resultSize) {
	ReadWriteAccess_t retval = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endptr = nullptr;
	if (UtilReadConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param),
									   resultBuffer, sizeof (resultBuffer))) {
		uint64_t r = std::strtoul(resultBuffer, &endptr, 10);
		if (endptr != resultBuffer) {
			retval = OUT_OF_RANGE;
			switch (resultSize) {
			case sizeof (uint8_t):
				if (r <= UINT8_MAX) {
					*static_cast<uint8_t*>(result) = static_cast<uint8_t>(r);
					retval = READ_OK;
				}
				break;
			case sizeof (uint16_t):
				if (r <= UINT16_MAX) {
					*static_cast<uint16_t*>(result) = static_cast<uint16_t>(r);
					retval = READ_OK;
				}
				break;
			case sizeof (uint32_t):
				if (r <= UINT32_MAX) {
					*static_cast<uint32_t*>(result) = static_cast<uint32_t>(r);
					retval = READ_OK;
				}
				break;
			case sizeof (uint64_t):
				if (r <= UINT64_MAX) {
					*static_cast<uint64_t*>(result) = r;
					retval = READ_OK;
				}
				break;
			default:
				LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", resultSize);
				retval = UNDEFINED;
				break;
			}
			if (retval == OUT_OF_RANGE) {
				LogMessage(LOG_LEVEL_ERROR, "Value %u for parameter %s falls outside permitted range of integer parameter with size %u",
						   r, UtilGetConfigurationParameterAsString(
								static_cast<const enum ConfigurationFileParameter>(param),
								resultBuffer, sizeof (resultBuffer)), resultSize);
			}
		}
		else {
			retval = PARAMETER_NOTFOUND;
			LogMessage(LOG_LEVEL_ERROR, "Cannot cast parameter value <%s> to integer",
					   resultBuffer);
		}
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						resultBuffer, sizeof (resultBuffer)));
	}
	return retval;
}


ReadWriteAccess_t getConfigAsString(
		const enum DataDictionaryParameter param,
		void *result,
		const size_t resultSize) {

	ReadWriteAccess_t retval = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_1024];

	if (UtilReadConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param),
									   resultBuffer, sizeof (resultBuffer))) {
		retval = READ_OK;
		strncpy(static_cast<char*>(result), resultBuffer,
				std::min(sizeof (resultBuffer), resultSize));
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						resultBuffer, sizeof (resultBuffer)));
	}
	return retval;
}

ReadWriteAccess_t getConfigAsIPAddress(
		const enum DataDictionaryParameter param,
		void *result,
		const size_t resultSize) {

	ReadWriteAccess_t retval = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_52];
	if (resultSize != sizeof (in_addr_t)) {
		LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", resultSize);
		return OUT_OF_RANGE;
	}
	retval = getConfigAsString(param, resultBuffer, sizeof (resultBuffer));
	if (retval == READ_OK) {
		in_addr_t r;
		if (inet_pton(AF_INET, resultBuffer, &r) > 0) {
			*static_cast<in_addr_t*>(result) = r;
		}
		else {
			char printoutBuffer[DD_CONTROL_BUFFER_SIZE_52];
			LogMessage(LOG_LEVEL_ERROR, "Parameter %s configuration %s cannot be interpreted as an IP address",
					   UtilGetConfigurationParameterAsString(
						   static_cast<const enum ConfigurationFileParameter>(param),
						   printoutBuffer, sizeof (printoutBuffer)), resultBuffer);
			retval = PARAMETER_NOTFOUND;
		}
	}
	return retval;
}

ReadWriteAccess_t defaultGetter(
		const enum DataDictionaryParameter,
		void *,
		const size_t) {
	LogMessage(LOG_LEVEL_ERROR, "No data dictionary function exists for getting selected parameter");
	return PARAMETER_NOTFOUND;
}


ReadWriteAccess_t setConfigToDouble(
		const enum DataDictionaryParameter param,
		const void *newValue,
		const size_t newValueSize) {

	ReadWriteAccess_t retval = UNDEFINED;
	char valueBuffer[DD_CONTROL_BUFFER_SIZE_20];

	if (newValueSize != sizeof (double)) {
		LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", newValueSize);
		return OUT_OF_RANGE;
	}

	snprintf(valueBuffer, sizeof (valueBuffer), "%.15f", *static_cast<const double*>(newValue));
	if (UtilWriteConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param), valueBuffer, sizeof (valueBuffer))) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						valueBuffer, sizeof (valueBuffer)));
	}
	return retval;
}

ReadWriteAccess_t setConfigToUnsignedInteger(
		const enum DataDictionaryParameter param,
		const void *newValue,
		const size_t newValueSize) {

	ReadWriteAccess_t retval = UNDEFINED;
	char valueBuffer[DD_CONTROL_BUFFER_SIZE_20];

	switch (newValueSize) {
	case sizeof (uint8_t):
		snprintf(valueBuffer, sizeof (valueBuffer), "%u", *static_cast<const uint8_t*>(newValue));
		break;
	case sizeof (uint16_t):
		snprintf(valueBuffer, sizeof (valueBuffer), "%u", *static_cast<const uint16_t*>(newValue));
		break;
	case sizeof (uint32_t):
		snprintf(valueBuffer, sizeof (valueBuffer), "%u", *static_cast<const uint32_t*>(newValue));
		break;
	case sizeof (uint64_t):
		snprintf(valueBuffer, sizeof (valueBuffer), "%lu", *static_cast<const uint64_t*>(newValue));
		break;
	default:
		LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", newValueSize);
		return OUT_OF_RANGE;
	}
	if (UtilWriteConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param),
										valueBuffer, sizeof (valueBuffer))) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						valueBuffer, sizeof (valueBuffer)));
	}
	return retval;
}

ReadWriteAccess_t setConfigToString(
		const enum DataDictionaryParameter param,
		const void *newValue,
		const size_t newValueSize) {
	if (!std::all_of(static_cast<const char*>(newValue),
					 static_cast<const char*>(newValue)+newValueSize,
					 [](const char &c){ return std::isprint(c) || c == '\0'; })) {
		std::string str(static_cast<const char*>(newValue), newValueSize);
		LogMessage(LOG_LEVEL_ERROR, "Detected nonprintable character in string %s", str.c_str());
		return UNDEFINED;
	}
	if (UtilWriteConfigurationParameter(static_cast<const enum ConfigurationFileParameter>(param),
										static_cast<const char*>(newValue), newValueSize)) {
		return WRITE_OK;
	}
	else {
		char parameterName[DD_CONTROL_BUFFER_SIZE_52];
		LogMessage(LOG_LEVEL_ERROR, "%s not found!",
				   UtilGetConfigurationParameterAsString(
						static_cast<const enum ConfigurationFileParameter>(param),
						parameterName, sizeof (parameterName)));
		return PARAMETER_NOTFOUND;
	}
}

ReadWriteAccess_t setConfigToIPAddress(
		const enum DataDictionaryParameter param,
		const void *newValue,
		const size_t newValueSize) {

	ReadWriteAccess_t retval = UNDEFINED;
	char valueBuffer[DD_CONTROL_BUFFER_SIZE_52];
	if (newValueSize != sizeof (in_addr_t)) {
		LogMessage(LOG_LEVEL_ERROR, "Parameter size %u invalid", newValueSize);
		return OUT_OF_RANGE;
	}
	if (inet_ntop(AF_INET, newValue, valueBuffer, sizeof (valueBuffer)) != nullptr) {
		if (UtilWriteConfigurationParameter(
					static_cast<const enum ConfigurationFileParameter>(param),
					valueBuffer, sizeof (valueBuffer))) {
			retval = WRITE_OK;
		}
		else {
			LogMessage(LOG_LEVEL_ERROR, "%s not found!",
					   UtilGetConfigurationParameterAsString(
							static_cast<const enum ConfigurationFileParameter>(param),
							valueBuffer, sizeof (valueBuffer)));
			retval = PARAMETER_NOTFOUND;
		}
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Error converting value to IP string");
		retval = UNDEFINED;
	}
	return retval;
}

ReadWriteAccess_t defaultSetter(
		const enum DataDictionaryParameter,
		const void *,
		const size_t) {
	LogMessage(LOG_LEVEL_ERROR, "No data dictionary function exists for setting selected parameter");
	return PARAMETER_NOTFOUND;
}


/*ASPDebug*/
/*!
 * \brief DataDictionarySetRVSSAsp Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPD
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSAsp(GSDType * GSD, ASPType * ASPD) {
	pthread_mutex_lock(&ASPDataMutex);
	GSD->ASPData = *ASPD;
	pthread_mutex_unlock(&ASPDataMutex);
	return WRITE_OK;
}

/*!
 * \brief DataDictionaryGetRVSSAsp Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPD Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetRVSSAsp(GSDType * GSD, ASPType * ASPD) {
	pthread_mutex_lock(&ASPDataMutex);
	*ASPD = GSD->ASPData;
	pthread_mutex_unlock(&ASPDataMutex);
	return READ_OK;
}

/*END ASPDebug*/


/*OBCState*/
/*!
 * \brief DataDictionarySetOBCStateU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param OBCState
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOBCStateU8(GSDType * GSD, OBCState_t OBCState) {
	ReadWriteAccess_t Res;

	Res = WRITE_OK;
	pthread_mutex_lock(&OBCStateMutex);
	GSD->OBCStateU8 = OBCState;
	pthread_mutex_unlock(&OBCStateMutex);
	return Res;
}

/*!
 * \brief DataDictionaryGetOBCStateU8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \return Current object control state according to ::OBCState_t
 */
OBCState_t DataDictionaryGetOBCStateU8(GSDType * GSD) {
	OBCState_t Ret;

	pthread_mutex_lock(&OBCStateMutex);
	Ret = GSD->OBCStateU8;
	pthread_mutex_unlock(&OBCStateMutex);
	return Ret;
}

/*END OBCState*/

/*!
 * \brief DataDictionaryInitObjectData inits a data structure for saving object monr
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitObjectData() {

	int createdMemory;

	objectDataMemory = createSharedMemory(MONR_DATA_FILENAME, 0, sizeof (ObjectDataType), &createdMemory);
	if (objectDataMemory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to create shared monitor data memory");
		return UNDEFINED;
	}
	return createdMemory ? WRITE_OK : READ_OK;
}

/*!
 * \brief DataDictionarySetMonitorData Parses input variable and sets variable to corresponding value
 * \param monitorData Monitor data
 * \param transmitterId requested object transmitterId
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetMonitorData(const uint32_t transmitterId,
											   const ObjectMonitorType * monitorData,
											   const struct timeval * receiveTime) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (monitorData == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (receiveTime == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (transmitterId == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (objectDataMemory[i].ClientID == transmitterId) {
			objectDataMemory[i].MonrData = *monitorData;
			objectDataMemory[i].lastDataUpdate = *receiveTime;
			result = WRITE_OK;
		}
	}

	if (result == PARAMETER_NOTFOUND) {
		// Search for unused memory space and place monitor data there
		LogMessage(LOG_LEVEL_INFO, "Received first monitor data from transmitter ID %u", transmitterId);
		for (int i = 0; i < numberOfObjects; ++i) {
			if (objectDataMemory[i].ClientID == 0) {
				objectDataMemory[i].MonrData = *monitorData;
				objectDataMemory[i].lastDataUpdate = *receiveTime;
				result = WRITE_OK;
			}
		}

		// No uninitialized memory space found - create new
		if (result == PARAMETER_NOTFOUND) {
			objectDataMemory = resizeSharedMemory(objectDataMemory, (unsigned int)(numberOfObjects + 1));
			if (objectDataMemory != NULL) {
				numberOfObjects = getNumberOfMemoryElements(objectDataMemory);
				LogMessage(LOG_LEVEL_INFO,
						   "Modified shared memory to hold monitor data for %u objects", numberOfObjects);
				objectDataMemory[numberOfObjects - 1].MonrData = *monitorData;
				objectDataMemory[numberOfObjects - 1].lastDataUpdate = *receiveTime;
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Error resizing shared memory");
				result = UNDEFINED;
			}
		}
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionaryClearObjectData Clears existing object data tagged with
 *			a certain transmitter ID.
 * \param transmitterID Transmitter ID of the monitor data to be cleared.
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryClearObjectData(const uint32_t transmitterID) {
	ReadWriteAccess_t result = PARAMETER_NOTFOUND;

	if (transmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (objectDataMemory[i].ClientID == transmitterID) {
			memset(&objectDataMemory[i], 0, sizeof (objectDataMemory));
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	if (result == PARAMETER_NOTFOUND) {
		LogMessage(LOG_LEVEL_WARNING, "Unable to find object data for transmitter ID %u", transmitterID);
	}

	return result;
}

/*!
 * \brief DataDictionaryGetMonitorData Reads variable from shared memory
 * \param transmitterId requested object transmitterId
 * \param monitorData Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetMonitorData(const uint32_t transmitterId, ObjectMonitorType * monitorData) {
	ReadWriteAccess_t result = PARAMETER_NOTFOUND;

	if (monitorData == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (transmitterId == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (objectDataMemory[i].ClientID == transmitterId) {
			memcpy(monitorData, &objectDataMemory[i].MonrData, sizeof (ObjectMonitorType));

			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	if (result == PARAMETER_NOTFOUND) {
		LogMessage(LOG_LEVEL_WARNING, "Unable to find monitor data for transmitter ID %u", transmitterId);
	}

	return result;
}

/*!
 * \brief DataDictionaryGetMonitorDataReceiveTime Gets the last receive time of monitor data for specified object
 * \param transmitterID Identifier of object
 * \param lastDataUpdate Return variable pointer
 * \return Value according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetMonitorDataReceiveTime(const uint32_t transmitterID,
														  struct timeval * lastDataUpdate) {
	ReadWriteAccess_t result = UNDEFINED;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (transmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	lastDataUpdate->tv_sec = 0;
	lastDataUpdate->tv_usec = 0;

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			*lastDataUpdate = objectDataMemory[i].lastDataUpdate;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionarySetMonitorDataReceiveTime Sets the last receive time of monitor data for specified object
 * \param transmitterID Identifier of object
 * \param lastDataUpdate Time to set
 * \return Value according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetMonitorDataReceiveTime(const uint32_t transmitterID,
														  const struct timeval * lastDataUpdate) {
	ReadWriteAccess_t result = UNDEFINED;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (transmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			objectDataMemory[i].lastDataUpdate = *lastDataUpdate;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionaryFreeObjectData Releases data structure for saving object monitor data
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryFreeObjectData() {
	ReadWriteAccess_t result = WRITE_OK;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Attempt to free uninitialized memory");
		return UNDEFINED;
	}

	destroySharedMemory(objectDataMemory);

	return result;
}

/*END of MONR*/


/*NbrOfObjects*/
/*!
 * \brief DataDictionarySetNumberOfObjects Sets the number of objects to the specified value and clears all
 *			monitor data currently present in the system
 * \param numberOfobjects number of objects
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetNumberOfObjects(const uint32_t newNumberOfObjects) {

	unsigned int numberOfObjects;
	ReadWriteAccess_t result = WRITE_OK;

	objectDataMemory = claimSharedMemory(objectDataMemory);
	objectDataMemory = resizeSharedMemory(objectDataMemory, newNumberOfObjects);
	numberOfObjects = getNumberOfMemoryElements(objectDataMemory);
	objectDataMemory = releaseSharedMemory(objectDataMemory);

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Error resizing shared memory");
		return UNDEFINED;
	}

	return result;
}

/*!
 * \brief DataDictionaryGetNumberOfObjects Reads variable from shared memory
 * \param numberOfobjects number of objects in a test
 * \return Number of objects present in memory
 */
ReadWriteAccess_t DataDictionaryGetNumberOfObjects(uint32_t * numberOfObjects) {
	int retval;

	objectDataMemory = claimSharedMemory(objectDataMemory);
	retval = getNumberOfMemoryElements(objectDataMemory);
	objectDataMemory = releaseSharedMemory(objectDataMemory);
	*numberOfObjects = retval == -1 ? 0 : (uint32_t) retval;
	return retval == -1 ? UNDEFINED : READ_OK;
}

/*END of NbrOfObjects*/

/*!
 * \brief DataDictionaryGetNumberOfObjects Reads variable from shared memory
 * \param numberOfobjects number of objects in a test
 * \return Number of objects present in memory
 */
ReadWriteAccess_t DataDictionaryGetObjectTransmitterIDs(uint32_t transmitterIDs[], const uint32_t arraySize) {
	int32_t retval;

	if (transmitterIDs == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Data dictionary input pointer error");
		return UNDEFINED;
	}
	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Data dictionary monitor data read error");
		return UNDEFINED;
	}

	memset(transmitterIDs, 0, arraySize * sizeof (transmitterIDs[0]));
	objectDataMemory = claimSharedMemory(objectDataMemory);
	retval = getNumberOfMemoryElements(objectDataMemory);
	if (retval == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Error reading number of objects from shared memory");
		objectDataMemory = releaseSharedMemory(objectDataMemory);
		return UNDEFINED;
	}
	else if ((uint32_t) retval > arraySize) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to list transmitter IDs in specified array");
		objectDataMemory = releaseSharedMemory(objectDataMemory);
		return UNDEFINED;
	}

	for (int i = 0; i < retval; ++i) {
		transmitterIDs[i] = objectDataMemory[i].ClientID;
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return READ_OK;
}


/*!
 * \brief DataDictionarySetObjectData 
 * \param objectData data to be initialized
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetObjectData(const ObjectDataType * objectData) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (objectData == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (objectData->ClientID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	if (result == PARAMETER_NOTFOUND) {
		// Search for unused memory space and place monitor data there
		LogMessage(LOG_LEVEL_INFO, "First Object Information data from added with ID %u",
				   objectData->ClientID);
		for (int i = 0; i < numberOfObjects; ++i) {
			if (objectDataMemory[i].ClientID == objectData->ClientID) {
				memcpy(&objectDataMemory[i], objectData, sizeof (ObjectDataType));
				result = WRITE_OK;
			}
		}

		// No uninitialized memory space found - create new
		if (result == PARAMETER_NOTFOUND) {
			objectDataMemory = resizeSharedMemory(objectDataMemory, (unsigned int)(numberOfObjects + 1));
			if (objectDataMemory != NULL) {
				numberOfObjects = getNumberOfMemoryElements(objectDataMemory);
				LogMessage(LOG_LEVEL_INFO,
						   "Modified shared memory to hold monitor data for %u objects", numberOfObjects);
				memcpy(&objectDataMemory[numberOfObjects - 1], objectData, sizeof (ObjectDataType));
			}
			else {
				LogMessage(LOG_LEVEL_ERROR, "Error resizing shared memory");
				result = UNDEFINED;
			}
		}
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);


	if (result != PARAMETER_NOTFOUND)

		objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}




/*!
 * \brief DataDictionarySetObjectEnableStatus sets the object enable status
 * \param transmitterId requested object transmitterId
 * \param enabledStatus the enable status - enable, disable, undefined
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetObjectEnableStatus(const uint32_t transmitterId,
													  ObjectEnabledType enabledStatus) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (transmitterId == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterId == objectDataMemory[i].ClientID) {
			objectDataMemory[i].Enabled = enabledStatus;
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionaryGetObjectEnableStatusById 
 * \param transmitterId requested object transmitterId
 * \param *enabledStatus Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusById(const uint32_t transmitterId,
														  ObjectEnabledType * enabledStatus) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (transmitterId == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	*enabledStatus = OBJECT_UNDEFINED;

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterId == objectDataMemory[i].ClientID) {
			*enabledStatus = objectDataMemory[i].Enabled;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionaryGetObjectEnableStatusByIp 
 * \param ClientIP requested object IP number
 * \param *enabledStatus Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetObjectEnableStatusByIp(const in_addr_t ClientIP,
														  ObjectEnabledType * enabledStatus) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (ClientIP == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	*enabledStatus = OBJECT_UNDEFINED;

	for (int i = 0; i < numberOfObjects; ++i) {
		if (ClientIP == objectDataMemory[i].ClientIP) {
			*enabledStatus = objectDataMemory[i].Enabled;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}


/*!
 * \brief DataDictionaryGetObjectTransmitterIDByIP
 * \param ClientIP requested object IP number
 * \param *transmitterId Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetObjectTransmitterIDByIP(const in_addr_t ClientIP, uint32_t * transmitterId) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (ClientIP == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Unable to get transmitter ID for IP 0.0.0.0");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	*transmitterId = 0;

	for (int i = 0; i < numberOfObjects; ++i) {
		if (ClientIP == objectDataMemory[i].ClientIP) {
			*transmitterId = objectDataMemory[i].ClientID;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}


/*!
 * \brief DataDictionaryGetTransmitterIdByIP
 * \param transmitterID requested object ID
 * \param *ClientIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetObjectIPByTransmitterID(const in_addr_t transmitterID,
														   in_addr_t * ClientIP) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (transmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	*ClientIP = 0;

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			*ClientIP = objectDataMemory[i].ClientIP;
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/*!
 * \brief DataDictionaryModifyTransmitterID Changes the transmitter ID of the object data identified by a transmitter ID
 * \param oldTransmitterID Present transmitter ID of object data
 * \param newTransmitterID Desired new transmitter ID of object data
 * \return Value according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryModifyTransmitterID(const uint32_t oldTransmitterID,
													const uint32_t newTransmitterID) {
	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (newTransmitterID == 0 || oldTransmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	if (objectDataMemory == NULL) {
		// If this code executes, objectDataMemory has been reallocated outside of DataDictionary
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	result = PARAMETER_NOTFOUND;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (oldTransmitterID == objectDataMemory[i].ClientID) {
			objectDataMemory[i].ClientID = newTransmitterID;
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

