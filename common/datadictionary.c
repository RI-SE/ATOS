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
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include "datadictionary.h"
#include "logging.h"
#include "shmem.h"


// Parameters and variables
#define MONR_DATA_FILENAME "MonitorData"
#define STATE_DATA_FILENAME "StateData"
#define RVSSASP_DATA_FILENAME "RvssAspData"

typedef struct {
	OBCState_t objectControlState;
} StateDataType;

static volatile ObjectDataType *objectDataMemory = NULL;
static volatile StateDataType *stateDataMemory = NULL;
static volatile ASPType *rvssAspDataMemory = NULL; 

/*------------------------------------------------------------
  -- Static function definitions
  ------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Functions
  ------------------------------------------------------------*/


/*!
 * \brief DataDictionaryConstructor Initialize data held by DataDictionary.
Initialization data that is configurable is stored in test.conf.
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryConstructor() {
	ReadWriteAccess_t result = READ_WRITE_OK;
	ReadWriteAccess_t retval = READ_WRITE_OK;

	// Configuration file parameters should always exist
	if ((result = DataDictionaryInitScenarioName()) != READ_OK) {
		retval = READ_FAIL;
	}
	if ((result = DataDictionaryInitMaxPacketsLost()) != READ_OK) {
		retval = READ_FAIL;
	}
	if ((result = DataDictionaryInitTransmitterID()) != READ_OK) {
		retval = READ_FAIL;
	}

	// Shared memory variables can either be preexisting or not
	if (DataDictionaryInitObjectData() != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting shared monitor data memory found by constructor");
		retval = WRITE_FAIL;
	}
	if (DataDictionaryInitStateData() != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting shared state memory found by constructor");
		retval = WRITE_FAIL;
	}
	else {
		DataDictionarySetOBCState(OBC_STATE_UNDEFINED);
	}

	if (DataDictionaryInitRVSSAsp() != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting shared ASP memory found by constructor");
		retval = WRITE_FAIL;
	}

	return retval;
}


/*!
 * \brief DataDictionaryDestructor Deallocate data held by DataDictionary.
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryDestructor() {
	ReadWriteAccess_t result = WRITE_OK;
	ReadWriteAccess_t retval = WRITE_OK;

	if ((result = DataDictionaryFreeObjectData()) != WRITE_OK) {
		retval = WRITE_FAIL;
	}
	if ((result = DataDictionaryFreeStateData()) != WRITE_OK) {
		retval = WRITE_FAIL;
	}
	if ((result = DataDictionaryFreeRVSSAsp()) != WRITE_OK) {
		retval = WRITE_FAIL;
	}
	return retval;
}


ReadWriteAccess_t DataDictionaryInitScenarioName() {
	ReadWriteAccess_t Res = UNDEFINED;
	char ResultBufferC8[DD_CONTROL_BUFFER_SIZE_1024];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_SCENARIO_NAME, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ScenarioName not found!");
	}
	return Res;
}

ReadWriteAccess_t DataDictionarySetScenarioName(const char *name, const size_t nameLength) {
	if (UtilWriteConfigurationParameter(CONFIGURATION_PARAMETER_SCENARIO_NAME, name, nameLength)) {
		return WRITE_OK;
	}
	else {
		return PARAMETER_NOTFOUND;
	}
}

ReadWriteAccess_t DataDictionaryGetScenarioName(char *name, const size_t nameLength) {
	ReadWriteAccess_t Res = UNDEFINED;

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_SCENARIO_NAME, name, nameLength)) {
		Res = READ_OK;
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ScenarioName not found!");
	}
	return Res;
}

/*!
 * \brief DataDictionarySetOriginLatitudeDbl Parses input variable and sets variable to corresponding value
 * \param Latitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginLatitudeDbl(const char* latitude) {
	if (latitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LATITUDE, latitude, strlen(latitude) + 1)) {
		return WRITE_OK;
	}
	else {
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetOriginLatitudeDbl Reads variable from shared memory
 * \param Latitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLatitudeDbl(double_t* latitude) {
	char readValue[DD_CONTROL_BUFFER_SIZE_20];
	char* endptr;
	ReadWriteAccess_t retval;
	if (latitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetOriginLatitudeString(readValue, sizeof (readValue))) != READ_OK) {
		return retval;
	}

	*latitude = strtod(readValue, &endptr);

	if (endptr == readValue) {
		*latitude = 0;
		LogMessage(LOG_LEVEL_ERROR, "Latitude badly formatted");
		return PARAMETER_NOTFOUND;
	}
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginLatitudeString Reads variable from shared memory
 * \param Latitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLatitudeString(char* latitude, const size_t bufferLength) {
	if (latitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LATITUDE, latitude, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Latitude not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of Origin Latitude*/

/*Origin Longitude*/
/*!
 * \brief DataDictionarySetOriginLongitudeDbl Parses input variable and sets variable to corresponding value
 * \param Longitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginLongitudeDbl(const char* longitude) {
	if (longitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE, longitude, strlen(longitude) + 1)) {
		return WRITE_OK;
	}
	else {
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetOriginLongitudeDbl Reads variable from shared memory
 * \param Longitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLongitudeDbl(double_t* longitude) {
	char readValue[DD_CONTROL_BUFFER_SIZE_20];
	char* endptr;
	ReadWriteAccess_t retval;
	if (longitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetOriginLongitudeString(readValue, sizeof (readValue))) != READ_OK) {
		return retval;
	}

	*longitude = strtod(readValue, &endptr);

	if (endptr == readValue) {
		*longitude = 0;
		LogMessage(LOG_LEVEL_ERROR, "Longitude badly formatted");
		return PARAMETER_NOTFOUND;
	}
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginLongitudeString Reads variable from shared memory
 * \param Longitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLongitudeString(char* longitude, const size_t bufferLength) {
	if (longitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE, longitude, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Longitude not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of Origin Longitude*/

/*Origin Altitude*/
/*!
 * \brief DataDictionarySetOriginAltitudeDbl Parses input variable and sets variable to corresponding value
 * \param Altitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginAltitudeDbl(const char* altitude) {
	if (altitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE, altitude, strlen(altitude) + 1)) {
		return WRITE_OK;
	}
	else {
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetOriginAltitudeDbl Reads variable from shared memory
 * \param Altitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginAltitudeDbl(double_t* altitude) {
	char readValue[DD_CONTROL_BUFFER_SIZE_20];
	char* endptr;
	ReadWriteAccess_t retval;
	if (altitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetOriginAltitudeString(readValue, sizeof (readValue))) != READ_OK) {
		return retval;
	}

	*altitude = strtod(readValue, &endptr);

	if (endptr == readValue) {
		*altitude = 0;
		LogMessage(LOG_LEVEL_ERROR, "Altitude badly formatted");
		return PARAMETER_NOTFOUND;
	}
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginAltitudeString Reads variable from shared memory
 * \param Altitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginAltitudeString(char* altitude, const size_t bufferLength) {
	if (altitude == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE, altitude, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Altitude not found!");
		return PARAMETER_NOTFOUND;
	}
}
/*END of Origin Altitude*/

/*VisualizationServer*/
/*!
 * \brief DataDictionarySetVisualizationServerU32 Parses input variable and sets variable to corresponding value
 * \param IP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetVisualizationServerU32(const char* IP) {
	ReadWriteAccess_t retval;
	int result;
	in_addr_t inaddr;

	if (IP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	result = inet_pton(AF_INET, IP, &inaddr);
	if (result <= 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Specified IP %s is not valid", IP);
		return WRITE_FAIL;
	}
	
	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME, IP, strlen(IP) + 1)) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetVisualizationServerU32 Reads variable from shared memory
 * \param IP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVisualizationServerU32(in_addr_t* IP) {
	char ipString[INET_ADDRSTRLEN];
	ReadWriteAccess_t retval;
	int result;

	if (IP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetVisualizationServerIPString(ipString, sizeof (ipString))) != READ_OK) {
		return retval;
	}
	result = inet_pton(AF_INET, ipString, IP);
	if (result > 0) {
		retval = READ_OK;
	}
	else if (result == 0) {
		LogMessage(LOG_LEVEL_ERROR, "VisualizationServerIP string %s is not a valid IPv4 address", ipString);
		retval = PARAMETER_NOTFOUND;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
		retval = UNDEFINED;
	}
	return retval;
}


/*!
 * \brief DataDictionaryGetVisualizationServerIPString Reads variable from shared memory
 * \param IP Return variable pointer
 * \param bufferLength Size of return variable buffer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVisualizationServerIPString(char* IP, const size_t bufferLength) {
	if (IP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME,
				IP, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "VisualizationServerIP not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of VisualizationServer*/

/*ASPMaxTimeDiff*/

/*!
 * \brief DataDictionarySetASPMaxTimeDiffDbl Parses input variable and sets variable to corresponding value
  * \param ASPMaxTimeDiff
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxTimeDiffDbl(const char * ASPMaxTimeDiff) {
	ReadWriteAccess_t Res;


	if (ASPMaxTimeDiff == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF, ASPMaxTimeDiff, strlen(ASPMaxTimeDiff) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxTimeDiffDbl Reads variable from shared memory
  * \param ASPMaxTimeDiff Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxTimeDiffDbl(double_t * ASPMaxTimeDiff) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	double_t readSetting;

	if (ASPMaxTimeDiff == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtod(resultBuffer, &endPtr);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for ASP max time diff configuration");
			result = PARAMETER_NOTFOUND;
			*ASPMaxTimeDiff = 0;
		}
		else {
			result = READ_OK;
			*ASPMaxTimeDiff = (double_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "ASP max time diff configuration not found!");
		result = PARAMETER_NOTFOUND;
		*ASPMaxTimeDiff = 0;
	}

}

/*END of ASPMaxTimeDiff*/

/*ASPMaxTrajDiff*/

/*!
 * \brief DataDictionarySetASPMaxTrajDiffDbl Parses input variable and sets variable to corresponding value
 * \param ASPMaxTrajDiff
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxTrajDiffDbl(const char * ASPMaxTrajDiff) {
	ReadWriteAccess_t Res;

	if (ASPMaxTrajDiff == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF, ASPMaxTrajDiff, strlen(ASPMaxTrajDiff) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxTrajDiffDbl Reads variable from shared memory
 * \param ASPMaxTrajDiff Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxTrajDiffDbl(double_t * ASPMaxTrajDiff) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	double_t readSetting;

	if (ASPMaxTrajDiff == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtod(resultBuffer, &endPtr);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for ASP max traj diff configuration");
			result = PARAMETER_NOTFOUND;
			*ASPMaxTrajDiff = 0;
		}
		else {
			result = READ_OK;
			*ASPMaxTrajDiff = (double_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "ASP max traj diff configuration not found!");
		result = PARAMETER_NOTFOUND;
		*ASPMaxTrajDiff = 0;
	}
}

/*END of ASPMaxTrajDiff*/


/*ASPStepBackCount*/
/*!
 * \brief DataDictionarySetASPStepBackCountU32 Parses input variable and sets variable to corresponding value
 * \param ASPStepBackCount
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPStepBackCountU32(const char * ASPStepBackCount) {
	ReadWriteAccess_t Res;

	if (ASPStepBackCount == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT, ASPStepBackCount, strlen(ASPStepBackCount) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPStepBackCountU32 Reads variable from shared memory
 * \param ASPStepBackCount Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPStepBackCountU32(uint32_t * ASPStepBackCount) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	uint64_t readSetting;

	if (ASPStepBackCount == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtoul(resultBuffer, &endPtr, 10);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for ASP step back count configuration");
			result = PARAMETER_NOTFOUND;
			*ASPStepBackCount = 0;
		}
		else {
			result = READ_OK;
			*ASPStepBackCount = (uint32_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "RASP step back count not found!");
		result = PARAMETER_NOTFOUND;
		*ASPStepBackCount = 0;
	}
}

/*END of ASPStepBackCount*/


/*ASPFilterLevel*/
/*!
 * \brief DataDictionarySetASPFilterLevelDbl Parses input variable and sets variable to corresponding value
 * \param ASPFilterLevel
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPFilterLevelDbl(const char * ASPFilterLevel) {
	ReadWriteAccess_t Res;

	if (ASPFilterLevel == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL, ASPFilterLevel, strlen(ASPFilterLevel) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPFilterLevelDbl Reads variable from shared memory
 * \param ASPFilterLevel Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPFilterLevelDbl(dbl * ASPFilterLevel) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	double_t readSetting;

	if (ASPFilterLevel == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtod(resultBuffer, &endPtr);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for ASP filter level configuration");
			result = PARAMETER_NOTFOUND;
			*ASPFilterLevel = 0;
		}
		else {
			result = READ_OK;
			*ASPFilterLevel = (double_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "ASP filter level configuration not found!");
		result = PARAMETER_NOTFOUND;
		*ASPFilterLevel = 0;
	}
}

/*END of ASPFilterLevel*/

/*ASPMaxDeltaTime*/

/*!
 * \brief DataDictionarySetASPMaxDeltaTimeDbl Parses input variable and sets variable to corresponding value
 * \param ASPMaxDeltaTime
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxDeltaTimeDbl(const char * ASPMaxDeltaTime) {
	ReadWriteAccess_t Res;

	if (ASPMaxDeltaTime == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME, ASPMaxDeltaTime, strlen(ASPMaxDeltaTime) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxDeltaTimeDbl Reads variable from shared memory
 * \param ASPMaxDeltaTime Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxDeltaTimeDbl(double_t * ASPMaxDeltaTime) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	double_t readSetting;

	if (ASPMaxDeltaTime == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtod(resultBuffer, &endPtr);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for ASP max delta time configuration");
			result = PARAMETER_NOTFOUND;
			*ASPMaxDeltaTime = 0;
		}
		else {
			result = READ_OK;
			*ASPMaxDeltaTime = (double_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "ASP max delta time configuration not found!");
		result = PARAMETER_NOTFOUND;
		*ASPMaxDeltaTime = 0;
	}
}

/*END of ASPFilterLevel*/

/*!
 * \brief DataDictionarySetTimeServerIPU32 Parses input variable and sets variable to corresponding value
 * \param TimeServerIP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetTimeServerIPU32(const char* timeServerIP) {
	ReadWriteAccess_t retval;
	int result;
	in_addr_t inaddr;

	if (timeServerIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	result = inet_pton(AF_INET, timeServerIP, &inaddr);
	if (result <= 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Specified IP %s is not valid", timeServerIP);
		return WRITE_FAIL;
	}
	
	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_TIME_SERVER_IP, timeServerIP, strlen(timeServerIP) + 1)) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetTimeServerIPU32 Reads variable from shared memory
 * \param TimeServerIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(in_addr_t* timeServerIP) {
	char ipString[INET_ADDRSTRLEN];
	ReadWriteAccess_t retval;
	int result;

	if (timeServerIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetTimeServerIPString(ipString, sizeof (ipString))) != READ_OK) {
		return retval;
	}
	result = inet_pton(AF_INET, ipString, timeServerIP);
	if (result > 0) {
		retval = READ_OK;
	}
	else if (result == 0) {
		LogMessage(LOG_LEVEL_ERROR, "TimeServerIP string %s is not a valid IPv4 address", ipString);
		retval = PARAMETER_NOTFOUND;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
		retval = UNDEFINED;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetTimeServerIPC8 Reads variable from shared memory
 * \param TimeServerIP Return variable pointer
 * \param bufferLength Size of return variable buffer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerIPString(char * timeServerIP, const size_t bufferLength) {

	if (timeServerIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_TIME_SERVER_IP,
				timeServerIP, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "TimeServerIP not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of TimeServerIP*/

/*!
 * \brief DataDictionarySetTimeServerPortU16 Parses input variable and sets variable to corresponding value
 * \param TimeServerPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetTimeServerPortU16(const char* timeServerPort) {

	if (timeServerPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_TIME_SERVER_PORT, timeServerPort, strlen(timeServerPort) + 1)) {
		return WRITE_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "TimeServerPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetTimeServerPortU16 Reads variable from shared memory
 * \param timeServerPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(uint16_t * timeServerPort) {
	char resultBuffer[10];
	char *endptr = NULL;

	if (timeServerPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_TIME_SERVER_PORT,
				resultBuffer, sizeof(resultBuffer)) > 0) {
		*timeServerPort = strtoul(resultBuffer, &endptr, 10);
		if (endptr == resultBuffer) {
			*timeServerPort = 0;
			LogMessage(LOG_LEVEL_ERROR, "TimeServerPort badly formatted");
			return PARAMETER_NOTFOUND;
		}
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "TimeServerPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of TimeServerPort*/


/*SimulatorIP*/

/*!
 * \brief DataDictionarySetSimulatorIPU32 Parses input variable and sets variable to corresponding value
 * \param SimulatorIP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorIPU32(const char* simulatorIP) {
	ReadWriteAccess_t retval;
	int result;
	in_addr_t inaddr;

	if (simulatorIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	result = inet_pton(AF_INET, simulatorIP, &inaddr);
	if (result <= 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Specified IP %s is not valid", simulatorIP);
		return WRITE_FAIL;
	}
	
	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_IP, simulatorIP, strlen(simulatorIP) + 1)) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetSimulatorIPU32 Reads variable from shared memory
 * \param SimulatorIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorIPU32(in_addr_t* simulatorIP) {
	char ipString[INET_ADDRSTRLEN];
	ReadWriteAccess_t retval;
	int result;

	if (simulatorIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetSimulatorIPString(ipString, sizeof (ipString))) != READ_OK) {
		return retval;
	}
	result = inet_pton(AF_INET, ipString, simulatorIP);
	if (result > 0) {
		retval = READ_OK;
	}
	else if (result == 0) {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorIP string %s is not a valid IPv4 address", ipString);
		retval = PARAMETER_NOTFOUND;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
		retval = UNDEFINED;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetSimulatorIPString Reads variable from shared memory
 * \param SimulatorIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorIPString(char* simulatorIP, const size_t bufferLength) {
	if (simulatorIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_SIMULATOR_IP,
				simulatorIP, bufferLength) > 0) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorIP not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of SimulatorIP*/

/*SimulatorTCPPort*/

/*!
 * \brief DataDictionarySetSimulatorTCPPortU16 Parses input variable and sets variable to corresponding value
 * \param SimulatorTCPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorTCPPortU16(const char* simulatorTCPPort) {
	if (simulatorTCPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP, simulatorTCPPort, strlen(simulatorTCPPort) + 1)) {
		return WRITE_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetSimulatorTCPPortU16 Reads variable from shared memory
 * \param SimulatorTCPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorTCPPortU16(uint16_t* simulatorTCPPort) {
	char resultBuffer[10];
	char *endptr = NULL;

	if (simulatorTCPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP,
				resultBuffer, sizeof(resultBuffer)) > 0) {
		*simulatorTCPPort = strtoul(resultBuffer, &endptr, 10);
		if (endptr == resultBuffer) {
			*simulatorTCPPort = 0;
			LogMessage(LOG_LEVEL_ERROR, "SimulatorPort badly formatted");
			return PARAMETER_NOTFOUND;
		}
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of SimulatorTCPPort*/

/*SimulatorUDPPort*/
/*!
 * \brief DataDictionarySetSimulatorUDPPortU16 Parses input variable and sets variable to corresponding value
 * \param SimulatorUDPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorUDPPortU16(const char* simulatorUDPPort) {
	if (simulatorUDPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP, simulatorUDPPort, strlen(simulatorUDPPort) + 1)) {
		return WRITE_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*!
 * \brief DataDictionaryGetSimulatorUDPPortU16 Reads variable from shared memory
 * \param SimulatorUDPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorUDPPortU16(uint16_t* simulatorUDPPort) {
	
	char resultBuffer[10];
	char *endptr = NULL;

	if (simulatorUDPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP,
				resultBuffer, sizeof(resultBuffer)) > 0) {
		*simulatorUDPPort = strtoul(resultBuffer, &endptr, 10);
		if (endptr == resultBuffer) {
			*simulatorUDPPort = 0;
			LogMessage(LOG_LEVEL_ERROR, "SimulatorPort badly formatted");
			return PARAMETER_NOTFOUND;
		}
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of SimulatorUDPPort*/

/*SimulatorMode*/
/*!
 * \brief DataDictionarySetSimulatorModeU8 Parses input variable and sets variable to corresponding value
 * \param SimulatorMode
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorModeU8(const char* simulatorMode) {
	ReadWriteAccess_t retval;

	if (simulatorMode == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	
	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_MODE, simulatorMode, strlen(simulatorMode) + 1)) {
		retval = WRITE_OK;
	}
	else {
		retval = PARAMETER_NOTFOUND;
	}
	return retval;
}

/*!
 * \brief DataDictionaryGetSimulatorModeU8 Reads variable from shared memory
 * \param SimulatorMode Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorModeU8(uint8_t* simulatorMode) {
	char readValue[10];
	char* endptr;

	if (simulatorMode == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(
		CONFIGURATION_PARAMETER_SIMULATOR_MODE, readValue, sizeof (readValue)) > 0) {
		*simulatorMode = strtoul(readValue, &endptr, 10);
		if (endptr == readValue) {
			*simulatorMode = 0;
			LogMessage(LOG_LEVEL_ERROR, "SimulatorMode badly formatted");
			return PARAMETER_NOTFOUND;
		}
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "SimulatorMode not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of SimulatorMode*/

/*VOILReceivers*/

/*!
 * \brief DataDictionarySetVOILReceiversString Parses input variable and sets variable to corresponding value

 * \param VOILReceivers
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetVOILReceiversString(const char * VOILReceivers) {
	ReadWriteAccess_t Res;

	if (VOILReceivers == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}
	

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_VOIL_RECEIVERS, VOILReceivers, strlen(VOILReceivers) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetVOILReceiversString Reads variable from shared memory
 * \param VOILReceivers Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVOILReceiversString(char * VOILReceivers, const size_t buflen) {
	ReadWriteAccess_t Res;

	if (VOILReceivers == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_VOIL_RECEIVERS, VOILReceivers, buflen)) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "VOIL receivers not found!");
		Res = PARAMETER_NOTFOUND;
		memset(VOILReceivers, 0, buflen);
	}

	return Res;
}

/*END of VOILReceivers*/

/*DTMReceivers*/

/*!
 * \brief DataDictionarySetDTMReceiversString Parses input variable and sets variable to corresponding value
 * \param DTMReceivers
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetDTMReceiversString(const char * DTMReceivers) {
	ReadWriteAccess_t Res;

	if (DTMReceivers == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}
	

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_DTM_RECEIVERS, DTMReceivers, strlen(DTMReceivers) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetDTMReceiversString Reads variable from shared memory
 * \param DTMReceivers Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetDTMReceiversString(char* DTMReceivers, const size_t buflen) {
	ReadWriteAccess_t Res;

	if (DTMReceivers == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_DTM_RECEIVERS, DTMReceivers, buflen)) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "DTM receivers receivers not found!");
		Res = PARAMETER_NOTFOUND;
		memset(DTMReceivers, 0, buflen);
	}

	return Res;
}

/*END of DTMReceivers*/

/*External Supervisor IP*/

/*!
 * \brief DataDictionarySetExternalSupervisorIPU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param IP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetExternalSupervisorIPU32(const char* IP) {
	ReadWriteAccess_t Res;

	if (IP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if(UtilWriteConfigurationParameter(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP, IP, strlen(IP) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;

}

/*!
 * \brief DataDictionaryGetExternalSupervisorIPU32 Reads variable from shared memory
 * \param externalSupervisorIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPU32(in_addr_t* externalSupervisorIP) {
	char ipString[INET_ADDRSTRLEN];
	ReadWriteAccess_t retval;
	int result;

	if (externalSupervisorIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}

	if ((retval = DataDictionaryGetExternalSupervisorIPString(ipString, sizeof (ipString))) != READ_OK) {
		return retval;
	}
	result = inet_pton(AF_INET, ipString, externalSupervisorIP);
	if (result > 0) {
		retval = READ_OK;
	}
	else if (result == 0) {
		LogMessage(LOG_LEVEL_ERROR, "External supervisor string %s is not a valid IPv4 address", ipString);
		retval = PARAMETER_NOTFOUND;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
		retval = UNDEFINED;
	}
	return retval;
}


/*!
 * \brief DataDictionaryGetExternalSupervisorIPString Reads variable from shared memory
 * \param externalSupervisorIP Return variable pointer
 * \param buflen Return parameter buffer length
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPString(char * externalSupervisorIP, uint32_t buflen) {
	ReadWriteAccess_t Res;

	if (externalSupervisorIP == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP, externalSupervisorIP, buflen)) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "External supervisor IP not found!");
		Res = PARAMETER_NOTFOUND;
		memset(externalSupervisorIP, 0, buflen);
	}

	return Res;
}

/*END of External Supervisor IP*/

/*External SupervisorTCPPort*/

/*!
 * \brief DataDictionarySetSupervisorTCPPortU16 Parses input variable and sets variable to corresponding value
 * \param SupervisorTCPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSupervisorTCPPortU16(const char * SupervisorTCPPort) {
	ReadWriteAccess_t Res;


	if (SupervisorTCPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}
	

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP, SupervisorTCPPort,
		 strlen(SupervisorTCPPort) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSupervisorTCPPortU16 Reads variable from shared memory
 * \param SupervisorTCPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSupervisorTCPPortU16(uint16_t * SupervisorTCPPort) {
	char resultBuffer[10];
	char *endptr = NULL;

	if (SupervisorTCPPort == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP,
				resultBuffer, sizeof(resultBuffer)) > 0) {
		*SupervisorTCPPort = strtoul(resultBuffer, &endptr, 10);
		if (endptr == resultBuffer) {
			*SupervisorTCPPort = 0;
			LogMessage(LOG_LEVEL_ERROR, "External SupervisorTCPPort badly formatted");
			return PARAMETER_NOTFOUND;
		}
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "External SupervisorTCPPort not found!");
		return PARAMETER_NOTFOUND;
	}
}

/*END of External SupervisorTCPPort*/

/*Runtime Variable Subscription Service (RVSS) Configuration*/

/*!
 * \brief DataDictionarySetInitRVSSConfigU32 Parses input variable and sets variable to corresponding value
 * \param RVSSConfig
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSConfigU32(uint32_t RVSSConfig) {
	ReadWriteAccess_t Res;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	memset(ResultBufferC8, 0, DD_CONTROL_BUFFER_SIZE_20);
	sprintf(ResultBufferC8, "%" PRIu32, RVSSConfig);

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_CONFIG, ResultBufferC8, strlen(ResultBufferC8) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetRVSSConfigU32 Reads variable from shared memory
 * \param RVSSConfig Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetRVSSConfigU32(uint32_t * RVSSConfig) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	uint64_t readSetting;

	if (RVSSConfig == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_CONFIG, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtoul(resultBuffer, &endPtr, 10);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for RVSS configuration");
			result = PARAMETER_NOTFOUND;
			*RVSSConfig = DEFAULT_RVSS_CONF;
		}
		else if (readSetting > UINT32_MAX) {
			LogMessage(LOG_LEVEL_WARNING, "Configuration for RVSS configuration outside accepted range");
			result = READ_OK;
			*RVSSConfig = UINT32_MAX;
		}
		else {
			result = READ_OK;
			*RVSSConfig = (uint32_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "RVSS configuration not found!");
		result = PARAMETER_NOTFOUND;
		*RVSSConfig = DEFAULT_RVSS_CONF;
	}

}

/*END of Runtime Variable Subscription Service (RVSS) Configuration**/


/*Runtime Variable Subscription Service (RVSS) Rate*/

/*!
 * \brief DataDictionarySetRVSSRateU8 Parses input variable and sets variable to corresponding value
 * \param RVSSRate
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSRateU8(uint8_t RVSSRate) {
	ReadWriteAccess_t Res;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	memset(ResultBufferC8, 0, DD_CONTROL_BUFFER_SIZE_20);
	sprintf(ResultBufferC8, "%" PRIu8, RVSSRate);

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_RATE, ResultBufferC8, strlen(ResultBufferC8) + 1)) {
		Res = WRITE_OK;
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetRVSSRateU8 Reads variable from shared memory
 * \param RVSSRate Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetRVSSRateU8(uint8_t * RVSSRate) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	uint64_t readSetting;

	if (RVSSRate == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Input pointer error");
		return UNDEFINED;
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_RATE, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtoul(resultBuffer, &endPtr, 10);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for RVSS rate");
			result = PARAMETER_NOTFOUND;
			*RVSSRate = DEFAULT_RVSS_RATE;
		}
		else if (readSetting > UINT8_MAX) {
			LogMessage(LOG_LEVEL_WARNING, "Configuration for RVSS rate outside accepted range");
			result = READ_OK;
			*RVSSRate = UINT8_MAX;
		}
		else {
			result = READ_OK;
			*RVSSRate = (uint8_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "RVSS rate not found!");
		result = PARAMETER_NOTFOUND;
		*RVSSRate = DEFAULT_MAX_PACKETS_LOST;
	}
}

/*END of Runtime Variable Subscription Service (RVSS) Rate**/


/*ASPDebug*/

/*!
 * \brief DataDictionaryInitRVSSAsp inits a data structure for saving RVSSAsp data
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitRVSSAsp() {

	int createdMemory;

	rvssAspDataMemory = createSharedMemory(RVSSASP_DATA_FILENAME, 0, sizeof (ASPType), &createdMemory);
	if (rvssAspDataMemory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to create shared RVSSAsp data memory");
		return UNDEFINED;
	}
	return createdMemory ? WRITE_OK : READ_OK;
}


/*!
 * \brief DataDictionarySetRVSSAsp Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPD
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSAsp(ASPType * ASPD) {


	rvssAspDataMemory = claimSharedMemory(rvssAspDataMemory);
	memcpy(&rvssAspDataMemory, ASPD, sizeof (ASPType));
	rvssAspDataMemory = releaseSharedMemory(rvssAspDataMemory);
	return WRITE_OK;
}

/*!
 * \brief DataDictionaryGetRVSSAsp Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPD Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetRVSSAsp(ASPType * ASPD) {
	

	rvssAspDataMemory = claimSharedMemory(rvssAspDataMemory);
	memcpy(ASPD, &rvssAspDataMemory, sizeof (ASPType));
	rvssAspDataMemory = releaseSharedMemory(rvssAspDataMemory);
	return READ_OK;
}
/*!
 * \brief DataDictionaryFreeRVSSAsp Releases data structure for saving RVSS asp data
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryFreeRVSSAsp() {
	if (rvssAspDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Attempt to free uninitialized ASP memory");
		return UNDEFINED;
	}

	destroySharedMemory(rvssAspDataMemory);

	return WRITE_OK;
}

/*END ASPDebug*/

/*MiscData*/
/**
 * \brief DataDictionarySetMiscData Sets the test misc data.
 * \param miscData The misc data string (ASCII).
 * \return ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetMiscData(
		const char* data,
		const size_t datalen) {
	// TODO implement setting of conf file
	return UNDEFINED;
}

/*!
 * \brief DataDictionaryGetMiscData Reads misc data from shared memory
 * \param MiscData Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetMiscData(char * miscDataBuffer, const size_t buflen) {
	ReadWriteAccess_t result = UNDEFINED;

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_MISC_DATA, miscDataBuffer, buflen)) {
		return READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "MiscData not found!");
		result = PARAMETER_NOTFOUND;
		memset(miscDataBuffer, 0, buflen);
	}
}
/*END of MiscData*/


/*OBCState*/
/*!
 * \brief DataDictionaryInitOBCState Initializes a data structure for saving module state
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitStateData() {
	int createdMemory;
	stateDataMemory = createSharedMemory(STATE_DATA_FILENAME, 0, sizeof (StateDataType), &createdMemory);
	if (stateDataMemory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to create shared state data memory");
		return UNDEFINED;
	}
	return createdMemory ? WRITE_OK : READ_OK;
}

/*!
 * \brief DataDictionarySetOBCStateU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param OBCState
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOBCState(const OBCState_t OBCState) {
	if (stateDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}

	stateDataMemory = claimSharedMemory(stateDataMemory);
	if (stateDataMemory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	stateDataMemory->objectControlState = OBCState;
	stateDataMemory = releaseSharedMemory(stateDataMemory);
	return WRITE_OK;
}

/*!
 * \brief DataDictionaryGetOBCState Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \return Current object control state according to ::OBCState_t
 */
ReadWriteAccess_t DataDictionaryGetOBCState(OBCState_t * OBCState) {
	if (stateDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (OBCState == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory inpput pointer error");
		return UNDEFINED;
	}

	stateDataMemory = claimSharedMemory(stateDataMemory);
	if (stateDataMemory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Shared memory pointer modified unexpectedly");
		return UNDEFINED;
	}

	*OBCState = stateDataMemory->objectControlState;
	stateDataMemory = releaseSharedMemory(stateDataMemory);
	return READ_OK;
}

/*!
 * \brief Releases data structure for saving state data
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryFreeStateData() {
	if (stateDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Attempt to free uninitialized memory");
		return UNDEFINED;
	}

	destroySharedMemory(stateDataMemory);
	return WRITE_OK;
}

/*END OBCState*/

/*MaxPacketLoss*/
ReadWriteAccess_t DataDictionaryInitMaxPacketsLost(void) {
	// TODO implement shmem solution
	return READ_OK;
}

ReadWriteAccess_t DataDictionarySetMaxPacketsLost(uint8_t maxPacketsLostSetting) {
	// TODO implement shmem solution
	return UNDEFINED;
}

ReadWriteAccess_t DataDictionaryGetMaxPacketsLost(uint8_t * maxPacketsLostSetting) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	uint64_t readSetting;

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_MAX_PACKETS_LOST, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtoul(resultBuffer, &endPtr, 10);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for MaxPacketsLost");
			result = PARAMETER_NOTFOUND;
			*maxPacketsLostSetting = DEFAULT_MAX_PACKETS_LOST;
		}
		else if (readSetting > UINT8_MAX) {
			LogMessage(LOG_LEVEL_WARNING, "Configuration for MaxPacketsLost outside accepted range");
			result = READ_OK;
			*maxPacketsLostSetting = UINT8_MAX;
		}
		else {
			result = READ_OK;
			*maxPacketsLostSetting = (uint8_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "MaxPacketsLost not found!");
		result = PARAMETER_NOTFOUND;
		*maxPacketsLostSetting = DEFAULT_MAX_PACKETS_LOST;
	}
}

/*END MaxPacketLoss*/
/*TransmitterID*/
ReadWriteAccess_t DataDictionaryInitTransmitterID(void) {
	// TODO implement shmem solution
	return READ_OK;
}

ReadWriteAccess_t DataDictionaryGetTransmitterID(uint32_t * transmitterID) {
	ReadWriteAccess_t result = UNDEFINED;
	char resultBuffer[DD_CONTROL_BUFFER_SIZE_20];
	char *endPtr;
	uint64_t readSetting;

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_TRANSMITTER_ID, resultBuffer, sizeof (resultBuffer))) {
		readSetting = strtoul(resultBuffer, &endPtr, 10);
		if (endPtr == resultBuffer) {
			LogMessage(LOG_LEVEL_WARNING, "Invalid configuration for TransmitterID");
			result = PARAMETER_NOTFOUND;
			*transmitterID = DEFAULT_TRANSMITTER_ID;
		}
		else if (readSetting > UINT32_MAX) {
			LogMessage(LOG_LEVEL_WARNING, "Configuration for TransmitterID outside accepted range");
			result = READ_OK;
			*transmitterID = UINT32_MAX;
		}
		else {
			result = READ_OK;
			*transmitterID = (uint32_t) readSetting;
		}
		result = READ_OK;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "TransmitterID not found!");
		result = PARAMETER_NOTFOUND;
		*transmitterID = DEFAULT_TRANSMITTER_ID;
	}
	return result;
}

ReadWriteAccess_t DataDictionarySetTransmitterID(const uint32_t transmitterID) {
	// TODO implement shmem solution
	return UNDEFINED;
}

/*END TransmitterID*/

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
											   const struct timeval *receiveTime) {

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
			objectDataMemory[i].lastPositionUpdate = *receiveTime;
			result = WRITE_OK;
		}
	}

	if (result == PARAMETER_NOTFOUND) {
		// Search for unused memory space and place monitor data there
		LogMessage(LOG_LEVEL_INFO, "Received first monitor data from transmitter ID %u", transmitterId);
		for (int i = 0; i < numberOfObjects; ++i) {
			if (objectDataMemory[i].ClientID == 0) {
				objectDataMemory[i].MonrData = *monitorData;
				objectDataMemory[i].lastPositionUpdate = *receiveTime;
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
				objectDataMemory[numberOfObjects - 1].lastPositionUpdate = *receiveTime;
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
														  struct timeval *lastDataUpdate) {
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
			*lastDataUpdate = objectDataMemory[i].lastPositionUpdate;
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
														  const struct timeval *lastDataUpdate) {
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
			objectDataMemory[i].lastPositionUpdate = *lastDataUpdate;
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
		LogMessage(LOG_LEVEL_ERROR, "Unable to list %d transmitter IDs in specified array of size %u", retval,
				   arraySize);
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
		LogMessage(LOG_LEVEL_INFO, "First object information data from ID %u added", objectData->ClientID);
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
				result = WRITE_OK;
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


/*!
 * \brief DataDictionaryModifyTransmitterIDByIP Changes the transmitter ID of the object data identified by a transmitter IP (this is a temporary function, don't use this too much)
 * \param ipKey IP identifying an object
 * \param newTransmitterID Desired new transmitter ID of object data
 * \return Value according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryModifyTransmitterIDByIP(const in_addr_t ipKey,
														const uint32_t newTransmitterID) {
	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (newTransmitterID == 0) {
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
		if (ipKey == objectDataMemory[i].ClientIP) {
			objectDataMemory[i].ClientID = newTransmitterID;
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

ReadWriteAccess_t DataDictionarySetObjectProperties(const uint32_t transmitterID,
													const ObjectPropertiesType * objectProperties) {

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

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			objectDataMemory[i].propertiesReceived = 1;
			objectDataMemory[i].properties = *objectProperties;
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}


ReadWriteAccess_t DataDictionaryGetObjectProperties(const uint32_t transmitterID,
													ObjectPropertiesType * objectProperties) {

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

	memset(objectProperties, 0, sizeof (*objectProperties));

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			if (objectDataMemory[i].propertiesReceived) {
				*objectProperties = objectDataMemory[i].properties;
				result = READ_OK;
			}
			else {
				result = UNINITIALIZED;
			}
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}

ReadWriteAccess_t DataDictionaryClearObjectProperties(const uint32_t transmitterID) {

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


	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			memset(&objectDataMemory[i].properties, 0, sizeof (objectDataMemory[i].properties));
			objectDataMemory[i].propertiesReceived = 0;
			result = WRITE_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}


ReadWriteAccess_t DataDictionarySetRequestedControlAction(const uint32_t transmitterID,
														  const RequestControlActionType * reqCtrlAction) {
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

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {

			objectDataMemory[i].requestedControlAction = *reqCtrlAction;
			result = WRITE_OK;
		}
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;

}

ReadWriteAccess_t DataDictionaryGetRequestedControlAction(const uint32_t transmitterID,
														  RequestControlActionType * reqCtrlAction) {
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

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			*reqCtrlAction = objectDataMemory[i].requestedControlAction;
			result = READ_OK;
		}
	}
	//printf("Grad %f\n",reqCtrlAction->steeringAction.rad);
	//printf("Gtime %d\n",reqCtrlAction->dataTimestamp.tv_sec);
	//printf("Gexc_id %lu\n",reqCtrlAction->executingID);
	//printf("Gspeed %f\n",reqCtrlAction->speedAction.m_s);
	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}

ReadWriteAccess_t DataDictionaryResetRequestedControlAction(const uint32_t transmitterID) {

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

	for (int i = 0; i < numberOfObjects; ++i) {
		if (transmitterID == objectDataMemory[i].ClientID) {
			timerclear(&objectDataMemory[i].requestedControlAction.dataTimestamp);
			result = WRITE_OK;
		}
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}

/**
 * \brief DataDictionarySetOrigin Sets the test origin for one or more objects.
 * \param transmitterID ID of the object to set origin for. If set to null, all objects' origins will be modified.
 * \param origin Geoposition data.
 * \return ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOrigin(const uint32_t * transmitterID, const GeoPosition * origin) {

	ReadWriteAccess_t result;

	if (objectDataMemory == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory not initialized");
		return UNDEFINED;
	}
	if (origin == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (transmitterID != NULL && transmitterID == 0) {
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
		if (transmitterID != NULL) {
			if (objectDataMemory[i].ClientID == *transmitterID) {
				objectDataMemory[i].origin = *origin;
				result = WRITE_OK;
			}
		}
		else {
			objectDataMemory[i].origin = *origin;
			result = WRITE_OK;
		}
	}
	objectDataMemory = releaseSharedMemory(objectDataMemory);

	return result;
}

/**
 * \brief DataDictionaryGetOrigin Read origin setting for specified object. Shared memory must have been
 *			initialized prior to this function call.
 * \param transmitterID Transmitter ID of object for which origin is requested.
 * \param origin Return variable pointer
 * \return ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOrigin(const uint32_t transmitterID, GeoPosition * origin) {

	ReadWriteAccess_t result = PARAMETER_NOTFOUND;

	if (origin == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Shared memory input pointer error");
		return UNDEFINED;
	}
	if (transmitterID == 0) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Transmitter ID 0 is reserved");
		return UNDEFINED;
	}

	objectDataMemory = claimSharedMemory(objectDataMemory);
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);

	for (int i = 0; i < numberOfObjects; ++i) {
		if (objectDataMemory[i].ClientID == transmitterID) {
			memcpy(origin, &objectDataMemory[i].origin, sizeof (GeoPosition));
			result = READ_OK;
		}
	}

	objectDataMemory = releaseSharedMemory(objectDataMemory);
	return result;
}

/**
 * \brief DataDictionaryInitOrigin Read config file and add the origin in .conf to all the objects that are created
 * \return ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitOrigin() {

	char resultBuffer[100];
	char *endptr = NULL;
	int numberOfObjects = getNumberOfMemoryElements(objectDataMemory);
	ReadWriteAccess_t retval = WRITE_OK;

	// should it be write or read Iam writeing to memory but also reading from config file?
	GeoPosition origin;

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE,
									   resultBuffer, sizeof (resultBuffer)) > 0) {
		origin.Longitude = strtod(resultBuffer, &endptr);
		if (endptr == resultBuffer) {
			LogMessage(LOG_LEVEL_ERROR, "OriginLongitude badly formatted");
			retval = PARAMETER_NOTFOUND;
		}
		memset(resultBuffer, 0, sizeof (resultBuffer));
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginLongitude not found!");
	}

	if (UtilReadConfigurationParameter(CONFIGURATION_PARAMETER_ORIGIN_LATITUDE,
									   resultBuffer, sizeof (resultBuffer)) > 0) {
		origin.Latitude = strtod(resultBuffer, &endptr);
		if (endptr == resultBuffer) {
			LogMessage(LOG_LEVEL_ERROR, "OriginLongitude badly formatted");
			retval = PARAMETER_NOTFOUND;
		}
		memset(resultBuffer, 0, sizeof (resultBuffer));
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginLatitude not found!");
	}

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE, resultBuffer, sizeof (resultBuffer))) {
		origin.Altitude = strtod(resultBuffer, &endptr);
		if (endptr == resultBuffer) {
			LogMessage(LOG_LEVEL_ERROR, "OriginAltitude badly formatted");
			retval = PARAMETER_NOTFOUND;
		}
		memset(resultBuffer, 0, sizeof (resultBuffer));
	}
	else {
		retval = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginAltitude not found!");
	}

	if (retval != PARAMETER_NOTFOUND) {
		for (int i = 0; i < numberOfObjects; ++i) {
			objectDataMemory[i].origin = origin;
		}
	}
	return retval;
}
