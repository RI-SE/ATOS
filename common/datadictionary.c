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
#include "datadictionary.h"
#include "logging.h"
#include "shmem.h"


// Parameters and variables
static pthread_mutex_t OriginLatitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t OriginLongitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t OriginAltitudeMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t VisualizationServerMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxTimeDiffMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxTrajDiffMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPStepBackCountMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPFilterLevelMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t ASPMaxDeltaTimeMutex = PTHREAD_MUTEX_INITIALIZER;
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
static pthread_mutex_t ObjectStatusMutex = PTHREAD_MUTEX_INITIALIZER;



#define MONR_DATA_FILENAME "MonitorData"
#define STATE_DATA_FILENAME "StateData"
#define MISC_DATA_FILENAME "MiscData"
#define RVSSASP_DATA_FILENAME "RvssAspData"
#define MISC_DATA_MAX_SIZE 1024

typedef struct {
	OBCState_t objectControlState;
} StateDataType;

static volatile ObjectDataType *objectDataMemory = NULL;
static volatile StateDataType *stateDataMemory = NULL;
static volatile char *miscDataMemory = NULL;
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
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryConstructor(GSDType * GSD) {
	ReadWriteAccess_t Res = READ_OK;

	Res = Res == READ_OK ? DataDictionaryInitScenarioName() : Res;
	Res = Res == READ_OK ? DataDictionaryInitOriginLatitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitOriginLongitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitOriginAltitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitVisualizationServerU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxTimeDiffDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxTrajDiffDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPStepBackCountU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPFilterLevelDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxDeltaTimeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorIPU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorTCPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorUDPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorModeU8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitVOILReceiversC8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitDTMReceiversC8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitExternalSupervisorIPU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitRVSSConfigU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSupervisorTCPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitRVSSAsp() : Res;
	Res = Res == READ_OK ? DataDictionaryInitMiscData() : Res;
	Res = Res == READ_OK ? DataDictionaryInitMaxPacketsLost() : Res;
	Res = Res == READ_OK ? DataDictionaryInitTransmitterID() : Res;
	if (Res == READ_OK && DataDictionaryInitObjectData() != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting shared monitor data memory found by constructor");
		Res = UNDEFINED;
	}
	if (Res == READ_OK && DataDictionaryInitStateData() != WRITE_OK) {
		LogMessage(LOG_LEVEL_WARNING, "Preexisting shared state memory found by constructor");
		Res = UNDEFINED;
	}
	if (Res == READ_OK) {
		DataDictionarySetOBCState(OBC_STATE_UNDEFINED);
	}

	return Res;
}


/*!
 * \brief DataDictionaryDestructor Deallocate data held by DataDictionary.
 * \param GSD Pointer to allocated shared memory
 * \return Error code defined by ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryDestructor() {
	ReadWriteAccess_t result = WRITE_OK;

	result = result == WRITE_OK ? DataDictionaryFreeObjectData() : result;
	result = result == WRITE_OK ? DataDictionaryFreeStateData() : result;

	return result;
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
 * \brief DataDictionaryInitOriginLatitudeDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitOriginLatitudeDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LATITUDE, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&OriginLatitudeMutex);
		GSD->OriginLatitudeDbl = atof(ResultBufferC8);
		bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginLatitudeC8, ResultBufferC8);
		pthread_mutex_unlock(&OriginLatitudeMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginLatitude not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetOriginLatitudeDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param Latitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginLatitudeDbl(GSDType * GSD, C8 * Latitude) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LATITUDE, Latitude, strlen(Latitude) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&OriginLatitudeMutex);
		GSD->OriginLatitudeDbl = atof(Latitude);
		bzero(GSD->OriginLatitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginLatitudeC8, Latitude);
		pthread_mutex_unlock(&OriginLatitudeMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetOriginLatitudeDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Latitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLatitudeDbl(GSDType * GSD, dbl * Latitude) {
	pthread_mutex_lock(&OriginLatitudeMutex);
	*Latitude = GSD->OriginLatitudeDbl;
	pthread_mutex_unlock(&OriginLatitudeMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginLatitudeC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Latitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLatitudeC8(GSDType * GSD, C8 * Latitude, U32 BuffLen) {
	pthread_mutex_lock(&OriginLatitudeMutex);
	bzero(Latitude, BuffLen);
	strcat(Latitude, GSD->OriginLatitudeC8);
	pthread_mutex_unlock(&OriginLatitudeMutex);
	return READ_OK;
}

/*END of Origin Latitude*/

/*Origin Longitude*/
/*!
 * \brief DataDictionaryInitOriginLongitudeDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitOriginLongitudeDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&OriginLongitudeMutex);
		GSD->OriginLongitudeDbl = atof(ResultBufferC8);
		bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginLongitudeC8, ResultBufferC8);
		pthread_mutex_unlock(&OriginLongitudeMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginLongitude not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetOriginLongitudeDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param Longitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginLongitudeDbl(GSDType * GSD, C8 * Longitude) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE, Longitude, strlen(Longitude) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&OriginLongitudeMutex);
		GSD->OriginLongitudeDbl = atof(Longitude);
		bzero(GSD->OriginLongitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginLongitudeC8, Longitude);
		pthread_mutex_unlock(&OriginLongitudeMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetOriginLongitudeDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Longitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLongitudeDbl(GSDType * GSD, dbl * Longitude) {
	pthread_mutex_lock(&OriginLongitudeMutex);
	*Longitude = GSD->OriginLongitudeDbl;
	pthread_mutex_unlock(&OriginLongitudeMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginLongitudeC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Longitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginLongitudeC8(GSDType * GSD, C8 * Longitude, U32 BuffLen) {
	pthread_mutex_lock(&OriginLongitudeMutex);
	bzero(Longitude, BuffLen);
	strcat(Longitude, GSD->OriginLongitudeC8);
	pthread_mutex_unlock(&OriginLongitudeMutex);
	return READ_OK;
}

/*END of Origin Longitude*/

/*Origin Altitude*/
/*!
 * \brief DataDictionaryInitOriginAltitudeDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitOriginAltitudeDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&OriginAltitudeMutex);
		GSD->OriginAltitudeDbl = atof(ResultBufferC8);
		bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginAltitudeC8, ResultBufferC8);
		pthread_mutex_unlock(&OriginAltitudeMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "OriginAltitude not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetOriginAltitudeDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param Altitude
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetOriginAltitudeDbl(GSDType * GSD, C8 * Altitude) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE, Altitude, strlen(Altitude) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&OriginAltitudeMutex);
		GSD->OriginAltitudeDbl = atof(Altitude);
		bzero(GSD->OriginAltitudeC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->OriginAltitudeC8, Altitude);
		pthread_mutex_unlock(&OriginAltitudeMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetOriginAltitudeDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Altitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginAltitudeDbl(GSDType * GSD, dbl * Altitude) {
	pthread_mutex_lock(&OriginAltitudeMutex);
	*Altitude = GSD->OriginAltitudeDbl;
	pthread_mutex_unlock(&OriginAltitudeMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetOriginAltitudeC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param Altitude Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetOriginAltitudeC8(GSDType * GSD, C8 * Altitude, U32 BuffLen) {
	pthread_mutex_lock(&OriginAltitudeMutex);
	bzero(Altitude, BuffLen);
	strcat(Altitude, GSD->OriginAltitudeC8);
	pthread_mutex_unlock(&OriginAltitudeMutex);
	return READ_OK;
}

/*END of Origin Altitude*/

/*VisualizationServer*/
/*!
 * \brief DataDictionaryInitVisualizationServerU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitVisualizationServerU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&VisualizationServerMutex);
		GSD->VisualizationServerU32 = UtilIPStringToInt(ResultBufferC8);
		bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->VisualizationServerC8, ResultBufferC8);
		pthread_mutex_unlock(&VisualizationServerMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "VisualizationServerName not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetVisualizationServerU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param IP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetVisualizationServerU32(GSDType * GSD, C8 * IP) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME, IP, strlen(IP) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&VisualizationServerMutex);
		GSD->VisualizationServerU32 = UtilIPStringToInt(IP);
		bzero(GSD->VisualizationServerC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->VisualizationServerC8, IP);
		pthread_mutex_unlock(&VisualizationServerMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetVisualizationServerU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param IP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVisualizationServerU32(GSDType * GSD, U32 * IP) {
	pthread_mutex_lock(&VisualizationServerMutex);
	*IP = GSD->VisualizationServerU32;
	pthread_mutex_unlock(&VisualizationServerMutex);
	return READ_OK;
}


/*!
 * \brief DataDictionaryGetVisualizationServerU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param IP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVisualizationServerC8(GSDType * GSD, C8 * IP, U32 BuffLen) {
	pthread_mutex_lock(&VisualizationServerMutex);
	bzero(IP, BuffLen);
	strcat(IP, GSD->VisualizationServerC8);
	pthread_mutex_unlock(&VisualizationServerMutex);
	return READ_OK;
}

/*END of VisualizationServer*/

/*ASPMaxTimeDiff*/
/*!
 * \brief DataDictionaryInitASPMaxTimeDiffDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPMaxTimeDiffDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ASPMaxTimeDiffMutex);
		GSD->ASPMaxTimeDiffDbl = atof(ResultBufferC8);
		pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ASPMaxTimeDiff not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetASPMaxTimeDiffDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxTimeDiff
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxTimeDiffDbl(GSDType * GSD, C8 * ASPMaxTimeDiff) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF, ASPMaxTimeDiff, strlen(ASPMaxTimeDiff) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ASPMaxTimeDiffMutex);
		GSD->ASPMaxTimeDiffDbl = atof(ASPMaxTimeDiff);
		pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxTimeDiffDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxTimeDiff Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxTimeDiffDbl(GSDType * GSD, dbl * ASPMaxTimeDiff) {
	pthread_mutex_lock(&ASPMaxTimeDiffMutex);
	*ASPMaxTimeDiff = GSD->ASPMaxTimeDiffDbl;
	pthread_mutex_unlock(&ASPMaxTimeDiffMutex);
	return READ_OK;
}

/*END of ASPMaxTimeDiff*/

/*ASPMaxTrajDiff*/
/*!
 * \brief DataDictionaryInitASPMaxTrajDiffDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPMaxTrajDiffDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ASPMaxTrajDiffMutex);
		GSD->ASPMaxTrajDiffDbl = atof(ResultBufferC8);
		pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ASPMaxTrajDiff not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetASPMaxTrajDiffDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxTrajDiff
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxTrajDiffDbl(GSDType * GSD, C8 * ASPMaxTrajDiff) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF, ASPMaxTrajDiff, strlen(ASPMaxTrajDiff) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ASPMaxTrajDiffMutex);
		GSD->ASPMaxTrajDiffDbl = atof(ASPMaxTrajDiff);
		pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxTrajDiffDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxTrajDiff Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxTrajDiffDbl(GSDType * GSD, dbl * ASPMaxTrajDiff) {
	pthread_mutex_lock(&ASPMaxTrajDiffMutex);
	*ASPMaxTrajDiff = GSD->ASPMaxTrajDiffDbl;
	pthread_mutex_unlock(&ASPMaxTrajDiffMutex);
	return READ_OK;
}

/*END of ASPMaxTrajDiff*/


/*ASPStepBackCount*/
/*!
 * \brief DataDictionaryInitASPStepBackCountU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPStepBackCountU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ASPStepBackCountMutex);
		GSD->ASPStepBackCountU32 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&ASPStepBackCountMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ASPStepBackCount not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetASPStepBackCountU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPStepBackCount
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPStepBackCountU32(GSDType * GSD, C8 * ASPStepBackCount) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT, ASPStepBackCount, strlen(ASPStepBackCount) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ASPStepBackCountMutex);
		GSD->ASPStepBackCountU32 = atoi(ASPStepBackCount);
		pthread_mutex_unlock(&ASPStepBackCountMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPStepBackCountU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPStepBackCount Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPStepBackCountU32(GSDType * GSD, U32 * ASPStepBackCount) {
	pthread_mutex_lock(&ASPStepBackCountMutex);
	*ASPStepBackCount = GSD->ASPStepBackCountU32;
	pthread_mutex_unlock(&ASPStepBackCountMutex);
	return READ_OK;
}

/*END of ASPStepBackCount*/

/*ASPFilterLevel*/
/*!
 * \brief DataDictionaryInitASPFilterLevelDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPFilterLevelDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ASPFilterLevelMutex);
		GSD->ASPFilterLevelDbl = atof(ResultBufferC8);
		pthread_mutex_unlock(&ASPFilterLevelMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ASPFilterLevel not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetASPFilterLevelDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPFilterLevel
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPFilterLevelDbl(GSDType * GSD, C8 * ASPFilterLevel) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL, ASPFilterLevel, strlen(ASPFilterLevel) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ASPFilterLevelMutex);
		GSD->ASPFilterLevelDbl = atof(ASPFilterLevel);
		pthread_mutex_unlock(&ASPFilterLevelMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPFilterLevelDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPFilterLevel Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPFilterLevelDbl(GSDType * GSD, dbl * ASPFilterLevel) {
	pthread_mutex_lock(&ASPFilterLevelMutex);
	*ASPFilterLevel = GSD->ASPFilterLevelDbl;
	pthread_mutex_unlock(&ASPFilterLevelMutex);
	return READ_OK;
}

/*END of ASPFilterLevel*/

/*ASPMaxDeltaTime*/
/*!
 * \brief DataDictionaryInitASPMaxDeltaTimeDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPMaxDeltaTimeDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
		GSD->ASPMaxDeltaTimeDbl = atof(ResultBufferC8);
		pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ASPMaxDeltaTime not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetASPMaxDeltaTimeDbl Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxDeltaTime
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetASPMaxDeltaTimeDbl(GSDType * GSD, C8 * ASPMaxDeltaTime) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME, ASPMaxDeltaTime, strlen(ASPMaxDeltaTime) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
		GSD->ASPMaxDeltaTimeDbl = atof(ASPMaxDeltaTime);
		pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetASPMaxDeltaTimeDbl Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ASPMaxDeltaTime Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetASPMaxDeltaTimeDbl(GSDType * GSD, dbl * ASPMaxDeltaTime) {
	pthread_mutex_lock(&ASPMaxDeltaTimeMutex);
	*ASPMaxDeltaTime = GSD->ASPMaxDeltaTimeDbl;
	pthread_mutex_unlock(&ASPMaxDeltaTimeMutex);
	return READ_OK;
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
 * \brief DataDictionaryInitSimulatorIPU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitSimulatorIPU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_IP, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&SimulatorIPMutex);
		GSD->SimulatorIPU32 = UtilIPStringToInt(ResultBufferC8);
		bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->SimulatorIPC8, ResultBufferC8);
		pthread_mutex_unlock(&SimulatorIPMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "SimulatorIP not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetSimulatorIPU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorIP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorIPU32(GSDType * GSD, C8 * SimulatorIP) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_IP, SimulatorIP, strlen(SimulatorIP) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&SimulatorIPMutex);
		GSD->SimulatorIPU32 = UtilIPStringToInt(SimulatorIP);
		bzero(GSD->SimulatorIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->SimulatorIPC8, SimulatorIP);
		pthread_mutex_unlock(&SimulatorIPMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSimulatorIPU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorIPU32(GSDType * GSD, U32 * SimulatorIP) {
	pthread_mutex_lock(&SimulatorIPMutex);
	*SimulatorIP = GSD->SimulatorIPU32;
	pthread_mutex_unlock(&SimulatorIPMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetSimulatorIPC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorIPC8(GSDType * GSD, C8 * SimulatorIP, U32 BuffLen) {
	pthread_mutex_lock(&SimulatorIPMutex);
	bzero(SimulatorIP, BuffLen);
	strcat(SimulatorIP, GSD->SimulatorIPC8);
	pthread_mutex_unlock(&SimulatorIPMutex);
	return READ_OK;
}

/*END of SimulatorIP*/

/*SimulatorTCPPort*/
/*!
 * \brief DataDictionaryInitSimulatorTCPPortU16 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitSimulatorTCPPortU16(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&SimulatorTCPPortMutex);
		GSD->SimulatorTCPPortU16 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&SimulatorTCPPortMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "SimulatorTCPPort not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetSimulatorTCPPortU16 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorTCPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorTCPPortU16(GSDType * GSD, C8 * SimulatorTCPPort) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP, SimulatorTCPPort, strlen(SimulatorTCPPort) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&SimulatorTCPPortMutex);
		GSD->SimulatorTCPPortU16 = atoi(SimulatorTCPPort);
		pthread_mutex_unlock(&SimulatorTCPPortMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSimulatorTCPPortU16 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorTCPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorTCPPortU16(GSDType * GSD, U16 * SimulatorTCPPort) {
	pthread_mutex_lock(&SimulatorTCPPortMutex);
	*SimulatorTCPPort = GSD->SimulatorTCPPortU16;
	pthread_mutex_unlock(&SimulatorTCPPortMutex);
	return READ_OK;
}

/*END of SimulatorTCPPort*/

/*SimulatorUDPPort*/
/*!
 * \brief DataDictionaryInitSimulatorUDPPortU16 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitSimulatorUDPPortU16(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&SimulatorUDPPortMutex);
		GSD->SimulatorUDPPortU16 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&SimulatorUDPPortMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "SimulatorUDPPort not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetSimulatorUDPPortU16 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorUDPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorUDPPortU16(GSDType * GSD, C8 * SimulatorUDPPort) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP, SimulatorUDPPort, strlen(SimulatorUDPPort) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&SimulatorUDPPortMutex);
		GSD->SimulatorUDPPortU16 = atoi(SimulatorUDPPort);
		pthread_mutex_unlock(&SimulatorUDPPortMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSimulatorUDPPortU16 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorUDPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorUDPPortU16(GSDType * GSD, U16 * SimulatorUDPPort) {
	pthread_mutex_lock(&SimulatorUDPPortMutex);
	*SimulatorUDPPort = GSD->SimulatorUDPPortU16;
	pthread_mutex_unlock(&SimulatorUDPPortMutex);
	return READ_OK;
}

/*END of SimulatorUDPPort*/

/*SimulatorMode*/
/*!
 * \brief DataDictionaryInitSimulatorModeU8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitSimulatorModeU8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_MODE, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&SimulatorModeMutex);
		GSD->SimulatorModeU8 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&SimulatorModeMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "SimulatorMode not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetSimulatorModeU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorMode
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSimulatorModeU8(GSDType * GSD, C8 * SimulatorMode) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_SIMULATOR_MODE, SimulatorMode, strlen(SimulatorMode) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&SimulatorModeMutex);
		GSD->SimulatorModeU8 = atoi(SimulatorMode);
		pthread_mutex_unlock(&SimulatorModeMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSimulatorModeU8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SimulatorMode Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSimulatorModeU8(GSDType * GSD, U8 * SimulatorMode) {
	pthread_mutex_lock(&SimulatorModeMutex);
	*SimulatorMode = GSD->SimulatorModeU8;
	pthread_mutex_unlock(&SimulatorModeMutex);
	return READ_OK;
}

/*END of SimulatorMode*/

/*VOILReceivers*/
/*!
 * \brief DataDictionaryInitVOILReceiversC8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitVOILReceiversC8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_1024];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_VOIL_RECEIVERS, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&VOILReceiversMutex);
		strcpy(GSD->VOILReceiversC8, ResultBufferC8);
		pthread_mutex_unlock(&VOILReceiversMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "VOILReceivers not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetVOILReceiversC8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param VOILReceivers
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetVOILReceiversC8(GSDType * GSD, C8 * VOILReceivers) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_VOIL_RECEIVERS, VOILReceivers, strlen(VOILReceivers) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&VOILReceiversMutex);
		strcpy(GSD->VOILReceiversC8, VOILReceivers);
		pthread_mutex_unlock(&VOILReceiversMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetVOILReceiversC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param VOILReceivers Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetVOILReceiversC8(GSDType * GSD, U8 * VOILReceivers, U32 BuffLen) {
	pthread_mutex_lock(&VOILReceiversMutex);
	bzero(VOILReceivers, BuffLen);
	strcpy(VOILReceivers, GSD->VOILReceiversC8);
	pthread_mutex_unlock(&VOILReceiversMutex);
	return READ_OK;
}

/*END of VOILReceivers*/

/*DTMReceivers*/
/*!
 * \brief DataDictionaryInitDTMReceiversC8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitDTMReceiversC8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_1024];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_DTM_RECEIVERS, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&DTMReceiversMutex);
		strcpy(GSD->DTMReceiversC8, ResultBufferC8);
		pthread_mutex_unlock(&DTMReceiversMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "DTMReceivers not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetDTMReceiversC8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param DTMReceivers
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetDTMReceiversC8(GSDType * GSD, C8 * DTMReceivers) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_DTM_RECEIVERS, DTMReceivers, strlen(DTMReceivers) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&DTMReceiversMutex);
		strcpy(GSD->DTMReceiversC8, DTMReceivers);
		pthread_mutex_unlock(&DTMReceiversMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetDTMReceiversC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param DTMReceivers Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetDTMReceiversC8(GSDType * GSD, U8 * DTMReceivers, U32 BuffLen) {
	pthread_mutex_lock(&DTMReceiversMutex);
	bzero(DTMReceivers, BuffLen);
	strcpy(DTMReceivers, GSD->DTMReceiversC8);
	pthread_mutex_unlock(&DTMReceiversMutex);
	return READ_OK;
}

/*END of DTMReceivers*/

/*External Supervisor IP*/
/*!
 * \brief DataDictionaryInitExternalSupervisorIPU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitExternalSupervisorIPU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&ExternalSupervisorIPMutex);
		GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(ResultBufferC8);
		bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->ExternalSupervisorIPC8, ResultBufferC8);
		pthread_mutex_unlock(&ExternalSupervisorIPMutex);
		//LogMessage(LOG_LEVEL_ERROR,"Supervisor IP: %s", ResultBufferC8);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "Supervisor IP not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetExternalSupervisorIPU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param IP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetExternalSupervisorIPU32(GSDType * GSD, C8 * IP) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP, IP, strlen(IP) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ExternalSupervisorIPMutex);
		GSD->ExternalSupervisorIPU32 = UtilIPStringToInt(IP);
		bzero(GSD->ExternalSupervisorIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->ExternalSupervisorIPC8, IP);
		pthread_mutex_unlock(&ExternalSupervisorIPMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetExternalSupervisorIPU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param IP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPU32(GSDType * GSD, U32 * IP) {
	pthread_mutex_lock(&ExternalSupervisorIPMutex);
	*IP = GSD->ExternalSupervisorIPU32;
	pthread_mutex_unlock(&ExternalSupervisorIPMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetExternalSupervisorIPC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param IP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetExternalSupervisorIPC8(GSDType * GSD, C8 * IP, U32 BuffLen) {
	pthread_mutex_lock(&ExternalSupervisorIPMutex);
	bzero(IP, BuffLen);
	strcat(IP, GSD->ExternalSupervisorIPC8);
	pthread_mutex_unlock(&ExternalSupervisorIPMutex);
	return READ_OK;
}

/*END of External Supervisor IP*/

/*External SupervisorTCPPort*/
/*!
 * \brief DataDictionaryInitSupervisorTCPPortU16 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitSupervisorTCPPortU16(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&SupervisorTCPPortMutex);
		GSD->SupervisorTCPPortU16 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&SupervisorTCPPortMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "SupervisorTCPPort not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetSupervisorTCPPortU16 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param SupervisorTCPPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetSupervisorTCPPortU16(GSDType * GSD, C8 * SupervisorTCPPort) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP, SupervisorTCPPort,
		 strlen(SupervisorTCPPort) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&SupervisorTCPPortMutex);
		GSD->SupervisorTCPPortU16 = atoi(SupervisorTCPPort);
		pthread_mutex_unlock(&SupervisorTCPPortMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetSupervisorTCPPortU16 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param SupervisorTCPPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetSupervisorTCPPortU16(GSDType * GSD, U16 * SupervisorTCPPort) {
	pthread_mutex_lock(&SupervisorTCPPortMutex);
	*SupervisorTCPPort = GSD->SupervisorTCPPortU16;
	pthread_mutex_unlock(&SupervisorTCPPortMutex);
	return READ_OK;
}

/*END of External SupervisorTCPPort*/

/*Runtime Variable Subscription Service (RVSS) Configuration*/
/*!
 * \brief DataDictionaryInitRVSSConfigU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitRVSSConfigU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (UtilReadConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_CONFIG, ResultBufferC8, sizeof (ResultBufferC8))) {
		Res = READ_OK;
		pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
		GSD->DataDictionaryRVSSConfigU32 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
		//LogMessage(LOG_LEVEL_ERROR,"RVSSConfig: %s", ResultBufferC8);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "RVSSConfig not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetInitRVSSConfigU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param RVSSConfig
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSConfigU32(GSDType * GSD, U32 RVSSConfig) {
	ReadWriteAccess_t Res;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
	sprintf(ResultBufferC8, "%" PRIu32, RVSSConfig);

	if (UtilWriteConfigurationParameter
		(CONFIGURATION_PARAMETER_RVSS_CONFIG, ResultBufferC8, strlen(ResultBufferC8) + 1)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
		GSD->DataDictionaryRVSSConfigU32 = RVSSConfig;
		pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetRVSSConfigU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param RVSSConfig Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetRVSSConfigU32(GSDType * GSD, U32 * RVSSConfig) {
	pthread_mutex_lock(&DataDictionaryRVSSConfigMutex);
	*RVSSConfig = GSD->DataDictionaryRVSSConfigU32;
	pthread_mutex_unlock(&DataDictionaryRVSSConfigMutex);
	return READ_OK;
}

/*END of Runtime Variable Subscription Service (RVSS) Configuration**/


/*Runtime Variable Subscription Service (RVSS) Rate*/

/*!
 * \brief DataDictionarySetRVSSRateU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
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
 * \param GSD Pointer to shared allocated memory
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

/*END ASPDebug*/

/*MiscData*/
/*!
 * \brief DataDictionaryInitMiscData inits a data structure for saving misc data
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitMiscData(void) {
	return READ_OK;
}

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
