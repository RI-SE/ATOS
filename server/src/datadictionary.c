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

#include "datadictionary.h"
#include "logging.h"

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
static pthread_mutex_t MONRMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t numberOfObjectsMutex = PTHREAD_MUTEX_INITIALIZER;



/*------------------------------------------------------------
  -- Static function definitions
  ------------------------------------------------------------*/
static U64 DataDictionarySearchParameter(C8 * ParameterName, C8 * ResultBuffer);

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

	Res = Res == READ_OK ? DataDictionaryInitOriginLatitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitOriginLongitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitOriginAltitudeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitVisualizationServerU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitForceToLocalhostU8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxTimeDiffDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxTrajDiffDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPStepBackCountU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPFilterLevelDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitASPMaxDeltaTimeDbl(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitTimeServerIPU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitTimeServerPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorIPU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorTCPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorUDPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSimulatorModeU8(GSD) : Res;
    Res = Res == READ_OK ? DataDictionaryInitVOILReceiversC8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitDTMReceiversC8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitExternalSupervisorIPU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitRVSSConfigU32(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitRVSSRateU8(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitSupervisorTCPPortU16(GSD) : Res;
	Res = Res == READ_OK ? DataDictionaryInitMiscDataC8(GSD) : Res;

    DataDictionarySetOBCStateU8(GSD, OBC_STATE_UNDEFINED);

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

	if (DataDictionarySearchParameter("OrigoLatitude=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("OrigoLatitude", Latitude, 0)) {
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

	if (DataDictionarySearchParameter("OrigoLongitude=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("OrigoLongitude", Longitude, 0)) {
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

	if (DataDictionarySearchParameter("OrigoAltitude=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("OrigoAltitude", Altitude, 0)) {
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

	if (DataDictionarySearchParameter("VisualizationServerName=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("VisualizationServerName", IP, 0)) {
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


/*ForceToLocalhost*/
/*!
 * \brief DataDictionaryInitForceToLocalhostU8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitForceToLocalhostU8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("ForceObjectToLocalhost=", ResultBufferC8)) {
		Res = READ_OK;
		pthread_mutex_lock(&ForceObjectToLocalhostMutex);
		GSD->ForceObjectToLocalhostU8 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "ForceObjectToLocalhost not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetForceToLocalhostU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param ForceLocalhost
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetForceToLocalhostU8(GSDType * GSD, C8 * ForceLocalhost) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter("ForceObjectToLocalhost", ForceLocalhost, 0)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&ForceObjectToLocalhostMutex);
		GSD->ForceObjectToLocalhostU8 = atoi(ForceLocalhost);
		pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetForceToLocalhostU8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param ForceLocalhost Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetForceToLocalhostU8(GSDType * GSD, U8 * ForceLocalhost) {
	pthread_mutex_lock(&ForceObjectToLocalhostMutex);
	*ForceLocalhost = GSD->ForceObjectToLocalhostU8;
	pthread_mutex_unlock(&ForceObjectToLocalhostMutex);
	return READ_OK;
}

/*END of ForceToLocalhost*/

/*ASPMaxTimeDiff*/
/*!
 * \brief DataDictionaryInitASPMaxTimeDiffDbl Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitASPMaxTimeDiffDbl(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("ASPMaxTimeDiff=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("ASPMaxTimeDiff", ASPMaxTimeDiff, 0)) {
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

	if (DataDictionarySearchParameter("ASPMaxTrajDiff=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("ASPMaxTrajDiff", ASPMaxTrajDiff, 0)) {
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

	if (DataDictionarySearchParameter("ASPStepBackCount=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("ASPStepBackCount", ASPStepBackCount, 0)) {
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

	if (DataDictionarySearchParameter("ASPFilterLevel=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("ASPFilterLevel", ASPFilterLevel, 0)) {
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

	if (DataDictionarySearchParameter("ASPMaxDeltaTime=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("ASPMaxDeltaTime", ASPMaxDeltaTime, 0)) {
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


/*TimeServerIP*/
/*!
 * \brief DataDictionaryInitTimeServerIPU32 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitTimeServerIPU32(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("TimeServerIP=", ResultBufferC8)) {
		Res = READ_OK;
		pthread_mutex_lock(&TimeServerIPMutex);
		GSD->TimeServerIPU32 = UtilIPStringToInt(ResultBufferC8);
		bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->TimeServerIPC8, ResultBufferC8);
		pthread_mutex_unlock(&TimeServerIPMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "TimeServerIP not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetTimeServerIPU32 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param TimeServerIP
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetTimeServerIPU32(GSDType * GSD, C8 * TimeServerIP) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter("TimeServerIP", TimeServerIP, 0)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&TimeServerIPMutex);
		GSD->TimeServerIPU32 = UtilIPStringToInt(TimeServerIP);
		bzero(GSD->TimeServerIPC8, DD_CONTROL_BUFFER_SIZE_20);
		strcat(GSD->TimeServerIPC8, TimeServerIP);
		pthread_mutex_unlock(&TimeServerIPMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetTimeServerIPU32 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param TimeServerIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerIPU32(GSDType * GSD, U32 * TimeServerIP) {
	pthread_mutex_lock(&TimeServerIPMutex);
	*TimeServerIP = GSD->TimeServerIPU32;
	pthread_mutex_unlock(&TimeServerIPMutex);
	return READ_OK;
}

/*!
 * \brief DataDictionaryGetTimeServerIPC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param TimeServerIP Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerIPC8(GSDType * GSD, C8 * TimeServerIP, U32 BuffLen) {
	pthread_mutex_lock(&TimeServerIPMutex);
	bzero(TimeServerIP, BuffLen);
	strcat(TimeServerIP, GSD->TimeServerIPC8);
	pthread_mutex_unlock(&TimeServerIPMutex);
	return READ_OK;
}

/*END of TimeServerIP*/


/*TimeServerPort*/
/*!
 * \brief DataDictionaryInitTimeServerPortU16 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitTimeServerPortU16(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("TimeServerPort=", ResultBufferC8)) {
		Res = READ_OK;
		pthread_mutex_lock(&TimeServerPortMutex);
		GSD->TimeServerPortU16 = atoi(ResultBufferC8);
		pthread_mutex_unlock(&TimeServerPortMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "TimeServerPort not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetTimeServerPortU16 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param TimeServerPort
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetTimeServerPortU16(GSDType * GSD, C8 * TimeServerPort) {
	ReadWriteAccess_t Res;

	if (UtilWriteConfigurationParameter("TimeServerPort", TimeServerPort, 0)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&TimeServerPortMutex);
		GSD->TimeServerPortU16 = atoi(TimeServerPort);
		pthread_mutex_unlock(&TimeServerPortMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetTimeServerPortU16 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param TimeServerPort Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetTimeServerPortU16(GSDType * GSD, U16 * TimeServerPort) {
	pthread_mutex_lock(&TimeServerPortMutex);
	*TimeServerPort = GSD->TimeServerPortU16;
	pthread_mutex_unlock(&TimeServerPortMutex);
	return READ_OK;
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

	if (DataDictionarySearchParameter("SimulatorIP=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SimulatorIP", SimulatorIP, 0)) {
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

	if (DataDictionarySearchParameter("SimulatorTCPPort=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SimulatorTCPPort", SimulatorTCPPort, 0)) {
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

	if (DataDictionarySearchParameter("SimulatorUDPPort=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SimulatorUDPPort", SimulatorUDPPort, 0)) {
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

	if (DataDictionarySearchParameter("SimulatorMode=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SimulatorMode", SimulatorMode, 0)) {
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

	if (DataDictionarySearchParameter("VOILReceivers=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("VOILReceivers", VOILReceivers, 0)) {
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

	if (DataDictionarySearchParameter("DTMReceivers=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("DTMReceivers", DTMReceivers, 0)) {
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

	if (DataDictionarySearchParameter("SupervisorIP=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SupervisorIP", IP, 0)) {
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

	if (DataDictionarySearchParameter("SupervisorTCPPort=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("SupervisorTCPPort", SupervisorTCPPort, 0)) {
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

	if (DataDictionarySearchParameter("RVSSConfig=", ResultBufferC8)) {
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

	if (UtilWriteConfigurationParameter("RVSSConfig", ResultBufferC8, 0)) {
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
 * \brief DataDictionaryInitRVSSRateU8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitRVSSRateU8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("RVSSRate=", ResultBufferC8)) {
		Res = READ_OK;
		pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
		GSD->DataDictionaryRVSSRateU8 = (U8) atoi(ResultBufferC8);
		pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
		//LogMessage(LOG_LEVEL_ERROR,"RVSSRate: %s", ResultBufferC8);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "RVSSRate not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetRVSSRateU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param RVSSRate
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetRVSSRateU8(GSDType * GSD, U8 RVSSRate) {
	ReadWriteAccess_t Res;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	bzero(ResultBufferC8, DD_CONTROL_BUFFER_SIZE_20);
	sprintf(ResultBufferC8, "%" PRIu8, RVSSRate);

	if (UtilWriteConfigurationParameter("RVSSRate", ResultBufferC8, 0)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
		GSD->DataDictionaryRVSSRateU8 = RVSSRate;
		pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
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
ReadWriteAccess_t DataDictionaryGetRVSSRateU8(GSDType * GSD, U8 * RVSSRate) {
	pthread_mutex_lock(&DataDictionaryRVSSRateMutex);
	*RVSSRate = GSD->DataDictionaryRVSSRateU8;
	pthread_mutex_unlock(&DataDictionaryRVSSRateMutex);
	return READ_OK;
}

/*END of Runtime Variable Subscription Service (RVSS) Rate**/


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

/*MiscData*/
/*!
 * \brief DataDictionaryInitMiscDataC8 Initializes variable according to the configuration file
 * \param GSD Pointer to shared allocated memory
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitMiscDataC8(GSDType * GSD) {
	ReadWriteAccess_t Res = UNDEFINED;
	C8 ResultBufferC8[DD_CONTROL_BUFFER_SIZE_20];

	if (DataDictionarySearchParameter("MiscData=", ResultBufferC8)) {
		Res = READ_OK;
		pthread_mutex_lock(&MiscDataMutex);
		strcpy(GSD->MiscDataC8, ResultBufferC8);
		pthread_mutex_unlock(&MiscDataMutex);
	}
	else {
		Res = PARAMETER_NOTFOUND;
		LogMessage(LOG_LEVEL_ERROR, "MiscData not found!");
	}

	return Res;
}

/*!
 * \brief DataDictionarySetMiscDataC8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param MiscData
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetMiscDataC8(GSDType * GSD, C8 * MiscData) {
	ReadWriteAccess_t Res;
	if (UtilWriteConfigurationParameter("MiscData", MiscData, 0)) {
		Res = WRITE_OK;
		pthread_mutex_lock(&MiscDataMutex);
		bzero(GSD->MiscDataC8, DD_CONTROL_BUFFER_SIZE_1024);
		strcpy(GSD->MiscDataC8, MiscData);
		pthread_mutex_unlock(&MiscDataMutex);
	}
	else
		Res = PARAMETER_NOTFOUND;
	return Res;
}

/*!
 * \brief DataDictionaryGetMiscDataC8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param MiscData Return variable pointer
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetMiscDataC8(GSDType * GSD, U8 * MiscData, U32 BuffLen) {
	pthread_mutex_lock(&MiscDataMutex);
	bzero(MiscData, BuffLen);
	strcpy(MiscData, GSD->MiscDataC8);
	pthread_mutex_unlock(&MiscDataMutex);
	return READ_OK;
}

/*END of MiscData*/


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
 * \brief DataDictionaryInitMONR inits a data structure for saving object monr
 * \param GSD Pointer to shared allocated memory
 * \param objects number of objects that will transmitt monr
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryInitMONR(GSDType * GSD, U8 objectCount){
    ReadWriteAccess_t Res;
    Res = WRITE_OK;
    pthread_mutex_lock(&MONRMutex);
    GSD->MonrMessages = malloc(sizeof(MONRType) * objectCount);
    //GSD->MonrMessages = ptr;
    pthread_mutex_unlock(&MONRMutex);
    return Res;
}

/*!
 * \brief DataDictionaryInitMONR inits a data structure for saving object monr
 * \param GSD Pointer to shared allocated memory
 * \param objects number of objects that will transmitt monr
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryFreeMONR(GSDType * GSD){
    ReadWriteAccess_t Res;
    Res = WRITE_OK;
    pthread_mutex_lock(&MONRMutex);
    free(GSD->MonrMessages);
    pthread_mutex_unlock(&MONRMutex);
    return Res;
}


/*!
 * \brief DataDictionarySetMONR Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param MONRdata Monitor data
 * \param id object id
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetMONR(GSDType * GSD, MONRType * MONR, U8 id) {
    ReadWriteAccess_t Res;
    Res = WRITE_OK;
    pthread_mutex_lock(&MONRMutex);
    GSD->MonrMessages[id].HeadingU16 = MONR->HeadingU16;
    LogPrint("Setting %d for %d in ptr %p", GSD->MonrMessages[id].HeadingU16, id, GSD->MonrMessages);
    pthread_mutex_unlock(&MONRMutex);
    return Res;
}

/*!
 * \brief DataDictionaryGetMONR Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \param MONRdata Return variable pointer
 * \param id requesed object id
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionaryGetMONR(GSDType * GSD, MONRType *MONR, U8 id) {
    pthread_mutex_lock(&MONRMutex);
    LogPrint("Getting %d for %d in ptr %p", GSD->MonrMessages[id].HeadingU16, id, GSD->MonrMessages);
    pthread_mutex_unlock(&MONRMutex);
    return READ_OK;
}

/*END of MONR*/


/*NbrOfObjects*/
/*!
 * \brief DataDictionarySetOBCStateU8 Parses input variable and sets variable to corresponding value
 * \param GSD Pointer to shared allocated memory
 * \param OBCState
 * \return Result according to ::ReadWriteAccess_t
 */
ReadWriteAccess_t DataDictionarySetNumberOfObjectsU8(GSDType * GSD, U8 *numberOfObjects) {
    ReadWriteAccess_t Res;
    Res = WRITE_OK;
    pthread_mutex_lock(&numberOfObjectsMutex);
    GSD->numberOfObjects = *numberOfObjects;
    pthread_mutex_unlock(&numberOfObjectsMutex);
    return Res;
}

/*!
 * \brief DataDictionaryGetOBCStateU8 Reads variable from shared memory
 * \param GSD Pointer to shared allocated memory
 * \return Current object control state according to ::OBCState_t
 */
ReadWriteAccess_t DataDictionaryGetNumberOfObjectsU8(GSDType * GSD, U8 *numberOfObjects) {
    pthread_mutex_lock(&numberOfObjectsMutex);
    *numberOfObjects = GSD->numberOfObjects;
    pthread_mutex_unlock(&numberOfObjectsMutex);
    return READ_OK;
}

/*END of NbrOfObjects*/


/*!
 * \brief DataDictionarySearchParameter Searches for parameters in the configuration file and returns
the parameter value.
 * \param ParameterName Parameter to search for
 * \param ResultBuffer Buffer where read result should be stored
 * \return Length of read parameter string
 */
U64 DataDictionarySearchParameter(C8 * ParameterName, C8 * ResultBuffer) {
	char confPathDir[MAX_FILE_PATH];

	UtilGetConfDirectoryPath(confPathDir, sizeof (confPathDir));
	strcat(confPathDir, CONF_FILE_NAME);
	bzero(ResultBuffer, DD_CONTROL_BUFFER_SIZE_20);
	UtilSearchTextFile(confPathDir, ParameterName, "", ResultBuffer);
	return strlen(ResultBuffer);
}
