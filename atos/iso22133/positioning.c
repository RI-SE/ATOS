#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "positioning.h"

static const char unknownStateString[] = "UNKNOWN";
static const char initStateString[] = "INIT";
static const char disarmedStateString[] = "DISARMED";
static const char armedStateString[] = "ARMED";
static const char runningStateString[] = "RUNNING";
static const char postrunStateString[] = "POSTRUN";
static const char abortingStateString[] = "ABORTING";
static const char remoteControlStateString[] = "REMOTECTRL";

static const char noErrorString[] = "OK";
static const char engineFaultString[] = "ENGINE_FAULT";
static const char batteryFaultString[] = "BATTERY_FAULT";
static const char abortRequestString[] = "ABORT_REQUEST";
static const char otherErrorString[] = "OTHER_ERROR";
static const char syncPointEndedString[] = "SYNC_PT_ENDED";
static const char outsideGeofenceString[] = "OUTSIDE_GEOFENCE";
static const char badPositioningAccuracyString[] = "BAD_POS_ACC";

static const char driveDirectionForwardString[] = "FORWARD";
static const char driveDirectionReverseString[] = "REVERSE";

static const char readyToArmString[] = "READY";
static const char notReadyToArmString[] = "NOT_READY";

/*!
 * \brief objectStateToASCII Converts an enumerated object state into a printable string
 * \param state Variable to be converted
 * \return Pointer to a string of the state
 */
const char *objectStateToASCII(const ObjectStateType state) {
	switch (state) {
	case OBJECT_STATE_DISARMED:
		return disarmedStateString;
	case OBJECT_STATE_ARMED:
		return armedStateString;
	case OBJECT_STATE_RUNNING:
		return runningStateString;
	case OBJECT_STATE_POSTRUN:
		return postrunStateString;
	case OBJECT_STATE_ABORTING:
		return abortingStateString;
	case OBJECT_STATE_REMOTE_CONTROL:
		return remoteControlStateString;
	case OBJECT_STATE_INIT:
		return initStateString;
	case OBJECT_STATE_UNKNOWN:
	default:
		return unknownStateString;
	}
}

/*!
 * \brief ASCIIToObjectState Converts a string into an enumerated state
 * \param asciiString String to convert
 * \return Value according to ::ObjectStateType
 */
ObjectStateType ASCIIToObjectState(const char *asciiString) {
	if (strstr(asciiString, disarmedStateString) != NULL)
		return OBJECT_STATE_DISARMED;
	if (strstr(asciiString, armedStateString) != NULL)
		return OBJECT_STATE_ARMED;
	if (strstr(asciiString, runningStateString) != NULL)
		return OBJECT_STATE_RUNNING;
	if (strstr(asciiString, postrunStateString) != NULL)
		return OBJECT_STATE_POSTRUN;
	if (strstr(asciiString, abortingStateString) != NULL)
		return OBJECT_STATE_ABORTING;
	if (strstr(asciiString, remoteControlStateString) != NULL)
		return OBJECT_STATE_REMOTE_CONTROL;
	return OBJECT_STATE_UNKNOWN;
}


/*!
 * \brief hasError Returns whether or not an ObjectErrorType contains an error
 * \param error Struct containing possible errors
 * \return True if any of the errors are set
 */
bool hasError(const ObjectErrorType error) {
	return error.engineFault || error.abortRequest || error.batteryFault || error.unknownError
		|| error.syncPointEnded || error.outsideGeofence || error.badPositioningAccuracy;
}

/*!
 * \brief errorStatusToASCII Converts an object error struct to ASCII text and prints it to a buffer
 * \param error Struct to be converted
 * \param asciiBuffer Buffer to which text is to be written
 * \param bufferLength Size of buffer to which text is to be written
 */
void errorStatusToASCII(const ObjectErrorType error, char *asciiBuffer, const size_t bufferLength) {

	memset(asciiBuffer, 0, bufferLength);

	if (!hasError(error)) {
		snprintf(asciiBuffer, bufferLength, noErrorString);
		return;
	}
	if (error.engineFault)
		snprintf(asciiBuffer, bufferLength, "%s,", engineFaultString);
	if (error.batteryFault)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", batteryFaultString);
	if (error.unknownError)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", otherErrorString);
	if (error.syncPointEnded)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", syncPointEndedString);
	if (error.outsideGeofence)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", outsideGeofenceString);
	if (error.badPositioningAccuracy)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", badPositioningAccuracyString);
	if (error.abortRequest)
		snprintf(asciiBuffer, bufferLength - strlen(asciiBuffer), "%s,", abortRequestString);

	if (asciiBuffer[strlen(asciiBuffer)] == ',')
		asciiBuffer[strlen(asciiBuffer)] = '\0';

	return;
}

/*!
 * \brief ASCIIToErrorStatus Converts a string with ASCII representation of object errors
 * into a corresponding struct
 * \param asciiString String to be parsed
 * \return Value according to ::ObjectErrorType
 */
ObjectErrorType ASCIIToErrorStatus(const char *asciiString) {
	ObjectErrorType error;

	memset(&error, 0, sizeof (error));

	// First check if string starts with the "all ok" string
	if (strstr(asciiString, noErrorString) == asciiString)
		return error;

	// Check against all error string representations
	if (strstr(asciiString, batteryFaultString) != NULL)
		error.batteryFault = true;
	if (strstr(asciiString, engineFaultString) != NULL)
		error.engineFault = true;
	if (strstr(asciiString, abortRequestString) != NULL)
		error.abortRequest = true;
	if (strstr(asciiString, otherErrorString) != NULL)
		error.unknownError = true;
	if (strstr(asciiString, syncPointEndedString) != NULL)
		error.syncPointEnded = true;
	if (strstr(asciiString, outsideGeofenceString) != NULL)
		error.outsideGeofence = true;
	if (strstr(asciiString, badPositioningAccuracyString) != NULL)
		error.badPositioningAccuracy = true;

	// Error unmatched against all strings casted into unknown type
	if (!hasError(error))
		error.unknownError = true;

	return error;
}

/*!
 * \brief objectMonitorDataToASCII Converts a monitor data struct into human readable ASCII text
 * \param MONRData Struct containing monitor data
 * \param asciiBuffer Buffer in which to print ASCII text representation
 * \param bufferLength Length of ASCII buffer
 * \return Number of bytes printed
 */
int objectMonitorDataToASCII(const ObjectMonitorType * monitorData, char *asciiBuffer,
							 const size_t bufferLength) {

	memset(asciiBuffer, 0, bufferLength);

	if (monitorData->isTimestampValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.6f;", (double) monitorData->timestamp.tv_sec + (double) monitorData->timestamp.tv_usec*1000000.0);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->position.isPositionValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.3f;%.3f;%.3f;", monitorData->position.xCoord_m, monitorData->position.yCoord_m,
				 monitorData->position.zCoord_m);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;NaN;NaN;");

	if (monitorData->position.isHeadingValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.2f;", monitorData->position.heading_rad * 180.0 / M_PI);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->speed.isLongitudinalValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.2f;", monitorData->speed.longitudinal_m_s);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->speed.isLateralValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.2f;", monitorData->speed.lateral_m_s);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->acceleration.isLongitudinalValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.3f;", monitorData->acceleration.longitudinal_m_s2);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->acceleration.isLateralValid)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%.3f;", monitorData->acceleration.lateral_m_s2);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "NaN;");

	if (monitorData->drivingDirection != OBJECT_DRIVE_DIRECTION_UNAVAILABLE)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%s;", monitorData->drivingDirection == OBJECT_DRIVE_DIRECTION_FORWARD ?
				 driveDirectionForwardString : driveDirectionReverseString);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "UNKNOWN;");

	snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
			 "%s;", objectStateToASCII(monitorData->state));

	if (monitorData->armReadiness != OBJECT_READY_TO_ARM_UNAVAILABLE)
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer),
				 "%s;", monitorData->armReadiness == OBJECT_READY_TO_ARM ?
				 readyToArmString : notReadyToArmString);
	else
		snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), "UNKNOWN;");

	errorStatusToASCII(monitorData->error, asciiBuffer + strlen(asciiBuffer),
					   bufferLength - strlen(asciiBuffer));

	snprintf(asciiBuffer + strlen(asciiBuffer), bufferLength - strlen(asciiBuffer), ";");

	return (int)strlen(asciiBuffer);
}




/*!
 * \brief ASCIIToObjectMonitorData Converts an ASCII string into a monitor data struct
 * \param asciiBuffer Buffer containing ASCII text representation
 * \param monitorData Struct containing monitor data
 * \return 0 on success, -1 otherwise
 */
int ASCIIToObjectMonitorData(const char *asciiBuffer, ObjectMonitorType * monitorData) {

	const char *token;
	char *endPtr;
	const char delim[] = ";";
	char *copy = strdup(asciiBuffer);
	double timestamp;

	memset(monitorData, 0, sizeof (*monitorData));

	token = strtok(copy, delim);

	// Timestamp
	token = strtok(NULL, delim);
	timestamp = strtod(token, &endPtr);
	if (endPtr == token) {
		monitorData->isTimestampValid = false;
		memset(&monitorData->timestamp, 0, sizeof (monitorData->timestamp));
	}
	else {
		monitorData->isTimestampValid = true;
		monitorData->timestamp.tv_sec = (time_t) timestamp;
		monitorData->timestamp.tv_usec = (time_t) ((timestamp - (double) monitorData->timestamp.tv_sec)
				* 1000000.0);
	}

	// Position
	monitorData->position.isPositionValid = true;

	token = strtok(NULL, delim);
	monitorData->position.xCoord_m = strtod(token, &endPtr);
	if (endPtr == token) {
		monitorData->position.isPositionValid = false;
	}

	token = strtok(NULL, delim);
	monitorData->position.yCoord_m = strtod(token, &endPtr);
	if (endPtr == token) {
		monitorData->position.isPositionValid = false;
	}

	token = strtok(NULL, delim);
	monitorData->position.zCoord_m = strtod(token, &endPtr);
	if (endPtr == token) {
		monitorData->position.isPositionValid = false;
	}

	token = strtok(NULL, delim);
	monitorData->position.heading_rad = strtod(token, &endPtr) * M_PI / 180.0;
	if (endPtr == token) {
		monitorData->position.isHeadingValid = false;
	}

	// Velocity
	token = strtok(NULL, delim);
	monitorData->speed.longitudinal_m_s = strtod(token, &endPtr);
	monitorData->speed.isLongitudinalValid = endPtr != token;

	token = strtok(NULL, delim);
	monitorData->speed.lateral_m_s = strtod(token, &endPtr);
	monitorData->speed.isLateralValid = endPtr != token;

	// Acceleration
	token = strtok(NULL, delim);
	monitorData->acceleration.longitudinal_m_s2 = strtod(token, &endPtr);
	monitorData->acceleration.isLongitudinalValid = endPtr != token;

	token = strtok(NULL, delim);
	monitorData->acceleration.lateral_m_s2 = strtod(token, &endPtr);
	monitorData->acceleration.isLateralValid = endPtr != token;

	// Drive direction
	token = strtok(NULL, delim);
	if (strstr(token, driveDirectionForwardString) != NULL) {
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_FORWARD;
	}
	else if (strstr(token, driveDirectionReverseString) != NULL) {
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_BACKWARD;
	}
	else {
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
	}

	// State
	token = strtok(NULL, delim);
	monitorData->state = ASCIIToObjectState(token);

	// Ready to arm
	token = strtok(NULL, delim);
	if (strstr(token, readyToArmString) != NULL) {
		monitorData->armReadiness = OBJECT_READY_TO_ARM;
	}
	else if (strstr(token, notReadyToArmString) != NULL) {
		monitorData->armReadiness = OBJECT_NOT_READY_TO_ARM;
	}
	else {
		monitorData->armReadiness = OBJECT_READY_TO_ARM_UNAVAILABLE;
	}

	// Error status
	token = strtok(NULL, delim);
	monitorData->error = ASCIIToErrorStatus(token);

	return 0;
}
