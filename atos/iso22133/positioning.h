#ifndef POSITIONING_H
#define POSITIONING_H
#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <time.h>
#include <stdbool.h>
#include <stddef.h>
#if defined __linux || defined __APPLE__
	#include <sys/time.h>
#endif


/*! Struct describing a position and orientation in a local cartesian coordinate system */
typedef struct {
	double xCoord_m;
	double yCoord_m;
	double zCoord_m;
	double heading_rad;
	bool isXcoordValid;
	bool isYcoordValid;
	bool isZcoordValid;
	bool isPositionValid;
	bool isHeadingValid;
} CartesianPosition; // TODO: rename

/*! Struct describing a position in geographic coordinates */
typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    bool isLatitudeValid;
    bool isLongitudeValid;
    bool isAltitudeValid;
} GeographicPositionType;

typedef enum {
	COORDINATE_SYSTEM_ETRS89 = 0,
	COORDINATE_SYSTEM_NAD83 = 1,
	COORDINATE_SYSTEM_ITRF2000 = 2,
	COORDINATE_SYSTEM_WGS84 = 3,
	COORDINATE_SYSTEM_LOCAL = 4,
	COORDINATE_SYSTEM_UNAVAILABLE = 255
} CoordinateSystemType;

/*! Struct describing longitudinal and lateral speed of an object */
typedef struct {
	double longitudinal_m_s;
	double lateral_m_s;
	bool isLongitudinalValid;
	bool isLateralValid;
} SpeedType;

/*! Struct describing longitudinal and lateral acceleration of an object */
typedef struct {
	double longitudinal_m_s2;
	double lateral_m_s2;
	bool isLongitudinalValid;
	bool isLateralValid;
} AccelerationType;

/*! Enumeration of object driving direction descriptions */
typedef enum {
	OBJECT_DRIVE_DIRECTION_FORWARD,
	OBJECT_DRIVE_DIRECTION_BACKWARD,
	OBJECT_DRIVE_DIRECTION_UNAVAILABLE
} DriveDirectionType;

/*! Enumeration of observable object states */
typedef enum {
	OBJECT_STATE_UNKNOWN,
	OBJECT_STATE_INIT,
	OBJECT_STATE_DISARMED,
	OBJECT_STATE_ARMED,
	OBJECT_STATE_RUNNING,
	OBJECT_STATE_POSTRUN,
	OBJECT_STATE_ABORTING,
    OBJECT_STATE_REMOTE_CONTROL,
    OBJECT_STATE_PRE_ARMING,
    OBJECT_STATE_PRE_RUNNING
} ObjectStateType; // TODO give numbers

/*! Enumeration of ready to arm statuses of an object */
typedef enum {
	OBJECT_NOT_READY_TO_ARM,
	OBJECT_READY_TO_ARM,
	OBJECT_READY_TO_ARM_UNAVAILABLE
} ObjectArmReadinessType;

/*! Struct describing a range of possible errors reported by an object */
typedef struct {
	bool abortRequest;
	bool outsideGeofence;
	bool badPositioningAccuracy;
	bool engineFault;
	bool batteryFault;
	bool syncPointEnded;
	bool unknownError;
} ObjectErrorType;

/*! Struct containing measured object data from a single point in time */
typedef struct {
	bool isTimestampValid;
	struct timeval timestamp;
	CartesianPosition position;
	SpeedType speed;
	AccelerationType acceleration;
	DriveDirectionType drivingDirection;
	ObjectStateType state;
	ObjectArmReadinessType armReadiness;
	ObjectErrorType error;
} ObjectMonitorType;


int objectMonitorDataToASCII(const ObjectMonitorType * monitorData, char *asciiBuffer, const size_t bufferLength);
int ASCIIToObjectMonitorData(const char *asciiBuffer, ObjectMonitorType * monitorData);
bool hasError(const ObjectErrorType error);
const char * objectStateToASCII(const ObjectStateType state);
ObjectStateType ASCIIToObjectState(const char * asciiString);
void errorStatusToASCII(const ObjectErrorType error, char * asciiBuffer, const size_t bufferLength);
ObjectErrorType ASCIIToErrorStatus(const char * asciiString);

#ifdef __cplusplus
}
#endif
#endif
