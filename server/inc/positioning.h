#ifndef POSITIONING_H
#define POSITIONING_H
#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <sys/time.h>
#include <stdbool.h>

typedef struct {
	double xCoord_m;
	double yCoord_m;
	double zCoord_m;
	double heading_deg;
	bool isPositionValid;
	bool isHeadingValid;
} CartesianPosition; // TODO: rename

typedef struct {
	double longitudinal_m_s;
	double lateral_m_s;
	bool isValid;
} SpeedType;

typedef struct {
	double longitudinal_m_s2;
	double lateral_m_s2;
	bool isValid;
} AccelerationType;

typedef enum {
	OBJECT_DRIVE_DIRECTION_FORWARD,
	OBJECT_DRIVE_DIRECTION_BACKWARD,
	OBJECT_DRIVE_DIRECTION_UNAVAILABLE
} DriveDirectionType;

typedef enum {
	OBJECT_STATE_UNKNOWN,
	OBJECT_STATE_DISARMED,
	OBJECT_STATE_ARMED,
	OBJECT_STATE_ACTIVE,
	OBJECT_STATE_POSTRUN,
	OBJECT_STATE_ABORTING,
	OBJECT_STATE_REMOTE_CONTROL
} ObjectStateType;

typedef enum {
	OBJECT_NOT_READY_TO_ARM,
	OBJECT_READY_TO_ARM,
	OBJECT_READY_TO_ARM_UNAVAILABLE
} ObjectArmReadinessType;

typedef struct {
	bool abortRequest;
	bool outsideGeofence;
	bool badPositioningAccuracy;
	bool engineFault;
	bool batteryFault;
	bool syncPointEnded;
	bool unknownError;
} ObjectErrorType;

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

#ifdef __cplusplus
}
#endif
#endif
