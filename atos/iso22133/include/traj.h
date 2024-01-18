#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "header.h"
#include "footer.h"
#include "defines.h"
#include "iohelpers.h"
#include <stdint.h>

#pragma pack(push, 1)
/*! TRAJ message */
#define TRAJ_NAME_STRING_MAX_LENGTH 64
typedef struct {
	HeaderType header;
	uint16_t trajectoryIDValueID;
	uint16_t trajectoryIDContentLength;
	uint16_t trajectoryID;
	uint16_t trajectoryNameValueID;
	uint16_t trajectoryNameContentLength;
	char trajectoryName[TRAJ_NAME_STRING_MAX_LENGTH];
	uint16_t trajectoryInfoValueID;
	uint16_t trajectoryInfoContentLength;
	uint8_t trajectoryInfo;
} TRAJHeaderType;

typedef struct {
	uint16_t trajectoryPointValueID;
	uint16_t trajectoryPointContentLength;
	uint32_t relativeTime;
	int32_t xPosition;
	int32_t yPosition;
	int32_t zPosition;
	uint16_t yaw;
	int16_t longitudinalSpeed;
	int16_t lateralSpeed;
	int16_t longitudinalAcceleration;
	int16_t lateralAcceleration;
	float_t curvature;
} TRAJPointType;

typedef struct {
	uint16_t lineInfoValueID;
	uint16_t lineInfoContentLength;
	uint8_t lineInfo;
	FooterType footer;
} TRAJFooterType;
#pragma pack(pop)

//! TRAJ value IDs
#define VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER 0x0101
#define VALUE_ID_TRAJ_TRAJECTORY_NAME 0x0102
#define VALUE_ID_TRAJ_TRAJECTORY_INFO 0x0104
#define VALUE_ID_TRAJ_POINT 0x0001
#define VALUE_ID_TRAJ_LINE_INFO 0x0053
#define VALUE_ID_TRAJ_VENDOR_SPECIFIC_RANGE_LOWER 0xA000
#define VALUE_ID_TRAJ_VENDOR_SPECIFIC_RANGE_UPPER 0xAFFF

#define TRAJ_LINE_INFO_END_OF_TRANSMISSION 0x04

static enum ISOMessageReturnValue convertTRAJHeaderToHostRepresentation(TRAJHeaderType* TRAJHeaderData,
				uint32_t trajectoryLength,	TrajectoryHeaderType* trajectoryHeaderData);
static enum ISOMessageReturnValue convertTRAJPointToHostRepresentation(TRAJPointType* TRAJPointData,
														TrajectoryWaypointType* wayPoint);

#ifdef __cplusplus
}
#endif