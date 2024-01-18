#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stddef.h>
#include "header.h"
#include "footer.h"
#include "iso22133.h"

#pragma pack(push, 1)
//! MONR message */
typedef struct {
	HeaderType header;
	uint16_t monrStructValueID;
	uint16_t monrStructContentLength;
	uint32_t gpsQmsOfWeek;
	int32_t xPosition;
	int32_t yPosition;
	int32_t zPosition;
	uint16_t yaw;
	int16_t pitch;
	int16_t roll;
	int16_t longitudinalSpeed;
	int16_t lateralSpeed;
	int16_t longitudinalAcc;
	int16_t lateralAcc;
	uint8_t driveDirection;
	uint8_t state;
	uint8_t readyToArm;
	uint8_t errorStatus;
	uint16_t errorCode;
	FooterType footer;
} MONRType;
#pragma pack(pop)

//! MONR value IDs
#define VALUE_ID_MONR_STRUCT 0x80


void convertMONRToHostRepresentation(
		const MONRType * MONRData,
		const struct timeval *currentTime,
		ObjectMonitorType * monitorData);
#ifdef __cplusplus
}
#endif