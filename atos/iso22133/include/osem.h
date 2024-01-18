#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#include "iso22133.h"
#include "header.h"
#include "footer.h"

#pragma pack(push, 1)
/*! OSEM ID struct */
typedef struct {
	uint32_t deviceID;
	uint32_t subDeviceID;
	uint32_t systemControlCentreID;
} OSEMIDType;

/*! OSEM origin struct */
typedef struct {
	int64_t latitude;
	int64_t longitude;
	int32_t altitude;
	uint16_t rotation;
	uint8_t coordinateSystem;
} OSEMOriginType;

/*! OSEM date time struct */
typedef struct {
	uint32_t dateISO8601;
	uint16_t gpsWeek;
	uint32_t gpsQmsOfWeek;
	uint8_t leapSeconds;
} OSEMDateTimeType;

/*! OSEM accuracy requirements struct */
typedef struct {
	uint16_t maxWayDeviation;
	uint16_t maxLateralDeviation;
	uint16_t maxYawDeviation;
	uint16_t maxPositionError;
	uint16_t heabTimeout;
	uint8_t testMode;
	uint8_t monrRate;
	uint8_t monr2Rate;
	uint8_t heabRate;
	uint32_t maxMessageLength;
} OSEMAccuracyRequirementsType;

/*! OSEM time server struct */
typedef struct {
	uint32_t ip;
	uint16_t port;
} OSEMTimeServerType;

/*! OSEM ID association struct */
typedef struct {
	// TODO
} OSEMIDAssociationType;

/*! OSEM message */
typedef struct {
	HeaderType header;
	uint16_t idStructValueID;
	uint16_t idStructContentLength;
	OSEMIDType ids;
	uint16_t originStructValueID;
	uint16_t originStructContentLength;
	OSEMOriginType origin;
	uint16_t dateTimeStructValueID;
	uint16_t dateTimeStructContentLength;
	OSEMDateTimeType timestamp;
	uint16_t accReqStructValueID;
	uint16_t accReqStructContentLength;
	OSEMAccuracyRequirementsType requirements;
	uint16_t timeServerStructValueID;
	uint16_t timeServerStructContentLength;
	OSEMTimeServerType timeserver;
	uint16_t idAssociationStructValueID;
	uint16_t idAssociationStructContentLength;
	OSEMIDAssociationType idAssociation;
	FooterType footer;
} OSEMType;
#pragma pack(pop)


//! OSEM value IDs
#define VALUE_ID_OSEM_ID_STRUCT 0x0020
#define VALUE_ID_OSEM_ORIGIN_STRUCT 0x0021
#define VALUE_ID_OSEM_DATE_TIME_STRUCT 0x0022
#define VALUE_ID_OSEM_ACC_REQ_STRUCT 0x0023
#define VALUE_ID_OSEM_TIME_SERVER_STRUCT 0x0024
#define VALUE_ID_OSEM_PAIRING_COUNT 0x0025
#define VALUE_ID_OSEM_PAIRING_ID 0x0026
#define VALUE_ID_OSEM_PAIRING_NAME 0x0027


void convertOSEMToHostRepresentation(
		const OSEMType * OSEMData,
		ObjectSettingsType * ObjectSettingsData);
#ifdef __cplusplus
}
#endif