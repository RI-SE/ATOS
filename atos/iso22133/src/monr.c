#include "monr.h"
#include <errno.h>
#include <string.h>

#include "timeconversions.h"
#include "defines.h"

/*!
 * \brief encodeMONRMessage Constructs an ISO MONR message based on object dynamics data from trajectory file or data generated in a simulator
 * \param inputHeader data to create header
 * \param objectTime Time of the object
 * \param position Position of the object in relation to test origin (includes heading/yaw)
 * \param speed Speed of the object (longitudinal and lateral)
 * \param acceleration Acceleration of the object (longitudinal and lateral)
 * \param driveDirection Drive direction with respect to the heading (backward or forward)
 * \param objectState Current State of the object (off(0),init(1),armed(2),disarmed(3),running(4),postrun(5),remoteControlled(6),aborting(7))
 * \param readyToArm Ready to arm indicator (notReady(0),readyToARM(1),unavailable(2))
 * \param objectErrorState Error status of the object (bit field encoded)
 * \param monrDataBuffer Buffer to hold the message
 * \param bufferLength Length of the buffer
 * \param debug Flag for enabling of debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t encodeMONRMessage(const MessageHeaderType *inputHeader,
						  const struct timeval *objectTime, const CartesianPosition position,
						  const SpeedType speed, const AccelerationType acceleration,
						  const unsigned char driveDirection, const unsigned char objectState,
						  const unsigned char readyToArm, const unsigned char objectErrorState,
						  const unsigned short objectErrorCode,
						  char *monrDataBuffer, const size_t bufferLength, const char debug) {
	MONRType MONRData;

	memset(monrDataBuffer, 0, bufferLength);

	const uint16_t MONRStructSize = (uint16_t) (sizeof (MONRData) - sizeof (MONRData.header)
												- sizeof (MONRData.footer.Crc) -
												sizeof (MONRData.monrStructValueID)
												- sizeof (MONRData.monrStructContentLength));

	// If buffer too small to hold MONR data, generate an error
	if (bufferLength < sizeof (MONRType)) {
		fprintf(stderr, "Buffer too small to hold necessary MONR data\n");
		return -1;
	}

	// Constuct the header
	MONRData.header = buildISOHeader(MESSAGE_ID_MONR, inputHeader, sizeof (MONRData), debug);

	// Fill contents
	MONRData.monrStructValueID = VALUE_ID_MONR_STRUCT;
	MONRData.monrStructContentLength = MONRStructSize;

	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(objectTime);

	MONRData.gpsQmsOfWeek =
		GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (position.isPositionValid) {
		MONRData.xPosition = position.isXcoordValid ? (int32_t) (position.xCoord_m * POSITION_ONE_METER_VALUE) : POSITION_UNAVAILABLE_VALUE;
		MONRData.yPosition = position.isYcoordValid ?  (int32_t) (position.yCoord_m * POSITION_ONE_METER_VALUE) : POSITION_UNAVAILABLE_VALUE;
		MONRData.zPosition = position.isZcoordValid ? (int32_t) (position.zCoord_m * POSITION_ONE_METER_VALUE) : POSITION_UNAVAILABLE_VALUE;
	}
	else {
		errno = EINVAL;
		fprintf(stderr, "Position is a required field in MONR messages\n");
		return -1;
	}

	if (position.isHeadingValid) {
		MONRData.yaw = (uint16_t) (position.heading_rad * 180.0 / M_PI * YAW_ONE_DEGREE_VALUE);
	}
	else {
		MONRData.yaw = YAW_UNAVAILABLE_VALUE;
	}
	// TODO: Add support for pitch and roll
	MONRData.pitch = 0;
	MONRData.roll = 0;

	if (speed.isLongitudinalValid) {
		MONRData.longitudinalSpeed = (int16_t) (speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE);
	}
	else {
		errno = EINVAL;
		fprintf(stderr, "Longitudinal speed is a required field in MONR messages\n");
		return -1;
	}
	MONRData.lateralSpeed =
		speed.isLateralValid ? (int16_t) (speed.lateral_m_s *
										  SPEED_ONE_METER_PER_SECOND_VALUE) : SPEED_UNAVAILABLE_VALUE;
	MONRData.longitudinalAcc = acceleration.isLongitudinalValid ?
		(int16_t) (acceleration.longitudinal_m_s2 *
				   ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) : ACCELERATION_UNAVAILABLE_VALUE;
	MONRData.lateralAcc =
		acceleration.isLateralValid ? (int16_t) (acceleration.lateral_m_s2 *
												 ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) :
		ACCELERATION_UNAVAILABLE_VALUE;

	MONRData.driveDirection = driveDirection;
	MONRData.state = objectState;
	MONRData.readyToArm = readyToArm;
	MONRData.errorStatus = objectErrorState;
	MONRData.errorCode = objectErrorCode;

	if (debug) {
		printf("MONR message:\n\tMONR struct value ID: 0x%x\n\t"
			   "MONR struct content length: %u\n\t"
			   "GPS second of week: %u [¼ ms]\n\t"
			   "X-position: %d [mm]\n\t"
			   "Y-position: %d [mm]\n\t"
			   "Z-position: %d [mm]\n\t"
			   "Yaw: %u [0,01 deg]\n\t"
			   "Longitudinal Speed: %d [0,01 m/s]\n\t"
			   "Lateral Speed: %d [0,01 m/s]\n\t"
			   "Longitudinal Acceleration: %d [0,001 m/s²]\n\t"
			   "Lateral Acceleration: %d [0,001 m/s²]\n\t"
			   "Driving Direction: 0x%x\n\t"
			   "Object State: 0x%x\n\t"
			   "Ready To Arm: 0x%x\n\t"
			   "Object Error State: 0x%x\n",
			   MONRData.monrStructValueID,
			   MONRData.monrStructContentLength,
			   MONRData.gpsQmsOfWeek,
			   MONRData.xPosition,
			   MONRData.yPosition,
			   MONRData.zPosition,
			   MONRData.yaw,
			   MONRData.longitudinalSpeed,
			   MONRData.lateralSpeed,
			   MONRData.longitudinalAcc,
			   MONRData.lateralAcc,
			   MONRData.driveDirection, MONRData.state, MONRData.readyToArm, MONRData.errorStatus);
	}

	// Convert from host endianness to little endian
	MONRData.monrStructValueID = htole16(MONRData.monrStructValueID);
	MONRData.monrStructContentLength = htole16(MONRData.monrStructContentLength);
	MONRData.gpsQmsOfWeek = htole32(MONRData.gpsQmsOfWeek);
	MONRData.xPosition = (int32_t) htole32(MONRData.xPosition);
	MONRData.yPosition = (int32_t) htole32(MONRData.yPosition);
	MONRData.zPosition = (int32_t) htole32(MONRData.zPosition);
	MONRData.yaw = htole16(MONRData.yaw);
	MONRData.pitch = (int16_t) htole16(MONRData.pitch);
	MONRData.roll = (int16_t) htole16(MONRData.roll);
	MONRData.longitudinalSpeed = (int16_t) htole16(MONRData.longitudinalSpeed);
	MONRData.lateralSpeed = (int16_t) htole16(MONRData.lateralSpeed);
	MONRData.longitudinalAcc = (int16_t) htole16(MONRData.longitudinalAcc);
	MONRData.lateralAcc = (int16_t) htole16(MONRData.lateralAcc);
	MONRData.errorCode = htole16(MONRData.errorCode);



	// Construct footer
	MONRData.footer = buildISOFooter(&MONRData, sizeof (MONRData), debug);

	// Copy struct onto the databuffer
	memcpy(monrDataBuffer, &MONRData, sizeof (MONRData));

	if (debug) {
		printf("Byte data[%lu]: ", sizeof (MONRData));
		unsigned int i;

		for (i = 0; i < sizeof (MONRData); i++) {
			if (i > 0)
				printf(":");
			printf("%02X", (unsigned char)monrDataBuffer[i]);
		}
		printf("\n");
	}

	return sizeof (MONRType);
}



/*!
 * \brief decodeMONRMessage Fills a monitor data struct from a buffer of raw data
 * \param monrDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of MONR message
 * \param monitorData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return Number of bytes decoded, or negative value according to ::ISOMessageReturnValue
 */
ssize_t decodeMONRMessage(
	const char *monrDataBuffer,
	const size_t bufferLength,
	const struct timeval currentTime,
	ObjectMonitorType * monitorData,
	const char debug) {

	MONRType MONRData;
	const char *p = monrDataBuffer;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (MONRData) - sizeof (MONRData.header)
														- sizeof (MONRData.footer.Crc) -
														sizeof (MONRData.monrStructValueID)
														- sizeof (MONRData.monrStructContentLength));
	ssize_t retval = MESSAGE_OK;

	if (monitorData == NULL || monrDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to MONR parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(monitorData, 0, sizeof (*monitorData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &MONRData.header, debug)) != MESSAGE_OK) {
		memset(monitorData, 0, sizeof (*monitorData));
		return retval;
	}
	p += sizeof (MONRData.header);


	// If message is not a MONR message, generate an error
	if (MONRData.header.messageID != MESSAGE_ID_MONR) {
		fprintf(stderr, "Attempted to pass non-MONR message into MONR parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	// Decode content header
	memcpy(&MONRData.monrStructValueID, p, sizeof (MONRData.monrStructValueID));
	p += sizeof (MONRData.monrStructValueID);
	MONRData.monrStructValueID = le16toh(MONRData.monrStructValueID);

	// If content is not a MONR struct or an unexpected size, generate an error
	if (MONRData.monrStructValueID != VALUE_ID_MONR_STRUCT) {
		fprintf(stderr, "Attempted to pass non-MONR struct into MONR parsing function\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&MONRData.monrStructContentLength, p, sizeof (MONRData.monrStructContentLength));
	p += sizeof (MONRData.monrStructContentLength);
	MONRData.monrStructContentLength = le16toh(MONRData.monrStructContentLength);

	if (MONRData.monrStructContentLength != ExpectedMONRStructSize) {
		fprintf(stderr, "MONR content length %u differs from the expected length %u\n",
				MONRData.monrStructContentLength, ExpectedMONRStructSize);
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode content
	memcpy(&MONRData.gpsQmsOfWeek, p, sizeof (MONRData.gpsQmsOfWeek));
	p += sizeof (MONRData.gpsQmsOfWeek);
	MONRData.gpsQmsOfWeek = le32toh(MONRData.gpsQmsOfWeek);

	memcpy(&MONRData.xPosition, p, sizeof (MONRData.xPosition));
	p += sizeof (MONRData.xPosition);
	MONRData.xPosition = (int32_t) le32toh(MONRData.xPosition);

	memcpy(&MONRData.yPosition, p, sizeof (MONRData.yPosition));
	p += sizeof (MONRData.yPosition);
	MONRData.yPosition = (int32_t) le32toh(MONRData.yPosition);

	memcpy(&MONRData.zPosition, p, sizeof (MONRData.zPosition));
	p += sizeof (MONRData.zPosition);
	MONRData.zPosition = (int32_t) le32toh(MONRData.zPosition);

	memcpy(&MONRData.yaw, p, sizeof (MONRData.yaw));
	p += sizeof (MONRData.yaw);
	MONRData.yaw = le16toh(MONRData.yaw);

	memcpy(&MONRData.pitch, p, sizeof (MONRData.pitch));
	p += sizeof (MONRData.pitch);
	MONRData.pitch = (int16_t) le16toh(MONRData.pitch);

	memcpy(&MONRData.roll, p, sizeof (MONRData.roll));
	p += sizeof (MONRData.roll);
	MONRData.roll = (int16_t) le16toh(MONRData.roll);

	memcpy(&MONRData.longitudinalSpeed, p, sizeof (MONRData.longitudinalSpeed));
	p += sizeof (MONRData.longitudinalSpeed);
	MONRData.longitudinalSpeed = (int16_t) le16toh(MONRData.longitudinalSpeed);

	memcpy(&MONRData.lateralSpeed, p, sizeof (MONRData.lateralSpeed));
	p += sizeof (MONRData.lateralSpeed);
	MONRData.lateralSpeed = (int16_t) le16toh(MONRData.lateralSpeed);

	memcpy(&MONRData.longitudinalAcc, p, sizeof (MONRData.longitudinalAcc));
	p += sizeof (MONRData.longitudinalAcc);
	MONRData.longitudinalAcc = (int16_t) le16toh(MONRData.longitudinalAcc);

	memcpy(&MONRData.lateralAcc, p, sizeof (MONRData.lateralAcc));
	p += sizeof (MONRData.lateralAcc);
	MONRData.lateralAcc = (int16_t) le16toh(MONRData.lateralAcc);

	memcpy(&MONRData.driveDirection, p, sizeof (MONRData.driveDirection));
	p += sizeof (MONRData.driveDirection);

	memcpy(&MONRData.state, p, sizeof (MONRData.state));
	p += sizeof (MONRData.state);

	memcpy(&MONRData.readyToArm, p, sizeof (MONRData.readyToArm));
	p += sizeof (MONRData.readyToArm);

	memcpy(&MONRData.errorStatus, p, sizeof (MONRData.errorStatus));
	p += sizeof (MONRData.errorStatus);

	memcpy(&MONRData.errorCode, p, sizeof(MONRData.errorCode));
	p += sizeof (MONRData.errorCode);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - monrDataBuffer), &MONRData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding MONR footer\n");
		return retval;
	}
	p += sizeof (MONRData.footer);

	if ((retval = verifyChecksum(monrDataBuffer, p - monrDataBuffer - sizeof (MONRData.footer),
								 MONRData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "MONR checksum error\n");
		return retval;
	}

	if (debug) {
		printf("MONR:\n");
		printf("SyncWord = %x\n", MONRData.header.syncWord);
		printf("TransmitterId = %d\n", MONRData.header.transmitterID);
		printf("PackageCounter = %d\n", MONRData.header.messageCounter);
		printf("AckReq = %d\n", MONRData.header.ackReqProtVer);
		printf("MessageId = %d\n", MONRData.header.messageID);
		printf("MessageLength = %d\n", MONRData.header.messageLength);
		printf("ValueId = %d\n", MONRData.monrStructValueID);
		printf("ContentLength = %d\n", MONRData.monrStructContentLength);
		printf("GPSSOW = %d\n", MONRData.gpsQmsOfWeek);
		printf("XPosition = %d\n", MONRData.xPosition);
		printf("YPosition = %d\n", MONRData.yPosition);
		printf("ZPosition = %d\n", MONRData.zPosition);
		printf("Yaw = %d\n", MONRData.yaw);
		printf("LongitudinalSpeed = %d\n", MONRData.longitudinalSpeed);
		printf("LateralSpeed = %d\n", MONRData.lateralSpeed);
		printf("LongitudinalAcc = %d\n", MONRData.longitudinalAcc);
		printf("LateralAcc = %d\n", MONRData.lateralAcc);
		printf("DriveDirection = %d\n", MONRData.driveDirection);
		printf("State = %d\n", MONRData.state);
		printf("ReadyToArm = %d\n", MONRData.readyToArm);
		printf("ErrorStatus = %d\n", MONRData.errorStatus);
		printf("ErrorCode = %d\n", MONRData.errorCode);
	}

	// Fill output struct with parsed data
	convertMONRToHostRepresentation(&MONRData, &currentTime, monitorData);

	return retval < 0 ? retval : p - monrDataBuffer;
}


/*!
 * \brief convertMONRToHostRepresentation Converts a MONR message to the internal representation for
 * object monitoring data
 * \param MONRData MONR message to be converted
 * \param currentTime Current system time, used to guess GPS week of MONR message
 * \param monitorData Monitor data in which result is to be placed
 */
void convertMONRToHostRepresentation(const MONRType * MONRData,
									 const struct timeval *currentTime, ObjectMonitorType * monitorData) {

	// Timestamp
	monitorData->isTimestampValid = MONRData->gpsQmsOfWeek != GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (monitorData->isTimestampValid) {
		int32_t GPSWeek = getAsGPSWeek(currentTime);

		if (GPSWeek < 0) {
			monitorData->isTimestampValid = false;
		}
		else {
			monitorData->isTimestampValid = setToGPStime(&monitorData->timestamp,
														 (uint16_t) GPSWeek, MONRData->gpsQmsOfWeek) >= 0;
		}
	}

	// Position / heading
	monitorData->position.xCoord_m = (double)(MONRData->xPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.yCoord_m = (double)(MONRData->yPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.zCoord_m = (double)(MONRData->zPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.isXcoordValid = MONRData->xPosition != POSITION_UNAVAILABLE_VALUE;
	monitorData->position.isYcoordValid = MONRData->yPosition != POSITION_UNAVAILABLE_VALUE;
	monitorData->position.isZcoordValid = MONRData->zPosition != POSITION_UNAVAILABLE_VALUE;
	monitorData->position.isPositionValid = monitorData->position.isXcoordValid && monitorData->position.isYcoordValid;
	monitorData->position.isHeadingValid = MONRData->yaw != YAW_UNAVAILABLE_VALUE;
	if (monitorData->position.isHeadingValid) {
		monitorData->position.heading_rad = MONRData->yaw / YAW_ONE_DEGREE_VALUE * M_PI / 180.0;
	}

	// Velocity
	monitorData->speed.isLongitudinalValid = MONRData->longitudinalSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.longitudinal_m_s = monitorData->speed.isLongitudinalValid ?
		(double)(MONRData->longitudinalSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;
	monitorData->speed.isLateralValid = MONRData->lateralSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.lateral_m_s = monitorData->speed.isLateralValid ?
		(double)(MONRData->lateralSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;

	// Acceleration
	monitorData->acceleration.isLongitudinalValid =
		MONRData->longitudinalAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.longitudinal_m_s2 =
		monitorData->acceleration.isLongitudinalValid ? (double)(MONRData->longitudinalAcc) /
		ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;
	monitorData->acceleration.isLateralValid = MONRData->lateralAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.lateral_m_s2 = monitorData->acceleration.isLateralValid ?
		(double)(MONRData->lateralAcc) / ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;

	// Drive direction
	switch (MONRData->driveDirection) {
	case ISO_DRIVE_DIRECTION_FORWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_FORWARD;
		break;
	case ISO_DRIVE_DIRECTION_BACKWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_BACKWARD;
		break;
	case ISO_DRIVE_DIRECTION_UNAVAILABLE:
	default:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
	}

	// State
	switch (MONRData->state) {
	case ISO_OBJECT_STATE_INIT:
		monitorData->state = OBJECT_STATE_INIT;
		break;
	case ISO_OBJECT_STATE_DISARMED:
		monitorData->state = OBJECT_STATE_DISARMED;
		break;
	case ISO_OBJECT_STATE_ARMED:
		monitorData->state = OBJECT_STATE_ARMED;
		break;
	case ISO_OBJECT_STATE_RUNNING:
		monitorData->state = OBJECT_STATE_RUNNING;
		break;
	case ISO_OBJECT_STATE_POSTRUN:
		monitorData->state = OBJECT_STATE_POSTRUN;
		break;
	case ISO_OBJECT_STATE_ABORTING:
		monitorData->state = OBJECT_STATE_ABORTING;
		break;
	case ISO_OBJECT_STATE_REMOTE_CONTROLLED:
		monitorData->state = OBJECT_STATE_REMOTE_CONTROL;
		break;
	case ISO_OBJECT_STATE_PRE_ARMING:
		monitorData->state = OBJECT_STATE_PRE_ARMING;
		break;
	case ISO_OBJECT_STATE_PRE_RUNNING:
		monitorData->state = OBJECT_STATE_PRE_RUNNING;
		break;
	case ISO_OBJECT_STATE_OFF:
	default:
		monitorData->state = OBJECT_STATE_UNKNOWN;
		break;
	}

	// Ready to arm
	switch (MONRData->readyToArm) {
	case ISO_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_READY_TO_ARM;
		break;
	case ISO_NOT_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_NOT_READY_TO_ARM;
		break;
	case ISO_READY_TO_ARM_UNAVAILABLE:
	default:
		monitorData->armReadiness = OBJECT_READY_TO_ARM_UNAVAILABLE;
	}

	// Error status
	monitorData->error.engineFault = MONRData->errorStatus & BITMASK_ERROR_ENGINE_FAULT;
	monitorData->error.abortRequest = MONRData->errorStatus & BITMASK_ERROR_ABORT_REQUEST;
	monitorData->error.batteryFault = MONRData->errorStatus & BITMASK_ERROR_BATTERY_FAULT;
	monitorData->error.unknownError = MONRData->errorStatus & BITMASK_ERROR_OTHER
		|| MONRData->errorStatus & BITMASK_ERROR_VENDOR_SPECIFIC;
	monitorData->error.syncPointEnded = MONRData->errorStatus & BITMASK_ERROR_SYNC_POINT_ENDED;
	monitorData->error.outsideGeofence = MONRData->errorStatus & BITMASK_ERROR_OUTSIDE_GEOFENCE;
	monitorData->error.badPositioningAccuracy =
		MONRData->errorStatus & BITMASK_ERROR_BAD_POSITIONING_ACCURACY;

	return;
}
