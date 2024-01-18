#include "iso22133.h"
#include "header.h"
#include "footer.h"
#include "defines.h"
#include "timeconversions.h"
#include "iohelpers.h"

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>




#pragma pack(push,1)			// Ensure sizeof() is useable for (most) network byte lengths

/*! OPRO message */
typedef struct {
	HeaderType header;
	uint16_t objectTypeValueID;
	uint16_t objectTypeContentLength;
	uint8_t objectType;
	uint16_t actorTypeValueID;
	uint16_t actorTypeContentLength;
	uint8_t actorType;
	uint16_t operationModeValueID;
	uint16_t operationModeContentLength;
	uint8_t operationMode;
	uint16_t massValueID;
	uint16_t massContentLength;
	uint32_t mass;
	uint16_t objectLengthXValueID;
	uint16_t objectLengthXContentLength;
	uint32_t objectLengthX;
	uint16_t objectLengthYValueID;
	uint16_t objectLengthYContentLength;
	uint32_t objectLengthY;
	uint16_t objectLengthZValueID;
	uint16_t objectLengthZContentLength;
	uint32_t objectLengthZ;
	uint16_t positionDisplacementXValueID;
	uint16_t positionDisplacementXContentLength;
	int16_t positionDisplacementX;
	uint16_t positionDisplacementYValueID;
	uint16_t positionDisplacementYContentLength;
	int16_t positionDisplacementY;
	uint16_t positionDisplacementZValueID;
	uint16_t positionDisplacementZContentLength;
	int16_t positionDisplacementZ;
	FooterType footer;
} OPROType;

#define VALUE_ID_OPRO_OBJECT_TYPE 0x0100
#define VALUE_ID_OPRO_ACTOR_TYPE 0x0101
#define VALUE_ID_OPRO_OPERATION_MODE 0x0102
#define VALUE_ID_OPRO_MASS 0x0103
#define VALUE_ID_OPRO_OBJECT_LENGTH_X 0x0104
#define VALUE_ID_OPRO_OBJECT_LENGTH_Y 0x0105
#define VALUE_ID_OPRO_OBJECT_LENGTH_Z 0x0106
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_X 0x0107
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y 0x0108
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z 0x0109

//! OPRO field descriptions
static DebugStrings_t OPROObjectTypeDescription =				{"ObjectType",				"",		&printU8};
static DebugStrings_t OPROActorTypeDescription =				{"ActorType",				"",		&printU8};
static DebugStrings_t OPROOperationModeDescription =			{"OperationMode",			"",		&printU8};
static DebugStrings_t OPROMassDescription =						{"Mass",					"[g]",	&printU32};
static DebugStrings_t OPROObjectLengthXDescription =			{"Object length X",			"[mm]", &printU32};
static DebugStrings_t OPROObjectLengthYDescription =			{"Object length Y",			"[mm]", &printU32};
static DebugStrings_t OPROObjectLengthZDescription =			{"Object length Z",			"[mm]", &printU32};
static DebugStrings_t OPROPositionDisplacementXDescription =	{"Position displacement X", "[mm]", &printI16};
static DebugStrings_t OPROPositionDisplacementYDescription =	{"Position displacement Y", "[mm]", &printI16};
static DebugStrings_t OPROPositionDisplacementZDescription =	{"Position displacement Z", "[mm]", &printI16};


/*! FOPR message */
typedef struct {
	HeaderType header;
	uint16_t foreignTransmitterIDValueID;
	uint16_t foreignTransmitterIDContentLength;
	uint32_t foreignTransmitterID;
	uint16_t objectTypeValueID;
	uint16_t objectTypeContentLength;
	uint8_t objectType;
	uint16_t actorTypeValueID;
	uint16_t actorTypeContentLength;
	uint8_t actorType;
	uint16_t operationModeValueID;
	uint16_t operationModeContentLength;
	uint8_t operationMode;
	uint16_t massValueID;
	uint16_t massContentLength;
	uint32_t mass;
	uint16_t objectLengthXValueID;
	uint16_t objectLengthXContentLength;
	uint32_t objectLengthX;
	uint16_t objectLengthYValueID;
	uint16_t objectLengthYContentLength;
	uint32_t objectLengthY;
	uint16_t objectLengthZValueID;
	uint16_t objectLengthZContentLength;
	uint32_t objectLengthZ;
	uint16_t positionDisplacementXValueID;
	uint16_t positionDisplacementXContentLength;
	int16_t positionDisplacementX;
	uint16_t positionDisplacementYValueID;
	uint16_t positionDisplacementYContentLength;
	int16_t positionDisplacementY;
	uint16_t positionDisplacementZValueID;
	uint16_t positionDisplacementZContentLength;
	int16_t positionDisplacementZ;
	FooterType footer;
} FOPRType;

#define VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID	0x00FF
#define VALUE_ID_FOPR_OBJECT_TYPE 0x0100
#define VALUE_ID_FOPR_ACTOR_TYPE 0x0101
#define VALUE_ID_FOPR_OPERATION_MODE 0x0102
#define VALUE_ID_FOPR_MASS 0x0103
#define VALUE_ID_FOPR_OBJECT_LENGTH_X 0x0104
#define VALUE_ID_FOPR_OBJECT_LENGTH_Y 0x0105
#define VALUE_ID_FOPR_OBJECT_LENGTH_Z 0x0106
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_X 0x0107
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y 0x0108
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z 0x0109

static DebugStrings_t FOPRForeignTransmitterIDDescription = 	{"Foreign transmitter ID", "", &printU32};
static DebugStrings_t FOPRObjectTypeDescription = 				{"ObjectType", "", &printU8};
static DebugStrings_t FOPRActorTypeDescription = 				{"ActorType", "", &printU8};
static DebugStrings_t FOPROperationModeDescription = 			{"OperationMode", "", &printU8};
static DebugStrings_t FOPRMassDescription = 					{"Mass", "[g]", &printU32};
static DebugStrings_t FOPRObjectLengthXDescription = 			{"Object length X", "[mm]", &printU32};
static DebugStrings_t FOPRObjectLengthYDescription = 			{"Object length Y", "[mm]", &printU32};
static DebugStrings_t FOPRObjectLengthZDescription = 			{"Object length Z", "[mm]", &printU32};
static DebugStrings_t FOPRPositionDisplacementXDescription = 	{"Position displacement X", "[mm]", &printI16};
static DebugStrings_t FOPRPositionDisplacementYDescription = 	{"Position displacement Y", "[mm]", &printI16};
static DebugStrings_t FOPRPositionDisplacementZDescription = 	{"Position displacement Z", "[mm]", &printI16};

/*! STRT message */
typedef struct {
	HeaderType header;
	uint16_t StartTimeValueIdU16;
	uint16_t StartTimeContentLengthU16;
	uint32_t StartTimeU32;
	uint16_t GPSWeekValueID;
	uint16_t GPSWeekContentLength;
	uint16_t GPSWeek;
	FooterType footer;
} STRTType;						//27 bytes

//! STRT value IDs
#define VALUE_ID_STRT_GPS_QMS_OF_WEEK 0x0002
#define VALUE_ID_STRT_GPS_WEEK 0x0003


/*! HEAB message */
typedef struct {
	HeaderType header;
	uint16_t HEABStructValueID;
	uint16_t HEABStructContentLength;
	uint32_t GPSQmsOfWeek;
	uint8_t controlCenterStatus;
	FooterType footer;
} HEABType;						//16 bytes

//! HEAB value IDs
#define VALUE_ID_HEAB_STRUCT 0x0090


/*! SYPM message */
typedef struct {
	HeaderType header;
	uint16_t syncPointTimeValueID;
	uint16_t syncPointTimeContentLength;
	uint32_t syncPointTime;
	uint16_t freezeTimeValueID;
	uint16_t freezeTimeContentLength;
	uint32_t freezeTime;
	FooterType footer;
} SYPMType;

//! SYPM value IDs
#define VALUE_ID_SYPM_SYNC_POINT_TIME 0x0001
#define VALUE_ID_SYPM_FREEZE_TIME 0x0002


/*! MTSP message */
typedef struct {
	HeaderType header;
	uint16_t estSyncPointTimeValueID;
	uint16_t estSyncPointTimeContentLength;
	uint32_t estSyncPointTime;
	FooterType footer;
} MTSPType;

//! MTSP value IDs
#define VALUE_ID_MTSP_EST_SYNC_POINT_TIME 0x0001


/*! TRCM message */
typedef struct {
	HeaderType header;
	uint16_t triggerIDValueID;
	uint16_t triggerIDContentLength;
	uint16_t triggerID;
	uint16_t triggerTypeValueID;
	uint16_t triggerTypeContentLength;
	uint16_t triggerType;
	uint16_t triggerTypeParameter1ValueID;
	uint16_t triggerTypeParameter1ContentLength;
	uint32_t triggerTypeParameter1;
	uint16_t triggerTypeParameter2ValueID;
	uint16_t triggerTypeParameter2ContentLength;
	uint32_t triggerTypeParameter2;
	uint16_t triggerTypeParameter3ValueID;
	uint16_t triggerTypeParameter3ContentLength;
	uint32_t triggerTypeParameter3;
	FooterType footer;
} TRCMType;

//! TRCM value IDs
#define VALUE_ID_TRCM_TRIGGER_ID 0x0001
#define VALUE_ID_TRCM_TRIGGER_TYPE 0x0002
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1 0x0011
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2 0x0012
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3 0x0013

/*! TREO message */
typedef struct {
	HeaderType header;
	uint16_t triggerIDValueID;
	uint16_t triggerIDContentLength;
	uint16_t triggerID;
	uint16_t timestamp_qmsowValueID;
	uint16_t timestamp_qmsowContentLength;
	uint32_t timestamp_qmsow;
	FooterType footer;
} TREOType;

//! TREO value IDs
#define VALUE_ID_TREO_TRIGGER_ID 0x0001
#define VALUE_ID_TREO_TRIGGER_TIMESTAMP 0x0002


/*! ACCM message */
typedef struct {
	HeaderType header;
	uint16_t actionIDValueID;
	uint16_t actionIDContentLength;
	uint16_t actionID;
	uint16_t actionTypeValueID;
	uint16_t actionTypeContentLength;
	uint16_t actionType;
	uint16_t actionTypeParameter1ValueID;
	uint16_t actionTypeParameter1ContentLength;
	uint32_t actionTypeParameter1;
	uint16_t actionTypeParameter2ValueID;
	uint16_t actionTypeParameter2ContentLength;
	uint32_t actionTypeParameter2;
	uint16_t actionTypeParameter3ValueID;
	uint16_t actionTypeParameter3ContentLength;
	uint32_t actionTypeParameter3;
	FooterType footer;
} ACCMType;

//! ACCM value IDs
#define VALUE_ID_ACCM_ACTION_ID 0x0002
#define VALUE_ID_ACCM_ACTION_TYPE 0x0003
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM1 0x00A1
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM2 0x00A2
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM3 0x00A3


/*! EXAC message */
typedef struct {
	HeaderType header;
	uint16_t actionIDValueID;
	uint16_t actionIDContentLength;
	uint16_t actionID;
	uint16_t executionTime_qmsoWValueID;
	uint16_t executionTime_qmsoWContentLength;
	uint32_t executionTime_qmsoW;
	FooterType footer;
} EXACType;

//! EXAC value IDs
#define VALUE_ID_EXAC_ACTION_ID 0x0002
#define VALUE_ID_EXAC_ACTION_EXECUTE_TIME 0x0003

/*! CATA message */
// TODO
//! CATA value IDs
// TODO


/*! INSUP message */
typedef struct {
	HeaderType header;
	uint16_t modeValueID;
	uint16_t modeContentLength;
	uint8_t mode;
	FooterType footer;
} INSUPType;

//! INSUP value IDs
#define VALUE_ID_INSUP_MODE 0x0200

/*! PODI message */
typedef struct {
	HeaderType header;
	uint16_t foreignTransmitterIDValueID;
	uint16_t foreignTransmitterIDContentLength;
	uint32_t foreignTransmitterID;
	uint16_t gpsQmsOfWeekValueID;
	uint16_t gpsQmsOfWeekContentLength;
	uint32_t gpsQmsOfWeek;
	uint16_t objectStateValueID;
	uint16_t objectStateContentLength;
	uint8_t  objectState;
	uint16_t xPositionValueID;
	uint16_t xPositionContentLength;
	int32_t  xPosition;
	uint16_t yPositionValueID;
	uint16_t yPositionContentLength;
	int32_t  yPosition;
	uint16_t zPositionValueID;
	uint16_t zPositionContentLength;
	int32_t  zPosition;
	uint16_t headingValueID;
	uint16_t headingContentLength;
	uint16_t heading;
	uint16_t pitchValueID;
	uint16_t pitchContentLength;
	uint16_t pitch;
	uint16_t rollValueID;
	uint16_t rollContentLength;
	uint16_t roll;
	uint16_t longitudinalSpeedValueID;
	uint16_t longitudinalSpeedContentLength;
	int16_t  longitudinalSpeed;
	uint16_t lateralSpeedValueID;
	uint16_t lateralSpeedContentLength;
	int16_t  lateralSpeed;
	FooterType footer;
} PODIType;

//! PODI value IDs
#define VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID	0x00FF
#define VALUE_ID_PODI_GPS_QMS_OF_WEEK			0x010A
#define VALUE_ID_PODI_OBJECT_STATE				0x010C
#define VALUE_ID_PODI_X_POSITION				0x010D
#define VALUE_ID_PODI_Y_POSITION				0x010E
#define VALUE_ID_PODI_Z_POSITION				0x010F
#define VALUE_ID_PODI_HEADING					0x0110
#define VALUE_ID_PODI_PITCH						0x0111
#define VALUE_ID_PODI_ROLL						0x0112
#define VALUE_ID_PODI_LONGITUDINAL_SPEED		0x0113
#define VALUE_ID_PODI_LATERAL_SPEED				0x0114

//! PODI field descriptions
static DebugStrings_t PODIForeignTransmitterIdDescription = {"ForeignTransmitterID",	"",			&printU32};
static DebugStrings_t PODIGpsQmsOfWeekDescription =			{"GpsQmsOfWeek",			"[¼ ms]",	&printU32};
static DebugStrings_t PODIObjectStateDescription =			{"ObjectState",				"",			&printU8};
static DebugStrings_t PODIxPositionDescription =			{"Object X position",		"[mm]",		&printI32};
static DebugStrings_t PODIyPositionDescription =			{"Object Y position",		"[mm]",		&printI32};
static DebugStrings_t PODIzPositionDescription =			{"Object Z position",		"[mm]",		&printI32};
static DebugStrings_t PODIHeadingDescription =				{"Object heading (yaw)",	"[cdeg]",	&printU16};
static DebugStrings_t PODIPitchDescription =				{"Object pitch",			"[cdeg]",	&printU16};
static DebugStrings_t PODIRollDescription =					{"Object roll",				"[cdeg]",	&printU16};
static DebugStrings_t PODILongitudinalSpeedDescription =	{"Longitudinal speed",		"[cm/s]",	&printI16};
static DebugStrings_t PODILateralSpeedDescription =			{"Lateral speed",			"[cm/s]",	&printI16};



/*! RCMM message */
typedef struct {
	HeaderType header;
	uint16_t controlStatusValueID;
	uint16_t controlStatusContentLength;
	uint8_t controlStatus;
	uint16_t speedValueID;
	uint16_t speedContentLength;
	int16_t speed;
	uint16_t steeringValueID;
	uint16_t steeringContentLength;
	int16_t steering;
	uint16_t throttleValueID;
	uint16_t throttleContentLength;
	int16_t throttle;
	uint16_t brakeValueID;
	uint16_t brakeContentLength;
	int16_t brake;
	uint16_t commandValueID;
	uint16_t commandContentLength;
	uint8_t command;
	FooterType footer;
} RCMMType;

//! RCMM value IDs
#define VALUE_ID_RCMM_CONTROL_STATUS			0x0001
#define VALUE_ID_RCMM_SPEED_METER_PER_SECOND	0x0011
#define VALUE_ID_RCMM_STEERING_ANGLE			0x0012
#define VALUE_ID_RCMM_STEERING_PERCENTAGE		0x0031
#define VALUE_ID_RCMM_SPEED_PERCENTAGE			0x0032
#define VALUE_ID_RCMM_THROTTLE_PERCENTAGE		0x0033
#define VALUE_ID_RCMM_BRAKE_PERCENTAGE			0x0034
#define VALUE_ID_RCMM_CONTROL					0xA201

static DebugStrings_t RCMMControlStatusDescription = {"Control Status",	"",	&printU8};
static DebugStrings_t RCMMSpeedDescription_m_s = {"Speed",	"[m/s]",	&printI16};
static DebugStrings_t RCMMSpeedDescriptionPct = {"Speed",	"[%%]",	&printI16};
static DebugStrings_t RCMMThrottleDescriptionPct = {"Throttle",	"[%%]",	&printI16};
static DebugStrings_t RCMMBrakeDescriptionPct = {"Brake",	"[%%]",	&printI16};
static DebugStrings_t RCMMSteeringDescriptionDeg = {"Steering",	"[deg]",	&printI16};
static DebugStrings_t RCMMSteeringDescriptionPct = {"Steering",	"[%%]",	&printI16};
static DebugStrings_t RCMMCommandDescription = {"Command (AstaZero)",	"",	&printU8};

/*! RDCA message - Request Direct Control Action*/
typedef struct {
	HeaderType header;
	uint16_t intendedReceiverIDValueID;
	uint16_t intendedReceiverIDContentLength;
	uint32_t intendedReceiverID;
	uint16_t gpsQmsOfWeekValueID;
	uint16_t gpsQmsOfWeekContentLength;
	uint32_t gpsQmsOfWeek;
	uint16_t steeringActionValueID;
	uint16_t steeringActionContentLength;
	int16_t steeringAction;
	uint16_t speedActionValueID;
	uint16_t speedActionContentLength;
	int16_t speedAction;
	FooterType footer;
} RDCAType;						//27 bytes

//! RDCA value IDs
#define VALUE_ID_RDCA_GPS_QMS_OF_WEEK			0x010A
#define VALUE_ID_RDCA_STEERING_ANGLE 			0x0204
#define VALUE_ID_RDCA_STEERING_PERCENTAGE		0x0205
#define VALUE_ID_RDCA_SPEED_METER_PER_SECOND	0x0206
#define VALUE_ID_RDCA_SPEED_PERCENTAGE			0x0207
#define VALUE_ID_RDCA_INTENDED_RECEIVER			0x0100

//! RDCA field descriptions
static DebugStrings_t RDCAIntendedReceiverDescription = {"Intended receiver ID", "",	&printU32};
static DebugStrings_t RDCAGpsQmsOfWeekDescription = {"GpsQmsOfWeek",	"[¼ ms]",	&printU32};
static DebugStrings_t RDCASteeringDescriptionDeg = {"Steering",		"[deg]",	&printI16};
static DebugStrings_t RDCASteeringDescriptionPct = {"Steering",		"%%",	&printI16};
static DebugStrings_t RDCASpeedDescription_m_s = {"Speed",		"[m/s]",	&printI16};
static DebugStrings_t RDCASpeedDescriptionPct = {"Speed",		"%%",	&printI16};


/*! GDRM message - General Data Request Message*/
typedef struct {
	HeaderType header;
	uint16_t DataCodeValueIdU16;
	uint16_t DataCodeContentLengthU16;
	uint16_t DataCode;
	FooterType footer;
} GDRMType;						//19 bytes

//! GDRM value IDs
#define VALUE_ID_GDRM_DATA_CODE 0x0205


//! GDRM field descriptions
static DebugStrings_t GDRMDataCodeDescription = {"Data code",	"",			&printU32};


/*! DCTI message - Direct Control Transmitter Ids*/
typedef struct {
	HeaderType header;
	uint16_t TotalCountValueIdU16;
	uint16_t TotalCountContentLengthU16;
	uint16_t TotalCount;
	uint16_t CounterValueIdU16;
	uint16_t CounterContentLengthU16;
	uint16_t Counter;
	uint16_t TransmitterIDValueIdU16;
	uint16_t TransmitterIDContentLengthU16;
	uint32_t TransmitterID;
	FooterType footer;
} DCTIType;						//33 bytes

//! DCTI value IDs
#define VALUE_ID_DCTI_TOTAL_COUNT 0x0202
#define VALUE_ID_DCTI_COUNTER 0x0203
#define VALUE_ID_DCTI_TRANSMITTER_ID 0x0010

//! DCTI field descriptions
static DebugStrings_t DCTITotalCountDescription = {"Total count",	"",			&printU32};
static DebugStrings_t DCTICounterDescription = {"Counter",	"",					&printU32};
static DebugStrings_t DCTITransmitterIdDescription = {"TransmitterID",	"",		&printU32};

#pragma pack(pop)

// ************************** static function declarations ********************************************************

static char isValidMessageID(const uint16_t id);
static double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad);
static double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad);

static enum ISOMessageReturnValue convertHEABToHostRepresentation(
		HEABType* HEABData,
		const struct timeval *currentTime,
		const uint8_t TransmitterId,
		HeabMessageDataType* heabData);
static enum ISOMessageReturnValue convertGDRMToHostRepresentation(
		GDRMType* GDRMData,
		GdrmMessageDataType* gdrmData);
static enum ISOMessageReturnValue convertDCTIToHostRepresentation(DCTIType* DCTIData,
		DctiMessageDataType* dctiData);
static enum ISOMessageReturnValue convertRDCAToHostRepresentation(
		RDCAType* RDCAData,const struct timeval* currentTime,
		RequestControlActionType* rdcaData);
static enum ISOMessageReturnValue convertPODIToHostRepresentation(
		PODIType* PODIData,
		const struct timeval* currentTime,
		PeerObjectInjectionType* peerData);
static enum ISOMessageReturnValue convertOPROToHostRepresentation(
		const OPROType* OPROData,
		ObjectPropertiesType* objectProperties);
static enum ISOMessageReturnValue convertFOPRToHostRepresentation(
		const FOPRType* FOPRData,
		ForeignObjectPropertiesType* foreignObjectProperties);
static enum ISOMessageReturnValue convertRCMMToHostRepresentation(RCMMType * RCMMData, 
		RemoteControlManoeuvreMessageType* rcmmData);
static enum ISOMessageReturnValue convertSTRTToHostRepresentation(const STRTType * STRTData, const struct timeval* currentTime,
		StartMessageType * startdata);


/*!
 * \brief isValidMessageID Determines if specified id is a valid ISO message ID. The reserved range is deemed
 * invalid and vendor specific range is deemed valid.
 * \param id An ISO message id to be checked
 * \return 1 if valid, 0 if not
 */
char isValidMessageID(const uint16_t id) {
	return id == MESSAGE_ID_MONR || id == MESSAGE_ID_HEAB || id == MESSAGE_ID_TRAJ || id == MESSAGE_ID_OSEM
		|| id == MESSAGE_ID_OSTM || id == MESSAGE_ID_STRT || id == MESSAGE_ID_MONR2 || id == MESSAGE_ID_SOWM
		|| id == MESSAGE_ID_INFO || id == MESSAGE_ID_RCMM || id == MESSAGE_ID_SYPM || id == MESSAGE_ID_MTSP
		|| id == MESSAGE_ID_TRCM || id == MESSAGE_ID_ACCM || id == MESSAGE_ID_TREO || id == MESSAGE_ID_EXAC
		|| id == MESSAGE_ID_CATA || id == MESSAGE_ID_RCCM || id == MESSAGE_ID_RCRT || id == MESSAGE_ID_PIME
		|| id == MESSAGE_ID_COSE || id == MESSAGE_ID_MOMA || id == MESSAGE_ID_GREM
		|| (id >= MESSAGE_ID_VENDOR_SPECIFIC_LOWER_LIMIT && id <= MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT);
}

/*!
 * \brief getISOMessageType Determines the ISO message type of a raw data buffer
 * \param messageData Buffer containing raw data to be parsed into an ISO message
 * \param length Size of buffer to be parsed
 * \param debug Flag for enabling debugging information
 * \return Value according to ::ISOMessageID
 */
enum ISOMessageID getISOMessageType(const char *messageData, const size_t length, const char debug) {
	HeaderType header;

	// Decode header
	if (decodeISOHeader(messageData, length, &header, debug) != MESSAGE_OK) {
		fprintf(stderr, "Unable to parse raw data into ISO message header\n");
		return MESSAGE_ID_INVALID;
	}

	// Check if header contains valid message ID, if so return it
	if (isValidMessageID(header.messageID))
		return (enum ISOMessageID) header.messageID;
	else {
		printf("Message ID %u does not match any known ISO message\n", header.messageID);
		return MESSAGE_ID_INVALID;
	}
}

/*!
 * \brief encodeSTRTMessage Constructs an ISO STRT message based on start time parameters
 * \param inputHeader data to create header with
 * \param timeOfStart Time when test shall start, a value of NULL indicates that the time is not known
 * \param strtDataBuffer Data buffer in which to place encoded STRT message
 * \param bufferLength Size of data buffer in which to place encoded STRT message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeSTRTMessage(const MessageHeaderType *inputHeader, const StartMessageType* startData, char *strtDataBuffer,
						  const size_t bufferLength, const char debug) {
	STRTType STRTData;

	memset(strtDataBuffer, 0, bufferLength);

	// If buffer too small to hold STRT data, generate an error
	if (bufferLength < sizeof (STRTType)) {
		fprintf(stderr, "Buffer too small to hold necessary STRT data\n");
		return -1;
	}

	STRTData.header = buildISOHeader(MESSAGE_ID_STRT, inputHeader, sizeof (STRTType), debug);

	// Fill contents
	STRTData.StartTimeValueIdU16 = VALUE_ID_STRT_GPS_QMS_OF_WEEK;
	STRTData.StartTimeContentLengthU16 = sizeof (STRTData.StartTimeU32);
	int64_t startTime = getAsGPSQuarterMillisecondOfWeek(&startData->startTime);

	STRTData.StartTimeU32 = startData == NULL || startTime < 0 || !startData->isTimestampValid ?
		GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) startTime;
	STRTData.GPSWeekValueID = VALUE_ID_STRT_GPS_WEEK;
	STRTData.GPSWeekContentLength = sizeof (STRTData.GPSWeek);
	int32_t GPSWeek = getAsGPSWeek(&startData->startTime);

	STRTData.GPSWeek = startData == NULL || GPSWeek < 0 || !startData->isTimestampValid ?
				GPS_WEEK_UNAVAILABLE_VALUE : (uint16_t) GPSWeek;

	if (debug) {
		printf("STRT message:\n\tGPS second of week value ID: 0x%x\n\t"
			   "GPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			   "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\t"
			   "GPS week: %u\n", STRTData.StartTimeValueIdU16, STRTData.StartTimeContentLengthU16,
			   STRTData.StartTimeU32, STRTData.GPSWeekValueID, STRTData.GPSWeekContentLength,
			   STRTData.GPSWeek);
	}

	// Swap from host endianness to little endian
	STRTData.StartTimeValueIdU16 = htole16(STRTData.StartTimeValueIdU16);
	STRTData.StartTimeContentLengthU16 = htole16(STRTData.StartTimeContentLengthU16);
	STRTData.StartTimeU32 = htole32(STRTData.StartTimeU32);
	STRTData.GPSWeekValueID = htole16(STRTData.GPSWeekValueID);
	STRTData.GPSWeekContentLength = htole16(STRTData.GPSWeekContentLength);
	STRTData.GPSWeek = htole16(STRTData.GPSWeek);

	// Construct footer
	STRTData.footer = buildISOFooter(&STRTData, sizeof (STRTType), debug);

	memcpy(strtDataBuffer, &STRTData, sizeof (STRTType));

	return sizeof (STRTType);
}

/*!
 * \brief decodeSTRTMessage Fills a start message struct from a buffer of raw data
 * \param strtDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, for determining current GPS week
 * \param startData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return Number of bytes decoded, or negative value according to ::ISOMessageReturnValue
 */
ssize_t decodeSTRTMessage(
		const char *strtDataBuffer,
		const size_t bufferLength,
		const struct timeval* currentTime,
		StartMessageType * startData,
		const char debug) {

	STRTType STRTData; 
	const char *p = strtDataBuffer;
	const uint16_t ExpectedSTRTStructSize = (uint16_t) (sizeof (STRTData) - sizeof (STRTData.header)
														- sizeof (STRTData.footer.Crc));
	ssize_t retval = MESSAGE_OK;

	if (startData == NULL || strtDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to STRT parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}								

	// Init struct with zeros 
	memset(startData, 0, sizeof (*startData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &STRTData.header, debug)) != MESSAGE_OK) {
		memset(startData, 0, sizeof (*startData));
		return retval;
	}
	p += sizeof (STRTData.header);

	// If message is not a STRT message, generate an error
	if (STRTData.header.messageID != MESSAGE_ID_STRT) {
		fprintf(stderr, "Attempted to pass non-STRT message into STRT parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (STRTData.header.messageLength > sizeof (STRTType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "STRT message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}
	
	// Decode content
	memcpy(&STRTData.StartTimeValueIdU16, p, sizeof (STRTData.StartTimeValueIdU16));
	p += sizeof (STRTData.StartTimeValueIdU16);
	STRTData.StartTimeValueIdU16 = le16toh(STRTData.StartTimeValueIdU16);

	if (STRTData.StartTimeValueIdU16 != VALUE_ID_STRT_GPS_QMS_OF_WEEK) {
		fprintf(stderr, "StartTime Value Id differs from expected\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&STRTData.StartTimeContentLengthU16, p, sizeof (STRTData.StartTimeContentLengthU16));
	p += sizeof (STRTData.StartTimeContentLengthU16);
	STRTData.StartTimeContentLengthU16 = le16toh(STRTData.StartTimeContentLengthU16);

	if (STRTData.StartTimeContentLengthU16 != sizeof(STRTData.StartTimeU32)) {
		fprintf(stderr, "StartTime Content Length %u differs from the expected length %lu\n",
		STRTData.StartTimeContentLengthU16, sizeof(STRTData.StartTimeU32));
		return MESSAGE_CONTENT_OUT_OF_RANGE;
	}

	memcpy(&STRTData.StartTimeU32, p, sizeof (STRTData.StartTimeU32));
	p += sizeof (STRTData.StartTimeU32);
	STRTData.StartTimeU32 = le32toh(STRTData.StartTimeU32);

	memcpy(&STRTData.GPSWeekValueID, p, sizeof (STRTData.GPSWeekValueID));
	p += sizeof (STRTData.GPSWeekValueID);
	STRTData.GPSWeekValueID = le16toh(STRTData.GPSWeekValueID);

	if (STRTData.GPSWeekValueID != VALUE_ID_STRT_GPS_WEEK) {
		fprintf(stderr, "GPSWeek Value Id differs from expected\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&STRTData.GPSWeekContentLength, p, sizeof (STRTData.GPSWeekContentLength));
	p += sizeof (STRTData.GPSWeekContentLength);
	STRTData.GPSWeekContentLength = le16toh(STRTData.GPSWeekContentLength);

	if (STRTData.GPSWeekContentLength != sizeof(STRTData.GPSWeek)) {
		fprintf(stderr, "GPSWeek Content Length %u differs from the expected length %lu\n",
		STRTData.GPSWeekContentLength, sizeof(STRTData.GPSWeek));
		return MESSAGE_CONTENT_OUT_OF_RANGE;
	}

	memcpy(&STRTData.GPSWeek, p, sizeof (STRTData.GPSWeek));
	p += sizeof (STRTData.GPSWeek);
	STRTData.GPSWeek = le16toh(STRTData.GPSWeek);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - strtDataBuffer), &STRTData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding STRT footer\n");
		return retval;
	}
	p += sizeof (STRTData.footer);

	if ((retval = verifyChecksum(&STRTData, sizeof (STRTData) - sizeof (STRTData.footer),
								 STRTData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "STRT checksum error\n");
		return retval;
	}

	if (debug) {
		printf("STRT:\n");
		printf("SyncWord = %x\n", STRTData.header.syncWord);
		printf("TransmitterId = %d\n", STRTData.header.transmitterID);
		printf("PackageCounter = %d\n", STRTData.header.messageCounter);
		printf("AckReq = %d\n", STRTData.header.ackReqProtVer);
		printf("MessageId = %d\n", STRTData.header.messageID);
		printf("MessageLength = %d\n", STRTData.header.messageLength);
		printf("StartTime value ID: 0x%x\n", STRTData.StartTimeValueIdU16);
		printf("StartTime content length: %u\n", STRTData.StartTimeContentLengthU16);
 		printf("StartTime value: %u\n", STRTData.StartTimeU32);
		printf("GPSWeek value ID: 0x%x\n", STRTData.GPSWeekValueID);
		printf("GPSWeek content length: %u\n", STRTData.GPSWeekContentLength);
		printf("GPSWeek value: %u\n", STRTData.GPSWeek);
	}


	// Fill output struct with parsed data
	convertSTRTToHostRepresentation(&STRTData,currentTime,startData);

	return retval < 0 ? retval : p - strtDataBuffer;

}

/*!
 * \brief convertSTRTToHostRepresentation Converts a STRT message to the internal representation for
 * start data
 * \param STRTData STRT message to be converted
 * \param startData Struct in which result is to be placed
 */

enum ISOMessageReturnValue convertSTRTToHostRepresentation(
		const STRTType * STRTData,
		const struct timeval* currentTime,
		StartMessageType * startData) {
	uint16_t gpsWeek = 0;
	if (currentTime) {
		gpsWeek = (uint16_t)getAsGPSWeek(currentTime);
	}

	if (!STRTData || !startData) {
		errno = EINVAL;
		fprintf(stderr, "STRT input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	if (STRTData->StartTimeU32 == GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE) {
		startData->isTimestampValid = 0;
		return MESSAGE_OK;
	}
	else if (STRTData->GPSWeek == GPS_WEEK_UNAVAILABLE_VALUE) {
		if (!currentTime) {
			startData->isTimestampValid = 0;
			return MESSAGE_OK;
		}
		setToGPStime(&startData->startTime, gpsWeek, STRTData->StartTimeU32);
	}
	else {
		if (currentTime && STRTData->GPSWeek != gpsWeek) {
			fprintf(stderr, "Parsed STRT message with non-matching GPS week");
			startData->isTimestampValid = 0;
			return MESSAGE_OK;
		}
		setToGPStime(&startData->startTime, STRTData->GPSWeek, STRTData->StartTimeU32);
	}
	return MESSAGE_OK;
}

/*!
 * \brief encodeHEABMessage Constructs an ISO HEAB message based on current control center status and system time
 * \param inputHeader data to create header with
 * \param heabTime Timestamp to be placed in heab struct
 * \param status Current control center status according to ::ControlCenterStatusType. Entering an unaccepable value
 *	makes this parameter default to ABORT
 * \param heabDataBuffer Buffer to which HEAB message is to be written
 * \param bufferLength Size of buffer to which HEAB message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeHEABMessage(const MessageHeaderType *inputHeader, const struct timeval *heabTime, const enum ControlCenterStatusType status,
						  char *heabDataBuffer, const size_t bufferLength, const char debug) {

	HEABType HEABData;

	memset(heabDataBuffer, 0, bufferLength);

	// If buffer too small to hold HEAB data, generate an error
	if (bufferLength < sizeof (HEABType)) {
		fprintf(stderr, "Buffer too small to hold necessary HEAB data\n");
		return -1;
	}

	// Construct header
	HEABData.header = buildISOHeader(MESSAGE_ID_HEAB, inputHeader, sizeof (HEABData), debug);

	// Fill contents
	HEABData.HEABStructValueID = VALUE_ID_HEAB_STRUCT;
	HEABData.HEABStructContentLength = sizeof (HEABType) - sizeof (HeaderType) - sizeof (FooterType)
		- sizeof (HEABData.HEABStructValueID) - sizeof (HEABData.HEABStructContentLength);
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(heabTime);

	HEABData.GPSQmsOfWeek =
		GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (!
		(status == CONTROL_CENTER_STATUS_INIT || status == CONTROL_CENTER_STATUS_READY
		 || status == CONTROL_CENTER_STATUS_ABORT || status == CONTROL_CENTER_STATUS_RUNNING
		 || status == CONTROL_CENTER_STATUS_TEST_DONE || status == CONTROL_CENTER_STATUS_NORMAL_STOP)) {
		printf("HEAB does not support status ID %u - defaulting to ABORT\n", (uint8_t) status);
		HEABData.controlCenterStatus = (uint8_t) CONTROL_CENTER_STATUS_ABORT;
	}
	else {
		HEABData.controlCenterStatus = (uint8_t) status;
	}

	if (debug) {
		printf("HEAB message:\n\tHEAB struct value ID: 0x%x\n\t"
			   "HEAB struct content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			   "Control center status: 0x%x\n", HEABData.HEABStructValueID, HEABData.HEABStructContentLength,
			   HEABData.GPSQmsOfWeek, HEABData.controlCenterStatus);
	}

	// Switch from host endianness to little endian
	HEABData.HEABStructValueID = htole16(HEABData.HEABStructValueID);
	HEABData.HEABStructContentLength = htole16(HEABData.HEABStructContentLength);
	HEABData.GPSQmsOfWeek = htole32(HEABData.GPSQmsOfWeek);

	HEABData.footer = buildISOFooter(&HEABData, sizeof (HEABData), debug);

	memcpy(heabDataBuffer, &HEABData, sizeof (HEABData));

	return sizeof (HEABType);

}

/*!
 * \brief decodeHEABMessage Fills HEAB data elements from a buffer of raw data
 * \param heabDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used for determining the GPS week
 * \param heabData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeHEABMessage(const char *heabDataBuffer,
										const size_t bufferLength,
										const struct timeval currentTime,
										HeabMessageDataType* heabData,
										const char debug) {

	HEABType HEABData;
	const char *p = heabDataBuffer;
	enum ISOMessageReturnValue retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (heabDataBuffer == NULL || heabData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to HEAB parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&HEABData, 0, sizeof (HEABData));
	memset(heabData, 0, sizeof (*heabData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &HEABData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (HEABData.header);

	// If message is not a HEAB message, generate an error
	if (HEABData.header.messageID != MESSAGE_ID_HEAB) {
		fprintf(stderr, "Attempted to pass non-HEAB message into HEAB parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	
	if (HEABData.header.messageLength > sizeof (HEABType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "HEAB message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}
	
	memcpy(&valueID, p, sizeof (valueID));
	p += sizeof (valueID);
	memcpy(&contentLength, p, sizeof (contentLength));
	p += sizeof (contentLength);
	valueID = le16toh(valueID);
	contentLength = le16toh(contentLength);
	HEABData.HEABStructValueID = valueID;
	HEABData.HEABStructContentLength = contentLength;

	if (contentLength != (sizeof(HEABData.GPSQmsOfWeek) + sizeof(HEABData.controlCenterStatus))) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, sizeof(HEABData.GPSQmsOfWeek) + sizeof(HEABData.controlCenterStatus));
			return MESSAGE_LENGTH_ERROR;
	}

	memcpy(&HEABData.GPSQmsOfWeek, p, sizeof (HEABData.GPSQmsOfWeek));
	HEABData.GPSQmsOfWeek = le32toh(HEABData.GPSQmsOfWeek);
	p += sizeof (HEABData.GPSQmsOfWeek);
	memcpy(&HEABData.controlCenterStatus, p, sizeof (HEABData.controlCenterStatus));
	p += sizeof (HEABData.controlCenterStatus);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - heabDataBuffer), &HEABData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding HEAB footer\n");
		return retval;
	}
	p += sizeof (HEABData.footer);

	if (debug) {
		printf("HEAB message:\n");
		printf("\tStruct value ID: 0x%x\n", HEABData.HEABStructValueID);
		printf("\tStruct content length: %u\n", HEABData.HEABStructContentLength);
		printf("\tGPSQmsOfWeek: %u\n", HEABData.GPSQmsOfWeek);
		printf("\tControlCenterStatus: %u\n", HEABData.controlCenterStatus);
	}

	retval = convertHEABToHostRepresentation(&HEABData, &currentTime, HEABData.header.transmitterID, heabData);

	return retval < 0 ? retval : p - heabDataBuffer;
}



/*!
 * \brief convertHEABToHostRepresentation Converts a HEAB message to be used by host
 * \param HEABData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param transmitterId of the HEAB sender
 * \param heabData Output data struct, to be used by host
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertHEABToHostRepresentation(HEABType* HEABData,
		const struct timeval *currentTime,
		const uint8_t transmitterId,
		HeabMessageDataType* heabData) {

	
	if (HEABData == NULL || heabData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "HEAB input pointer error");
		return ISO_FUNCTION_ERROR;
	}


	heabData->transmitterID = transmitterId;
	setToGPStime(&heabData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), HEABData->GPSQmsOfWeek);
	heabData->controlCenterStatus = HEABData->controlCenterStatus;


	return MESSAGE_OK;
}

/*!
 * \brief decodeRCMMessage Fills RCCM data elements from a buffer of raw data
 * \param rcmmDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param rcmmData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeRCMMMessage(
		const char *rcmmDataBuffer,
		const size_t bufferLength,
		RemoteControlManoeuvreMessageType* rcmmData,
		const char debug) {

	RCMMType RCMMData;
	const char *p = rcmmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (rcmmDataBuffer == NULL || rcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to RCMM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&RCMMData, 0, sizeof (RCMMData));
	memset(rcmmData, 0, sizeof(*rcmmData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &RCMMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (RCMMData.header);


	// If message is not a RCMM message, generate an error
	if (RCMMData.header.messageID != MESSAGE_ID_RCMM) {
		fprintf(stderr, "Attempted to pass non-RCMM message into RCMM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (RCMMData.header.messageLength > sizeof (RCMMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "RCMM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - rcmmDataBuffer < RCMMData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);
		switch (valueID) {
		case VALUE_ID_RCMM_CONTROL_STATUS:
			memcpy(&RCMMData.controlStatus, p, sizeof (RCMMData.controlStatus));
			RCMMData.controlStatusValueID = valueID;
			RCMMData.controlStatusContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.controlStatus);
			break;
		case VALUE_ID_RCMM_SPEED_METER_PER_SECOND:
			memcpy(&RCMMData.speed, p, sizeof (RCMMData.speed));
			RCMMData.speed = (int16_t)le16toh (RCMMData.speed);
			RCMMData.speedValueID = valueID;
			RCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.speed);
			break;
		case VALUE_ID_RCMM_STEERING_ANGLE:
			memcpy(&RCMMData.steering, p, sizeof (RCMMData.steering));
			RCMMData.steering = (int16_t)le16toh (RCMMData.steering);
			RCMMData.steeringValueID = valueID;
			RCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.steering);
			break;
		case VALUE_ID_RCMM_STEERING_PERCENTAGE:
			memcpy(&RCMMData.steering, p, sizeof (RCMMData.steering));
			RCMMData.steering = (int16_t)le16toh (RCMMData.steering); 
			RCMMData.steeringValueID = valueID;
			RCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.steering);
			break;
		case VALUE_ID_RCMM_SPEED_PERCENTAGE:
			memcpy(&RCMMData.speed, p, sizeof (RCMMData.speed));
			RCMMData.speed = (int16_t)le16toh (RCMMData.speed);
			RCMMData.speedValueID = valueID;
			RCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.speed);
			break;
		case VALUE_ID_RCMM_CONTROL:
			memcpy(&RCMMData.command, p, sizeof (RCMMData.command));
			RCMMData.commandValueID = valueID;
			RCMMData.commandContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.command);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known RCMM value IDs\n", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld\n",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - rcmmDataBuffer), &RCMMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding RCMM footer\n");
		return retval;
	}

	p += sizeof (RCMMData.footer);
	if ((retval = verifyChecksum(rcmmDataBuffer, RCMMData.header.messageLength + sizeof (HeaderType),
								 RCMMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "RCMM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("RCMM message:\n");
		printContent(RCMMData.controlStatusValueID, RCMMData.controlStatusContentLength,
					 &RCMMData.controlStatus, &RCMMControlStatusDescription);
		if (RCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE)
			printContent(RCMMData.steeringValueID, RCMMData.steeringContentLength,
					 	&RCMMData.steering, &RCMMSteeringDescriptionDeg);
		else if (RCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE)
			printContent(RCMMData.steeringValueID, RCMMData.steeringContentLength,
					 	&RCMMData.steering, &RCMMSteeringDescriptionPct);
		if (RCMMData.speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND)
			printContent(RCMMData.speedValueID, RCMMData.speedContentLength,
					 	&RCMMData.speed, &RCMMSpeedDescription_m_s);
		else if (RCMMData.speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE)
			printContent(RCMMData.speedValueID, RCMMData.speedContentLength,
				&RCMMData.speed, &RCMMSpeedDescriptionPct);
	}

	retval = convertRCMMToHostRepresentation(&RCMMData, rcmmData);

	return retval < 0 ? retval : p - rcmmDataBuffer;
}
/**
 * @brief Convert a RCMM message to host represntation
 * 
 * @param RCMMData Data struct containing ISO formated data
 * @param rcmmData Output data struct
 * @return ISOMessageReturnValue 
 */
enum ISOMessageReturnValue convertRCMMToHostRepresentation(RCMMType * RCMMData, 
		RemoteControlManoeuvreMessageType* rcmmData) {
	
	if (RCMMData == NULL ||  rcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "RCMM input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	// Fill steering values and check out of range values
	rcmmData->status = RCMMData->controlStatus;
	if (RCMMData->steering != STEERING_ANGLE_UNAVAILABLE_VALUE && RCMMData->steeringValueID) {
		if(RCMMData->steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE) {
			if (RCMMData->steering <= STEERING_ANGLE_MAX_VALUE_DEG
			&& RCMMData->steering >= STEERING_ANGLE_MIN_VALUE_DEG) {
				rcmmData->isSteeringManoeuvreValid= 1;
				rcmmData->steeringManoeuvre.rad = RCMMData->steering / STEERING_ANGLE_ONE_DEGREE_VALUE * (M_PI / 180.0);
				rcmmData->steeringUnit = ISO_UNIT_TYPE_STEERING_DEGREES; 
			}
			else {
				fprintf(stderr, "Steering angle value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else if(RCMMData->steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE) {
			if (RCMMData->steering <= MAX_VALUE_PERCENTAGE
			&& RCMMData->steering >= MIN_VALUE_PERCENTAGE) {
				rcmmData->isSteeringManoeuvreValid = 1;
				rcmmData->steeringManoeuvre.pct = RCMMData->steering;
				rcmmData->steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Steering percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else {
			fprintf(stderr, "Steering Value ID error\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}

	// Fill speed values and check out of range values
	if (RCMMData->speed != SPEED_UNAVAILABLE_VALUE && RCMMData->speedValueID) {
		if(RCMMData->speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND) {
				rcmmData->isSpeedManoeuvreValid = 1;
				rcmmData->speedManoeuvre.m_s = RCMMData->speed / SPEED_ONE_METER_PER_SECOND_VALUE;
				rcmmData->speedUnit = ISO_UNIT_TYPE_SPEED_METER_SECOND; 
			}
		else if(RCMMData->speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE) {
			if (RCMMData->speed <= MAX_VALUE_PERCENTAGE 
				&& RCMMData->speed >= MIN_VALUE_PERCENTAGE) {
				rcmmData->isSpeedManoeuvreValid = 1;
				rcmmData->speedManoeuvre.pct = RCMMData->speed;
				rcmmData->speedUnit = ISO_UNIT_TYPE_SPEED_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Speed percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
			
		}
		else {
			fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}
	return MESSAGE_OK;

}

/*!
 * \brief encodeRCMMMessage Fills an ISO RCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param rcmmData Struct containing relevant RCMM data
 * \param rcmmDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeRCMMMessage(const MessageHeaderType *inputHeader, const RemoteControlManoeuvreMessageType* rcmmData,
		char* rcmmDataBuffer,
		const size_t bufferLength,
		const char debug) {

	RCMMType RCMMData;

	memset(rcmmDataBuffer, 0, bufferLength);
	char* p = rcmmDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (rcmmDataBuffer == NULL) {
		fprintf(stderr, "RCMM data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold RCMM data, generate an error
	if (bufferLength < sizeof (RCMMType)) {
		fprintf(stderr, "Buffer too small to hold necessary RCMM data\n");
		return -1;
	}
	// Construct header
	RCMMData.header = buildISOHeader(MESSAGE_ID_RCMM, inputHeader, sizeof (RCMMData), debug);
	memcpy(p, &RCMMData.header, sizeof (RCMMData.header));
	p += sizeof (RCMMData.header);
	remainingBytes -= sizeof (RCMMData.header);

	if (debug) {
			printf("RCMM message:\n");
	}
	retval |= encodeContent(VALUE_ID_RCMM_CONTROL_STATUS, &rcmmData->status, &p,
						sizeof (rcmmData->status), &remainingBytes, &RCMMControlStatusDescription, debug);
	
	if (rcmmData->command != MANOEUVRE_NONE) {
		RCMMData.command = (uint8_t)(rcmmData->command);
		retval |= encodeContent(VALUE_ID_RCMM_CONTROL, &RCMMData.command, &p,
								sizeof (RCMMData.command), &remainingBytes, &RCMMCommandDescription, debug);
	}

	if (rcmmData->isSteeringManoeuvreValid && rcmmData->steeringUnit == ISO_UNIT_TYPE_STEERING_DEGREES) {
		if(rcmmData->steeringManoeuvre.rad <= STEERING_ANGLE_MAX_VALUE_RAD && rcmmData->steeringManoeuvre.rad >= STEERING_ANGLE_MIN_VALUE_RAD) {
			RCMMData.steering = (int16_t) (rcmmData->steeringManoeuvre.rad * (180.0 / M_PI) * STEERING_ANGLE_ONE_DEGREE_VALUE);
			retval |= encodeContent(VALUE_ID_RCMM_STEERING_ANGLE, &RCMMData.steering, &p,
							  sizeof (RCMMData.steering), &remainingBytes, &RCMMSteeringDescriptionDeg, debug);
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for angle value\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else if (rcmmData->isSteeringManoeuvreValid && rcmmData->steeringUnit == ISO_UNIT_TYPE_STEERING_PERCENTAGE) {
		if(rcmmData->steeringManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->steeringManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.steering = (int16_t) (rcmmData->steeringManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_STEERING_PERCENTAGE, &RCMMData.steering, &p,
						 	  sizeof(RCMMData.steering), &remainingBytes, &RCMMSteeringDescriptionPct, debug);		
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}

	if(rcmmData->isSpeedManoeuvreValid && rcmmData->speedUnit == ISO_UNIT_TYPE_SPEED_METER_SECOND) {
		RCMMData.speed = (int16_t) (rcmmData->speedManoeuvre.m_s *SPEED_ONE_METER_PER_SECOND_VALUE);
		retval |= encodeContent(VALUE_ID_RCMM_SPEED_METER_PER_SECOND, &RCMMData.speed, &p,
								sizeof(RCMMData.speed), &remainingBytes, &RCMMSpeedDescription_m_s, debug);
	}
	else if(rcmmData->isSpeedManoeuvreValid && rcmmData->speedUnit == ISO_UNIT_TYPE_SPEED_PERCENTAGE) {
		if (rcmmData->speedManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->speedManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.speed = (int16_t) (rcmmData->speedManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_SPEED_PERCENTAGE, &RCMMData.speed, &p,
			sizeof(RCMMData.speed), &remainingBytes, &RCMMSpeedDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Speed value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}

	if (rcmmData->isThrottleManoeuvreValid && rcmmData->throttleUnit == ISO_UNIT_TYPE_THROTTLE_PERCENTAGE) {
		if (rcmmData->throttleManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->throttleManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.throttle = (int16_t) (rcmmData->throttleManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_THROTTLE_PERCENTAGE, &RCMMData.throttle, &p,
			sizeof(RCMMData.throttle), &remainingBytes, &RCMMThrottleDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Throttle value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else {
		fprintf(stderr, "Throttle unit is not valid\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	if (rcmmData->isBrakeManoeuvreValid && rcmmData->brakeUnit == ISO_UNIT_TYPE_BRAKE_PERCENTAGE) {
		if (rcmmData->brakeManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->brakeManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.brake = (int16_t) (rcmmData->brakeManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_BRAKE_PERCENTAGE, &RCMMData.brake, &p,
			sizeof(RCMMData.brake), &remainingBytes, &RCMMBrakeDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Brake value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else {
		fprintf(stderr, "Brake unit is not valid\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary RCMM data\n");
		return -1;
	}
	
	// Construct footer
	RCMMData.footer = buildISOFooter(&RCMMData, (size_t) (p-rcmmDataBuffer)+ sizeof(RCMMData.footer), debug);
	
	memcpy(p, &RCMMData.footer, sizeof (RCMMData.footer));
	p += sizeof (RCMMData.footer);
	remainingBytes -= sizeof (RCMMData.footer);

	if(debug)
	{
		printf("RCMM message data (size = %zu):\n", sizeof (RCMMType));
		for(size_t i = 0; i < sizeof (RCMMType); i++) printf("%x ", *(rcmmDataBuffer+i));
		printf("\n");
	}

	return p - rcmmDataBuffer;
}

/*!
 * \brief encodeSYPMMessage Fills an ISO SYPM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param synchronizationTime Time along trajectory at which objects are to be synchronized
 * \param freezeTime Time along trajectory after which no further adaptation to the master is allowed
 * \param mtspDataBuffer Buffer to which SYPM message is to be written
 * \param bufferLength Size of buffer to which SYPM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeSYPMMessage(const MessageHeaderType *inputHeader, const struct timeval synchronizationTime, const struct timeval freezeTime,
						  char *sypmDataBuffer, const size_t bufferLength, const char debug) {
 
	SYPMType SYPMData;

	// If buffer too small to hold SYPM data, generate an error
	if (bufferLength < sizeof (SYPMType)) {
		fprintf(stderr, "Buffer too small to hold necessary SYPM data\n");
		return -1;
	}

	// Construct header
	SYPMData.header = buildISOHeader(MESSAGE_ID_SYPM, inputHeader, sizeof (SYPMData), debug);

	// Fill contents
	SYPMData.syncPointTimeValueID = VALUE_ID_SYPM_SYNC_POINT_TIME;
	SYPMData.syncPointTimeContentLength = sizeof (SYPMData.syncPointTime);
	SYPMData.syncPointTime =
		(uint32_t) (synchronizationTime.tv_sec * 1000 + synchronizationTime.tv_usec / 1000);

	SYPMData.freezeTimeValueID = VALUE_ID_SYPM_FREEZE_TIME;
	SYPMData.freezeTimeContentLength = sizeof (SYPMData.freezeTime);
	SYPMData.freezeTime = (uint32_t) (freezeTime.tv_sec * 1000 + freezeTime.tv_usec / 1000);

	if (debug) {
		printf("SYPM message:\n\tSynchronization point time value ID: 0x%x\n\t"
			   "Synchronization point time content length: %u\n\t"
			   "Synchronization point time: %u [ms]\n\t"
			   "Freeze time value ID: 0x%x\n\t"
			   "Freeze time content length: %u\n\t"
			   "Freeze time: %u [ms]\n", SYPMData.syncPointTimeValueID,
			   SYPMData.syncPointTimeContentLength, SYPMData.syncPointTime,
			   SYPMData.freezeTimeValueID, SYPMData.freezeTimeContentLength, SYPMData.freezeTime);
	}

	// Switch from host endianness to little endian
	SYPMData.syncPointTimeValueID = htole16(SYPMData.syncPointTimeValueID);
	SYPMData.syncPointTimeContentLength = htole16(SYPMData.syncPointTimeContentLength);
	SYPMData.syncPointTime = htole16(SYPMData.syncPointTime);
	SYPMData.freezeTimeValueID = htole16(SYPMData.freezeTimeValueID);
	SYPMData.freezeTimeContentLength = htole16(SYPMData.freezeTimeContentLength);
	SYPMData.freezeTime = htobe16(SYPMData.freezeTime);

	// Construct footer
	SYPMData.footer = buildISOFooter(&SYPMData, sizeof (SYPMData), debug);

	memcpy(sypmDataBuffer, &SYPMData, sizeof (SYPMData));

	return sizeof (SYPMType);
}

/*!
 * \brief encodeMTSPMessage Fills an ISO MTSP struct with relevant data fields, and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param estSyncPointTime Estimated time when the master object will reach the synchronization point
 * \param mtspDataBuffer Buffer to which MTSP message is to be written
 * \param bufferLength Size of buffer to which MTSP message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeMTSPMessage(const MessageHeaderType *inputHeader, const struct timeval *estSyncPointTime, char *mtspDataBuffer,
						  const size_t bufferLength, const char debug) {
	MTSPType MTSPData;

	memset(mtspDataBuffer, 0, bufferLength);

	// If buffer too small to hold MTSP data, generate an error
	if (bufferLength < sizeof (MTSPType)) {
		fprintf(stderr, "Buffer too small to hold necessary MTSP data\n");
		return -1;
	}

	// Construct header
	MTSPData.header = buildISOHeader(MESSAGE_ID_MTSP, inputHeader, sizeof (MTSPData), debug);

	// Fill contents
	MTSPData.estSyncPointTimeValueID = VALUE_ID_MTSP_EST_SYNC_POINT_TIME;
	MTSPData.estSyncPointTimeContentLength = sizeof (MTSPData.estSyncPointTime);
	int64_t syncPtTime = getAsGPSQuarterMillisecondOfWeek(estSyncPointTime);

	MTSPData.estSyncPointTime = estSyncPointTime == NULL || syncPtTime < 0 ?
		ESTIMATED_SYNC_POINT_TIME_UNAVAILABLE_VALUE : (uint32_t) syncPtTime;

	if (debug) {
		printf("MTSP message:\n\t"
			   "Estimated sync point time value ID: 0x%x\n\t"
			   "Estimated sync point time content length: %u\n\t"
			   "Estimated sync point time: %u [¼ ms]\n",
			   MTSPData.estSyncPointTimeValueID, MTSPData.estSyncPointTimeContentLength,
			   MTSPData.estSyncPointTime);
	}

	// Switch from host endianness to little endian
	MTSPData.estSyncPointTimeValueID = htole16(MTSPData.estSyncPointTimeValueID);
	MTSPData.estSyncPointTimeContentLength = htole16(MTSPData.estSyncPointTimeContentLength);
	MTSPData.estSyncPointTime = htole32(MTSPData.estSyncPointTime);

	// Construct footer
	MTSPData.footer = buildISOFooter(&MTSPData, sizeof (MTSPData), debug);

	memcpy(mtspDataBuffer, &MTSPData, sizeof (MTSPData));

	return sizeof (MTSPType);
}

/*!
 * \brief encodeTRCMMessage Fills an ISO TRCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param triggerID ID of the trigger to be configured
 * \param triggerType Type of the trigger to be configured according to ::TriggerType_t
 * \param param1 First parameter of the trigger to be configured according to ::TriggerTypeParameter_t
 * \param param2 Second parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param param3 Third parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param trcmDataBuffer Buffer to which TRCM message is to be written
 * \param bufferLength Size of buffer to which TRCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeTRCMMessage(const MessageHeaderType *inputHeader, const uint16_t * triggerID, const enum TriggerType_t * triggerType,
						  const enum TriggerTypeParameter_t * param1, const enum TriggerTypeParameter_t * param2,
						  const enum TriggerTypeParameter_t * param3, char *trcmDataBuffer,
						  const size_t bufferLength, const char debug) {
	TRCMType TRCMData;

	memset(trcmDataBuffer, 0, bufferLength);

	// If buffer too small to hold TRCM data, generate an error
	if (bufferLength < sizeof (TRCMType)) {
		fprintf(stderr, "Buffer too small to hold necessary TRCM data\n");
		return -1;
	}

	// Construct header
	TRCMData.header = buildISOHeader(MESSAGE_ID_TRCM, inputHeader, sizeof (TRCMData), debug);

	// Fill contents
	TRCMData.triggerIDValueID = VALUE_ID_TRCM_TRIGGER_ID;
	TRCMData.triggerIDContentLength = sizeof (TRCMData.triggerID);
	TRCMData.triggerID = triggerID == NULL ? TRIGGER_ID_UNAVAILABLE : *triggerID;

	TRCMData.triggerTypeValueID = VALUE_ID_TRCM_TRIGGER_TYPE;
	TRCMData.triggerTypeContentLength = sizeof (TRCMData.triggerType);
	TRCMData.triggerType = triggerType == NULL ? TRIGGER_TYPE_UNAVAILABLE : (uint16_t) (*triggerType);

	TRCMData.triggerTypeParameter1ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1;
	TRCMData.triggerTypeParameter2ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2;
	TRCMData.triggerTypeParameter3ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3;

	TRCMData.triggerTypeParameter1ContentLength = sizeof (TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2ContentLength = sizeof (TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3ContentLength = sizeof (TRCMData.triggerTypeParameter3);

	TRCMData.triggerTypeParameter1 =
		param1 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	TRCMData.triggerTypeParameter2 =
		param2 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	TRCMData.triggerTypeParameter3 =
		param3 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		printf("TRCM message:\n\tTrigger ID value ID: 0x%x\n\tTrigger ID content length: %u\n\t"
			   "Trigger ID: %u\n\tTrigger type value ID: 0x%x\n\tTrigger type content length: %u\n\t"
			   "Trigger type: %u\n\tTrigger type parameter 1 value ID: 0x%x\n\tTrigger type parameter 1 content length: %u\n\t"
			   "Trigger type parameter 1: %u\n\tTrigger type parameter 2 value ID: 0x%x\n\tTrigger type parameter 2 content length: %u\n\t"
			   "Trigger type parameter 2: %u\n\tTrigger type parameter 3 value ID: 0x%x\n\tTrigger type parameter 3 content length: %u"
			   "Trigger type parameter 3: %u\n\t", TRCMData.triggerIDValueID,
			   TRCMData.triggerIDContentLength, TRCMData.triggerID, TRCMData.triggerTypeValueID,
			   TRCMData.triggerTypeContentLength, TRCMData.triggerType,
			   TRCMData.triggerTypeParameter1ValueID, TRCMData.triggerTypeParameter1ContentLength,
			   TRCMData.triggerTypeParameter1, TRCMData.triggerTypeParameter2ValueID,
			   TRCMData.triggerTypeParameter2ContentLength, TRCMData.triggerTypeParameter2,
			   TRCMData.triggerTypeParameter3ValueID, TRCMData.triggerTypeParameter3ContentLength,
			   TRCMData.triggerTypeParameter3);
	}

	// Switch from host endianness to little endian
	TRCMData.triggerIDValueID = htole16(TRCMData.triggerIDValueID);
	TRCMData.triggerIDContentLength = htole16(TRCMData.triggerIDContentLength);
	TRCMData.triggerID = htole16(TRCMData.triggerID);

	TRCMData.triggerTypeValueID = htole16(TRCMData.triggerTypeValueID);
	TRCMData.triggerTypeContentLength = htole16(TRCMData.triggerTypeContentLength);
	TRCMData.triggerType = htole16(TRCMData.triggerType);

	TRCMData.triggerTypeParameter1ValueID = htole16(TRCMData.triggerTypeParameter1ValueID);
	TRCMData.triggerTypeParameter2ValueID = htole16(TRCMData.triggerTypeParameter2ValueID);
	TRCMData.triggerTypeParameter3ValueID = htole16(TRCMData.triggerTypeParameter3ValueID);

	TRCMData.triggerTypeParameter1ContentLength = htole16(TRCMData.triggerTypeParameter1ContentLength);
	TRCMData.triggerTypeParameter2ContentLength = htole16(TRCMData.triggerTypeParameter2ContentLength);
	TRCMData.triggerTypeParameter3ContentLength = htole16(TRCMData.triggerTypeParameter3ContentLength);

	TRCMData.triggerTypeParameter1 = htole32(TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2 = htole32(TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3 = htole32(TRCMData.triggerTypeParameter3);

	// Construct footer
	TRCMData.footer = buildISOFooter(&TRCMData, sizeof (TRCMData), debug);

	memcpy(trcmDataBuffer, &TRCMData, sizeof (TRCMData));

	return sizeof (TRCMData);
}


/*!
 * \brief encodeACCMMessage Fills an ISO ACCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param actionID ID of the action to be configured
 * \param actionType Type of the action to be configured according to ::ActionType_t
 * \param param1 First parameter of the action to be configured according to ::ActionTypeParameter_t
 * \param param2 Second parameter of the action to be configured ::ActionTypeParameter_t
 * \param param3 Third parameter of the action to be configured ::ActionTypeParameter_t
 * \param trcmDataBuffer Buffer to which ACCM message is to be written
 * \param bufferLength Size of buffer to which ACCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeACCMMessage(const MessageHeaderType *inputHeader, const uint16_t * actionID, const enum ActionType_t * actionType,
						  const enum ActionTypeParameter_t * param1, const enum ActionTypeParameter_t * param2,
						  const enum ActionTypeParameter_t * param3, char *accmDataBuffer,
						  const size_t bufferLength, const char debug) {

	ACCMType ACCMData;

	memset(accmDataBuffer, 0, bufferLength);

	// If buffer too small to hold ACCM data, generate an error
	if (bufferLength < sizeof (ACCMType)) {
		fprintf(stderr, "Buffer too small to hold necessary ACCM data\n");
		return -1;
	}

	// Construct header
	ACCMData.header = buildISOHeader(MESSAGE_ID_ACCM, inputHeader, sizeof (ACCMData), debug);

	// Fill contents
	ACCMData.actionIDValueID = VALUE_ID_ACCM_ACTION_ID;
	ACCMData.actionIDContentLength = sizeof (ACCMData.actionID);
	ACCMData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	ACCMData.actionTypeValueID = VALUE_ID_ACCM_ACTION_TYPE;
	ACCMData.actionTypeContentLength = sizeof (ACCMData.actionType);
	ACCMData.actionType = actionType == NULL ? ACTION_TYPE_UNAVAILABLE : (uint16_t) (*actionType);

	ACCMData.actionTypeParameter1ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM1;
	ACCMData.actionTypeParameter2ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM2;
	ACCMData.actionTypeParameter3ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM3;

	ACCMData.actionTypeParameter1ContentLength = sizeof (ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2ContentLength = sizeof (ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3ContentLength = sizeof (ACCMData.actionTypeParameter3);

	ACCMData.actionTypeParameter1 = param1 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	ACCMData.actionTypeParameter2 = param2 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	ACCMData.actionTypeParameter3 = param3 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		printf("ACCM message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\t"
			   "Action ID: %u\n\tAction type value ID: 0x%x\n\tAction type content length: %u\n\t"
			   "Action type: %u\n\tAction type parameter 1 value ID: 0x%x\n\tAction type parameter 1 content length: %u\n\t"
			   "Action type parameter 1: %u\n\tAction type parameter 2 value ID: 0x%x\n\tAction type parameter 2 content length: %u\n\t"
			   "Action type parameter 2: %u\n\tAction type parameter 3 value ID: 0x%x\n\tAction type parameter 3 content length: %u"
			   "Action type parameter 3: %u\n\t", ACCMData.actionIDValueID, ACCMData.actionIDContentLength,
			   ACCMData.actionID, ACCMData.actionTypeValueID, ACCMData.actionTypeContentLength,
			   ACCMData.actionType, ACCMData.actionTypeParameter1ValueID,
			   ACCMData.actionTypeParameter1ContentLength, ACCMData.actionTypeParameter1,
			   ACCMData.actionTypeParameter2ValueID, ACCMData.actionTypeParameter2ContentLength,
			   ACCMData.actionTypeParameter2, ACCMData.actionTypeParameter3ValueID,
			   ACCMData.actionTypeParameter3ContentLength, ACCMData.actionTypeParameter3);
	}

	// Switch from host endianness to little endian
	ACCMData.actionIDValueID = htole16(ACCMData.actionIDValueID);
	ACCMData.actionIDContentLength = htole16(ACCMData.actionIDContentLength);
	ACCMData.actionID = htole16(ACCMData.actionID);

	ACCMData.actionTypeValueID = htole16(ACCMData.actionTypeValueID);
	ACCMData.actionTypeContentLength = htole16(ACCMData.actionTypeContentLength);
	ACCMData.actionType = htole16(ACCMData.actionType);

	ACCMData.actionTypeParameter1ValueID = htole16(ACCMData.actionTypeParameter1ValueID);
	ACCMData.actionTypeParameter2ValueID = htole16(ACCMData.actionTypeParameter2ValueID);
	ACCMData.actionTypeParameter3ValueID = htole16(ACCMData.actionTypeParameter3ValueID);

	ACCMData.actionTypeParameter1ContentLength = htole16(ACCMData.actionTypeParameter1ContentLength);
	ACCMData.actionTypeParameter2ContentLength = htole16(ACCMData.actionTypeParameter2ContentLength);
	ACCMData.actionTypeParameter3ContentLength = htole16(ACCMData.actionTypeParameter3ContentLength);

	ACCMData.actionTypeParameter1 = htole32(ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2 = htole32(ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3 = htole32(ACCMData.actionTypeParameter3);

	// Construct footer
	ACCMData.footer = buildISOFooter(&ACCMData, sizeof (ACCMData), debug);

	memcpy(accmDataBuffer, &ACCMData, sizeof (ACCMData));

	return sizeof (ACCMData);
}

/*!
 * \brief encodeEXACMessage Fills an ISO EXAC struct with relevant data fields, and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param actionID ID of the action to be executed
 * \param executionTime Time when the action is to be executed
 * \param exacDataBuffer Buffer to which EXAC message is to be written
 * \param bufferLength Size of buffer to which EXAC message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeEXACMessage(const MessageHeaderType *inputHeader, const uint16_t * actionID, const struct timeval *executionTime,
						  char *exacDataBuffer, const size_t bufferLength, const char debug) {

	EXACType EXACData;

	memset(exacDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (EXACType)) {
		fprintf(stderr, "Buffer too small to hold necessary EXAC data\n");
		return -1;
	}

	// Construct header
	EXACData.header = buildISOHeader(MESSAGE_ID_EXAC, inputHeader, sizeof (EXACData), debug);

	// Fill contents
	EXACData.actionIDValueID = VALUE_ID_EXAC_ACTION_ID;
	EXACData.actionIDContentLength = sizeof (EXACData.actionID);
	EXACData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	EXACData.executionTime_qmsoWValueID = VALUE_ID_EXAC_ACTION_EXECUTE_TIME;
	EXACData.executionTime_qmsoWContentLength = sizeof (EXACData.executionTime_qmsoW);
	int64_t execTime = getAsGPSQuarterMillisecondOfWeek(executionTime);

	EXACData.executionTime_qmsoW =
		executionTime == NULL || execTime < 0 ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) execTime;

	if (debug) {
		printf
			("EXAC message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\tAction ID: %u\n\t"
			 "Action execute time value ID: 0x%x\n\tAction execute time content length: %u\n\tAction execute time: %u [¼ ms]\n",
			 EXACData.actionIDValueID, EXACData.actionIDContentLength, EXACData.actionID,
			 EXACData.executionTime_qmsoWValueID, EXACData.executionTime_qmsoWContentLength,
			 EXACData.executionTime_qmsoW);
	}

	// Switch from host endianness to little endian
	EXACData.actionIDValueID = htole16(EXACData.actionIDValueID);
	EXACData.actionIDContentLength = htole16(EXACData.actionIDContentLength);
	EXACData.actionID = htole16(EXACData.actionID);
	EXACData.executionTime_qmsoWValueID = htole16(EXACData.executionTime_qmsoWValueID);
	EXACData.executionTime_qmsoWContentLength = htole16(EXACData.executionTime_qmsoWContentLength);
	EXACData.executionTime_qmsoW = htole32(EXACData.executionTime_qmsoW);

	// Construct footer
	EXACData.footer = buildISOFooter(&EXACData, sizeof (EXACData), debug);

	memcpy(exacDataBuffer, &EXACData, sizeof (EXACData));

	return sizeof (EXACType);
}

/*!
 * \brief encodeINSUPMessage Fills an ISO vendor specific (RISE) INSUP struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param header ISO header to be used for the message
 * \param command Command to send to supervisor
 * \param insupDataBuffer Data buffer to which INSUP is to be written
 * \param bufferLength Length of data buffer to which INSUP is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeINSUPMessage(const MessageHeaderType *inputHeader, const enum SupervisorCommandType command, char *insupDataBuffer,
						   const size_t bufferLength, const char debug) {
	INSUPType INSUPData;

	memset(insupDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (INSUPType)) {
		fprintf(stderr, "Buffer too small to hold necessary INSUP data\n");
		return -1;
	}

	// Construct header
	INSUPData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_RISE_INSUP, inputHeader, sizeof (INSUPData), debug);

	// Fill contents
	INSUPData.modeValueID = VALUE_ID_INSUP_MODE;
	INSUPData.modeContentLength = sizeof (INSUPData.mode);
	INSUPData.mode = (uint8_t) command;

	if (debug) {
		printf("INSUP message:\n\tMode value ID: 0x%x\n\t"
			   "Mode content length: %u\n\tMode: %u\n", INSUPData.modeValueID,
			   INSUPData.modeContentLength, INSUPData.mode);
	}

	// Switch from host endianness to little endian
	INSUPData.modeValueID = htole16(INSUPData.modeValueID);
	INSUPData.modeContentLength = htole16(INSUPData.modeContentLength);

	// Construct footer
	INSUPData.footer = buildISOFooter(&INSUPData, sizeof (INSUPData), debug);

	memcpy(insupDataBuffer, &INSUPData, sizeof (INSUPData));

	return sizeof (INSUPData);
}

/*!
 * \brief encodePODIMessage Fills an ISO vendor specific (AstaZero) PODI struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param peerObjectData Struct containing relevant PODI data
 * \param podiDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodePODIMessage(const MessageHeaderType *inputHeader, const PeerObjectInjectionType* peerObjectData, 
		char* podiDataBuffer,
		const size_t bufferLength,
		const char debug) {

	PODIType PODIData;

	memset(podiDataBuffer, 0, bufferLength);
	char* p = podiDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (peerObjectData == NULL) {
		fprintf(stderr, "PODI data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold PODI data, generate an error
	if (bufferLength < sizeof (PODIType)) {
		fprintf(stderr, "Buffer too small to hold necessary PODI data\n");
		return -1;
	}

	// Construct header
	PODIData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_PODI, inputHeader, sizeof (PODIData), debug);
	memcpy(p, &PODIData.header, sizeof (PODIData.header));
	p += sizeof (PODIData.header);
	remainingBytes -= sizeof (PODIData.header);

	if (debug) {
			printf("PODI message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID, &peerObjectData->foreignTransmitterID, &p,
						  sizeof (peerObjectData->foreignTransmitterID), &remainingBytes, &PODIForeignTransmitterIdDescription, debug);
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&peerObjectData->dataTimestamp);
	PODIData.gpsQmsOfWeek = GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_GPS_QMS_OF_WEEK, &PODIData.gpsQmsOfWeek, &p,
						  sizeof (PODIData.gpsQmsOfWeek), &remainingBytes, &PODIGpsQmsOfWeekDescription, debug);
	PODIData.objectState = (uint8_t) peerObjectData->state;
	retval |= encodeContent(VALUE_ID_PODI_OBJECT_STATE, &PODIData.objectState, &p,
						  sizeof (PODIData.objectState), &remainingBytes, &PODIObjectStateDescription, debug);
	if (!peerObjectData->position.isPositionValid) {
		errno = EINVAL;
		fprintf(stderr, "Position is a required field in PODI messages\n");
		return -1;
	}
	PODIData.xPosition = (int32_t) (peerObjectData->position.xCoord_m * POSITION_ONE_METER_VALUE);
	PODIData.yPosition = (int32_t) (peerObjectData->position.yCoord_m * POSITION_ONE_METER_VALUE);
	PODIData.zPosition = (int32_t) (peerObjectData->position.zCoord_m * POSITION_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_PODI_X_POSITION, &PODIData.xPosition, &p,
						  sizeof (PODIData.xPosition), &remainingBytes, &PODIxPositionDescription, debug);
	retval |= encodeContent(VALUE_ID_PODI_Y_POSITION, &PODIData.yPosition, &p,
						  sizeof (PODIData.yPosition), &remainingBytes, &PODIyPositionDescription, debug);
	retval |= encodeContent(VALUE_ID_PODI_Z_POSITION, &PODIData.zPosition, &p,
						  sizeof (PODIData.zPosition), &remainingBytes, &PODIzPositionDescription, debug);
	PODIData.heading = peerObjectData->position.isHeadingValid ?
				(uint16_t) (mapHostHeadingToISOHeading(peerObjectData->position.heading_rad)
					* 180.0 / M_PI * YAW_ONE_DEGREE_VALUE)
			  : YAW_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_HEADING, &PODIData.heading, &p,
						  sizeof (PODIData.heading), &remainingBytes, &PODIHeadingDescription, debug);
	PODIData.pitch = peerObjectData->isPitchValid ?
				(uint16_t)(peerObjectData->pitch_rad * 180.0 / M_PI * PITCH_ONE_DEGREE_VALUE)
			  : PITCH_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_PITCH, &PODIData.pitch, &p,
						  sizeof (PODIData.pitch), &remainingBytes, &PODIPitchDescription, debug);
	PODIData.roll = peerObjectData->isRollValid ?
				(uint16_t)(peerObjectData->roll_rad * 180.0 / M_PI * ROLL_ONE_DEGREE_VALUE)
			  : ROLL_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_ROLL, &PODIData.roll, &p,
						  sizeof (PODIData.roll), &remainingBytes, &PODIRollDescription, debug);
	PODIData.longitudinalSpeed = peerObjectData->speed.isLongitudinalValid ?
				(int16_t) (peerObjectData->speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE)
			  : SPEED_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_LONGITUDINAL_SPEED, &PODIData.longitudinalSpeed, &p,
						  sizeof (PODIData.longitudinalSpeed), &remainingBytes, &PODILongitudinalSpeedDescription, debug);
	PODIData.lateralSpeed = peerObjectData->speed.isLateralValid ?
				(int16_t) (peerObjectData->speed.lateral_m_s * SPEED_ONE_METER_PER_SECOND_VALUE)
			  : SPEED_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_LATERAL_SPEED, &PODIData.lateralSpeed, &p,
						  sizeof (PODIData.lateralSpeed), &remainingBytes, &PODILongitudinalSpeedDescription, debug);

	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary PODI data\n");
		return -1;
	}

	if (debug) {
		printf("PODI message:\n\t<<debug printout not implemented>>\n");
	}

	// Construct footer
	PODIData.footer = buildISOFooter(&PODIData, sizeof (PODIData), debug);
	memcpy(p, &PODIData.footer, sizeof (PODIData.footer));
	p += sizeof (PODIData.footer);
	remainingBytes -= sizeof (PODIData.footer);

	if(debug)
	{
		printf("PODI message data (size = %zu):\n", sizeof (PODIType));
		for(int i = 0; i < sizeof (PODIType); i++) printf("%x ", *(podiDataBuffer+i));
		printf("\n");
	}

	return p - podiDataBuffer;
}

/*!
 * \brief decodePODIMessage Fills PODI data elements from a buffer of raw data
 * \param podiDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of PODI message
 * \param peerData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodePODIMessage(
		const char *podiDataBuffer,
		const size_t bufferLength,
		const struct timeval currentTime,
		PeerObjectInjectionType* peerData,
		const char debug) {

	PODIType PODIData;
	const char *p = podiDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (podiDataBuffer == NULL || peerData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to PODI parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&PODIData, 0, sizeof (PODIData));
	memset(peerData, 0, sizeof (*peerData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &PODIData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (PODIData.header);

	// If message is not a PODI message, generate an error
	if (PODIData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_PODI) {
		fprintf(stderr, "Attempted to pass non-PODI message into PODI parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (PODIData.header.messageLength > sizeof (PODIType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "PODI message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - podiDataBuffer < PODIData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID:
			memcpy(&PODIData.foreignTransmitterID, p, sizeof (PODIData.foreignTransmitterID));
			PODIData.foreignTransmitterIDValueID = valueID;
			PODIData.foreignTransmitterIDContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.foreignTransmitterID);
			break;
		case VALUE_ID_PODI_GPS_QMS_OF_WEEK:
			memcpy(&PODIData.gpsQmsOfWeek, p, sizeof (PODIData.gpsQmsOfWeek));
			PODIData.gpsQmsOfWeek = le32toh(PODIData.gpsQmsOfWeek);
			PODIData.gpsQmsOfWeekValueID = valueID;
			PODIData.gpsQmsOfWeekContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.gpsQmsOfWeek);
			break;
		case VALUE_ID_PODI_X_POSITION:
			memcpy(&PODIData.xPosition, p, sizeof (PODIData.xPosition));
			PODIData.xPosition = le32toh(PODIData.xPosition);
			PODIData.xPositionValueID = valueID;
			PODIData.xPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.xPosition);
			break;
		case VALUE_ID_PODI_Y_POSITION:
			memcpy(&PODIData.yPosition, p, sizeof (PODIData.yPosition));
			PODIData.yPosition = le32toh(PODIData.yPosition);
			PODIData.yPositionValueID = valueID;
			PODIData.yPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.yPosition);
			break;
		case VALUE_ID_PODI_Z_POSITION:
			memcpy(&PODIData.zPosition, p, sizeof (PODIData.zPosition));
			PODIData.zPosition = le32toh(PODIData.zPosition);
			PODIData.zPositionValueID = valueID;
			PODIData.zPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.zPosition);
			break;
		case VALUE_ID_PODI_OBJECT_STATE:
			memcpy(&PODIData.objectState, p, sizeof (PODIData.objectState));
			PODIData.objectStateValueID = valueID;
			PODIData.objectStateContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.objectState);
			break;
		case VALUE_ID_PODI_HEADING:
			memcpy(&PODIData.heading, p, sizeof (PODIData.heading));
			PODIData.heading = le16toh(PODIData.heading);
			PODIData.headingValueID = valueID;
			PODIData.headingContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.heading);
			break;
		case VALUE_ID_PODI_PITCH:
			memcpy(&PODIData.pitch, p, sizeof (PODIData.pitch));
			PODIData.pitch = le16toh(PODIData.pitch);
			PODIData.pitchValueID = valueID;
			PODIData.pitchContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.pitch);
			break;
		case VALUE_ID_PODI_ROLL:
			memcpy(&PODIData.roll, p, sizeof (PODIData.roll));
			PODIData.roll = le16toh(PODIData.roll);
			PODIData.rollValueID = valueID;
			PODIData.rollContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.roll);
			break;
		case VALUE_ID_PODI_LATERAL_SPEED:
			memcpy(&PODIData.lateralSpeed, p, sizeof (PODIData.lateralSpeed));
			PODIData.lateralSpeed = le16toh(PODIData.lateralSpeed);
			PODIData.lateralSpeedValueID = valueID;
			PODIData.lateralSpeedContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.lateralSpeed);
			break;
		case VALUE_ID_PODI_LONGITUDINAL_SPEED:
			memcpy(&PODIData.longitudinalSpeed, p, sizeof (PODIData.longitudinalSpeed));
			PODIData.longitudinalSpeed = le16toh(PODIData.longitudinalSpeed);
			PODIData.longitudinalSpeedValueID = valueID;
			PODIData.longitudinalSpeedContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.longitudinalSpeed);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known PODI value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - podiDataBuffer), &PODIData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding PODI footer\n");
		return retval;
	}
	p += sizeof (PODIData.footer);

	/*if ((retval = verifyChecksum(podiDataBuffer, PODIData.header.messageLength + sizeof (HeaderType),
								 PODIData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "PODI checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("PODI message:\n");
		printf("\tForeign transmitter ID value ID: 0x%x\n", PODIData.foreignTransmitterIDValueID);
		printf("\tForeign transmitter ID content length: %u\n", PODIData.foreignTransmitterIDContentLength);
		printf("\tForeign transmitter ID: %u\n", PODIData.foreignTransmitterID);
		printf("\tGPS second of week value ID: 0x%x\n", PODIData.gpsQmsOfWeekValueID);
		printf("\tGPS second of week content length: %u\n", PODIData.gpsQmsOfWeekContentLength);
		printf("\tGPS second of week: %u [¼ ms]\n", PODIData.gpsQmsOfWeek);
		printf("\tObject state value ID: 0x%x\n", PODIData.objectStateValueID);
		printf("\tObject state content length: %u\n", PODIData.objectStateContentLength);
		printf("\tObject state: %u\n", PODIData.objectState);
		printf("\tx position value ID: 0x%x\n", PODIData.xPositionValueID);
		printf("\tx position content length: %u\n", PODIData.xPositionContentLength);
		printf("\tx position: %u\n", PODIData.xPosition);
		printf("\ty position value ID: 0x%x\n", PODIData.yPositionValueID);
		printf("\ty position content length: %u\n", PODIData.yPositionContentLength);
		printf("\ty position: %u\n", PODIData.yPosition);
		printf("\tz position value ID: 0x%x\n", PODIData.zPositionValueID);
		printf("\tz position content length: %u\n", PODIData.zPositionContentLength);
		printf("\tz position: %u\n", PODIData.zPosition);
		printf("\tHeading value ID: 0x%x\n", PODIData.headingValueID);
		printf("\tHeading content length: %u\n", PODIData.headingContentLength);
		printf("\tHeading: %u\n", PODIData.heading);
		printf("\tPitch value ID: 0x%x\n", PODIData.pitchValueID);
		printf("\tPitch content length: %u\n", PODIData.pitchContentLength);
		printf("\tPitch: %u\n", PODIData.pitch);
		printf("\tRoll value ID: 0x%x\n", PODIData.rollValueID);
		printf("\tRoll content length: %u\n", PODIData.rollContentLength);
		printf("\tRoll: %u\n", PODIData.roll);
		printf("\tLongitudinal speed value ID: 0x%x\n", PODIData.longitudinalSpeedValueID);
		printf("\tLongitudinal speed content length: %u\n", PODIData.longitudinalSpeedContentLength);
		printf("\tLongitudinal speed: %u\n", PODIData.longitudinalSpeed);
		printf("\tLateral speed value ID: 0x%x\n", PODIData.lateralSpeedValueID);
		printf("\tLateral speed content length: %u\n", PODIData.lateralSpeedContentLength);
		printf("\tLateral speed: %u\n", PODIData.lateralSpeed);
	}

	retval = convertPODIToHostRepresentation(&PODIData, &currentTime, peerData);

	return retval < 0 ? retval : p - podiDataBuffer;
}

/*!
 * \brief convertPODIToHostRepresentation Converts a PODI message to SI representation
 * \param PODIData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertPODIToHostRepresentation(PODIType* PODIData,
		const struct timeval* currentTime,
		PeerObjectInjectionType* peerData) {

	if (PODIData == NULL || currentTime == NULL || peerData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "PODI input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	if (!PODIData->foreignTransmitterIDValueID) {
		fprintf(stderr, "Foreign transmitter ID not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->foreignTransmitterID = PODIData->foreignTransmitterID;

	if (!PODIData->gpsQmsOfWeekValueID
			|| PODIData->gpsQmsOfWeek == GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE) {
		fprintf(stderr, "Timestamp not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	setToGPStime(&peerData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), PODIData->gpsQmsOfWeek);

	peerData->state = PODIData->objectStateValueID ?
				PODIData->objectState : OBJECT_STATE_UNKNOWN;

	if (!PODIData->xPositionValueID
			|| !PODIData->yPositionValueID
			|| !PODIData->zPositionValueID) {
		fprintf(stderr, "Position not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->position.isPositionValid = 1;
	peerData->position.xCoord_m = PODIData->xPosition / POSITION_ONE_METER_VALUE;
	peerData->position.yCoord_m = PODIData->yPosition / POSITION_ONE_METER_VALUE;
	peerData->position.zCoord_m = PODIData->zPosition / POSITION_ONE_METER_VALUE;

	if (!PODIData->headingValueID) {
		fprintf(stderr, "Heading not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->position.isHeadingValid = PODIData->heading != YAW_UNAVAILABLE_VALUE;
	peerData->position.heading_rad = mapISOHeadingToHostHeading(
				PODIData->heading / YAW_ONE_DEGREE_VALUE * M_PI / 180.0);

	if (PODIData->rollValueID && PODIData->roll != ROLL_UNAVAILABLE_VALUE) {
		peerData->isRollValid = 1;
		peerData->roll_rad = PODIData->roll / ROLL_ONE_DEGREE_VALUE * M_PI / 180.0;
	}
	else {
		peerData->isRollValid = 0;
	}

	if (PODIData->pitchValueID && PODIData->pitch != PITCH_UNAVAILABLE_VALUE) {
		peerData->isPitchValid = 1;
		peerData->pitch_rad = PODIData->pitch / PITCH_ONE_DEGREE_VALUE * M_PI / 180.0;
	}
	else {
		peerData->isRollValid = 0;
	}

	// Until these are clearly defined, force them to be invalid
	peerData->isRollValid = 0;
	peerData->isPitchValid = 0;

	return MESSAGE_OK;
}

/*!
 * \brief decodeOPROMessage Decodes a buffer containing OPRO data into an object properties struct
 * \param objectPropertiesData Struct to be filled
 * \param oproDataBuffer Buffer containing data to be decoded
 * \param bufferLength Size of buffer containing data to be decoded
 * \param debug Parameter for enabling debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t decodeOPROMessage(
		ObjectPropertiesType * objectPropertiesData,
		const char *oproDataBuffer,
		const size_t bufferLength,
		const char debug) {
	OPROType OPROData;
	const char *p = oproDataBuffer;

	uint16_t valueID;
	uint16_t contentLength;

	if (objectPropertiesData == NULL || oproDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	ssize_t retval = MESSAGE_OK;

	memset(objectPropertiesData, 0, sizeof (*objectPropertiesData));
	memset(&OPROData, 0, sizeof (OPROData));

	if ((retval = decodeISOHeader(p, bufferLength, &OPROData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (OPROData.header);
	

	// If message is not a OPRO message, generate an error
	if (OPROData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO) {
		fprintf(stderr, "Attempted to pass non-OPRO message into OPRO parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}


	// Decode contents
	while ((size_t) (p - oproDataBuffer) < OPROData.header.messageLength + sizeof (OPROData.header)) {
		// Decode value ID and length
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);

		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		// Handle contents
		switch (valueID) {
		case VALUE_ID_OPRO_OBJECT_TYPE:
			memcpy(&OPROData.objectTypeValueID, &valueID, sizeof (OPROData.objectTypeValueID));
			memcpy(&OPROData.objectTypeContentLength, &contentLength,
				   sizeof (OPROData.objectTypeContentLength));
			memcpy(&OPROData.objectType, p, sizeof (OPROData.objectType));
			break;
		case VALUE_ID_OPRO_ACTOR_TYPE:
			memcpy(&OPROData.actorTypeValueID, &valueID, sizeof (OPROData.actorTypeValueID));
			memcpy(&OPROData.actorTypeContentLength, &contentLength,
				   sizeof (OPROData.actorTypeContentLength));
			memcpy(&OPROData.actorType, p, sizeof (OPROData.actorType));
			break;
		case VALUE_ID_OPRO_OPERATION_MODE:
			memcpy(&OPROData.operationModeValueID, &valueID, sizeof (OPROData.operationModeValueID));
			memcpy(&OPROData.operationModeContentLength, &contentLength,
				   sizeof (OPROData.operationModeContentLength));
			memcpy(&OPROData.operationMode, p, sizeof (OPROData.operationMode));
			break;
		case VALUE_ID_OPRO_MASS:
			memcpy(&OPROData.massValueID, &valueID, sizeof (OPROData.massValueID));
			memcpy(&OPROData.massContentLength, &contentLength, sizeof (OPROData.massContentLength));
			memcpy(&OPROData.mass, p, sizeof (OPROData.mass));
			OPROData.mass = le32toh(OPROData.mass);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_X:
			memcpy(&OPROData.objectLengthXValueID, &valueID, sizeof (OPROData.objectLengthXValueID));
			memcpy(&OPROData.objectLengthXContentLength, &contentLength,
				   sizeof (OPROData.objectLengthXContentLength));
			memcpy(&OPROData.objectLengthX, p, sizeof (OPROData.objectLengthX));
			OPROData.objectLengthX = le32toh(OPROData.objectLengthX);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_Y:
			memcpy(&OPROData.objectLengthYValueID, &valueID, sizeof (OPROData.objectLengthYValueID));
			memcpy(&OPROData.objectLengthYContentLength, &contentLength,
				   sizeof (OPROData.objectLengthYContentLength));
			memcpy(&OPROData.objectLengthY, p, sizeof (OPROData.objectLengthY));
			OPROData.objectLengthY = le32toh(OPROData.objectLengthY);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_Z:
			memcpy(&OPROData.objectLengthZValueID, &valueID, sizeof (OPROData.objectLengthZValueID));
			memcpy(&OPROData.objectLengthZContentLength, &contentLength,
				   sizeof (OPROData.objectLengthZContentLength));
			memcpy(&OPROData.objectLengthZ, p, sizeof (OPROData.objectLengthZ));
			OPROData.objectLengthZ = le32toh(OPROData.objectLengthZ);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_X:
			memcpy(&OPROData.positionDisplacementXValueID, &valueID,
				   sizeof (OPROData.positionDisplacementXValueID));
			memcpy(&OPROData.positionDisplacementXContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementXContentLength));
			memcpy(&OPROData.positionDisplacementX, p, sizeof (OPROData.positionDisplacementX));
			OPROData.positionDisplacementX = le16toh(OPROData.positionDisplacementX);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y:
			memcpy(&OPROData.positionDisplacementYValueID, &valueID,
				   sizeof (OPROData.positionDisplacementYValueID));
			memcpy(&OPROData.positionDisplacementYContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementYContentLength));
			memcpy(&OPROData.positionDisplacementY, p, sizeof (OPROData.positionDisplacementY));
			OPROData.positionDisplacementY = le16toh(OPROData.positionDisplacementY);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z:
			memcpy(&OPROData.positionDisplacementZValueID, &valueID,
				   sizeof (OPROData.positionDisplacementZValueID));
			memcpy(&OPROData.positionDisplacementZContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementZContentLength));
			memcpy(&OPROData.positionDisplacementZ, p, sizeof (OPROData.positionDisplacementZ));
			OPROData.positionDisplacementZ = le16toh(OPROData.positionDisplacementZ);
			break;

		default:
			printf("Unable to handle OPRO value ID 0x%x\n", valueID);
			break;
		}
		p += contentLength;
	}


	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - oproDataBuffer), &OPROData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding OPRO footer\n");
		return retval;
	}
	p += sizeof (OPROData.footer);

	if ((retval = verifyChecksum(oproDataBuffer, OPROData.header.messageLength + sizeof (OPROData.header),
								 OPROData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "OPRO checksum error\n");
		return retval;
	}

	if (debug) {
		printf("OPRO message:\n");
		printf("\tObject type value ID: 0x%x\n", OPROData.objectTypeValueID);
		printf("\tObject type content length: %u\n", OPROData.objectTypeContentLength);
		printf("\tObject type: %u\n", OPROData.objectType);
		printf("\tActor type value ID: 0x%x\n", OPROData.actorTypeValueID);
		printf("\tActor type content length: %u\n", OPROData.actorTypeContentLength);
		printf("\tActor type: %u\n", OPROData.actorType);
		printf("\tOperation mode value ID: 0x%x\n", OPROData.operationModeValueID);
		printf("\tOperation mode content length: %u\n", OPROData.operationModeContentLength);
		printf("\tOperation mode: %u\n", OPROData.operationMode);
		printf("\tMass value ID: 0x%x\n", OPROData.massValueID);
		printf("\tMass content length: %u\n", OPROData.massContentLength);
		printf("\tMass: %u [g]\n", OPROData.mass);
		printf("\tObject length X value ID: 0x%x\n", OPROData.objectLengthXValueID);
		printf("\tObject length X content length: %u\n", OPROData.objectLengthXContentLength);
		printf("\tObject length X: %u [mm]\n", OPROData.objectLengthX);
		printf("\tObject length Y value ID: 0x%x\n", OPROData.objectLengthYValueID);
		printf("\tObject length Y content length: %u\n", OPROData.objectLengthYContentLength);
		printf("\tObject length Y: %u [mm]\n", OPROData.objectLengthY);
		printf("\tObject length Z value ID: 0x%x\n", OPROData.objectLengthZValueID);
		printf("\tObject length Z content length: %u\n", OPROData.objectLengthZContentLength);
		printf("\tObject length Z: %u [mm]\n", OPROData.objectLengthZ);
		printf("\tPosition displacement X value ID: 0x%x\n", OPROData.positionDisplacementXValueID);
		printf("\tPosition displacement X content length: %u\n", OPROData.positionDisplacementXContentLength);
		printf("\tPosition displacement X: %d [mm]\n", OPROData.positionDisplacementX);
		printf("\tPosition displacement Y value ID: 0x%x\n", OPROData.positionDisplacementYValueID);
		printf("\tPosition displacement Y content length: %u\n", OPROData.positionDisplacementYContentLength);
		printf("\tPosition displacement Y: %d [mm]\n", OPROData.positionDisplacementY);
		printf("\tPosition displacement Z value ID: 0x%x\n", OPROData.positionDisplacementZValueID);
		printf("\tPosition displacement Z content length: %u\n", OPROData.positionDisplacementZContentLength);
		printf("\tPosition displacement Z: %d [mm]\n", OPROData.positionDisplacementZ);
	}

	// Fill output struct with parsed data
	retval = convertOPROToHostRepresentation(&OPROData, objectPropertiesData);
	return retval < 0 ? retval : p - oproDataBuffer;
}

/*!
 * \brief encodeOPROMessage Fills an ISO vendor specific (AstaZero) OPRO struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param objectPropertiesData Struct containing relevant OPRO data
 * \param oproDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeOPROMessage(
		const MessageHeaderType *inputHeader,
		const ObjectPropertiesType* objectPropertiesData,
		char *oproDataBuffer,
		const size_t bufferLength,
		const char debug) {
	OPROType OPROData;

	memset(oproDataBuffer, 0, bufferLength);
	char* p = oproDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (objectPropertiesData == NULL) {
		fprintf(stderr, "OPRO data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold OPRO data, generate an error
	if (bufferLength < sizeof (OPROType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}

	// Construct header
	OPROData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO, inputHeader, sizeof (OPROData), debug);
	memcpy(p, &OPROData.header, sizeof (OPROData.header));
	p += sizeof (OPROData.header);
	remainingBytes -= sizeof (OPROData.header);

	if (debug) {
			printf("OPRO message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_TYPE, &objectPropertiesData->objectType, &p,
							sizeof (OPROData.objectType), &remainingBytes, &OPROObjectTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_OPRO_ACTOR_TYPE, &objectPropertiesData->actorType, &p,
							sizeof (OPROData.actorType), &remainingBytes, &OPROActorTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_OPRO_OPERATION_MODE, &objectPropertiesData->operationMode, &p,
							sizeof (OPROData.operationMode), &remainingBytes, &OPROOperationModeDescription, debug);

	if (objectPropertiesData->isMassValid) OPROData.mass = (uint32_t)(objectPropertiesData->mass_kg * MASS_ONE_KILOGRAM_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_MASS, &OPROData.mass, &p,
								sizeof (OPROData.mass), &remainingBytes, &OPROMassDescription, debug);

	if (objectPropertiesData->isObjectXDimensionValid) OPROData.objectLengthX = (uint32_t)(objectPropertiesData->objectXDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_X, &OPROData.objectLengthX, &p,
								sizeof (OPROData.objectLengthX), &remainingBytes, &OPROObjectLengthXDescription, debug);
 
	if (objectPropertiesData->isObjectYDimensionValid) OPROData.objectLengthY = (uint32_t)(objectPropertiesData->objectYDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_Y, &OPROData.objectLengthY, &p,
								sizeof (OPROData.objectLengthY), &remainingBytes, &OPROObjectLengthYDescription, debug);

	if (objectPropertiesData->isObjectZDimensionValid) OPROData.objectLengthZ = (uint32_t)(objectPropertiesData->objectZDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_Z, &OPROData.objectLengthZ, &p,
								sizeof (OPROData.objectLengthZ), &remainingBytes, &OPROObjectLengthZDescription, debug);

	if (objectPropertiesData->isObjectXDisplacementValid) OPROData.positionDisplacementX = (int16_t)(objectPropertiesData->positionDisplacementX_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_X, &OPROData.positionDisplacementX, &p,
								sizeof (OPROData.positionDisplacementX), &remainingBytes, &OPROPositionDisplacementXDescription, debug);

	if (objectPropertiesData->isObjectYDisplacementValid) OPROData.positionDisplacementY = (int16_t)(objectPropertiesData->positionDisplacementY_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y, &OPROData.positionDisplacementY, &p,
								sizeof (OPROData.positionDisplacementY), &remainingBytes, &OPROPositionDisplacementYDescription, debug);

	if (objectPropertiesData->isObjectZDisplacementValid) OPROData.positionDisplacementZ = (int16_t)(objectPropertiesData->positionDisplacementZ_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z, &OPROData.positionDisplacementZ, &p,
								sizeof (OPROData.positionDisplacementZ), &remainingBytes, &OPROPositionDisplacementZDescription, debug);

	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}
	
	// Construct footer
	OPROData.footer = buildISOFooter(oproDataBuffer, (size_t)(p - oproDataBuffer) + sizeof (FooterType), debug);
	memcpy(p, &OPROData.footer, sizeof (OPROData.footer));
	p += sizeof (OPROData.footer);
	remainingBytes -= sizeof (OPROData.footer);
	if(debug)
	{
		printf("OPRO message data (size = %zu):\n", sizeof (OPROType));
		for(int i = 0; i < sizeof (OPROType); i++) printf("%x ", *(oproDataBuffer+i));
		printf("\n");
	}
	return p - oproDataBuffer;
}

/*!
 * \brief encodeFOPRMessage Fills an ISO vendor specific (AstaZero) FOPR struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param foreignObjectPropertiesData Struct containing relevant OPRO data
 * \param foprDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeFOPRMessage(
		const MessageHeaderType *inputHeader,
		const ForeignObjectPropertiesType* foreignObjectPropertiesData,
		char *foprDataBuffer,
		const size_t bufferLength,
		const char debug) {
	FOPRType FOPRData;

	memset(foprDataBuffer, 0, bufferLength);
	char* p = foprDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (foreignObjectPropertiesData == NULL) {
		fprintf(stderr, "FOPR data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold FOPR data, generate an error
	if (bufferLength < sizeof (FOPRType)) {
		fprintf(stderr, "Buffer too small to hold necessary FOPR data\n");
		return -1;
	}

	// Construct header
	FOPRData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_FOPR, inputHeader, sizeof (FOPRData), debug);
	memcpy(p, &FOPRData.header, sizeof (FOPRData.header));
	p += sizeof (FOPRData.header);
	remainingBytes -= sizeof (FOPRData.header);

	if (debug) {
		printf("FOPR message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID, &foreignObjectPropertiesData->foreignTransmitterID, &p,
							sizeof (FOPRData.foreignTransmitterID), &remainingBytes, &FOPRForeignTransmitterIDDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_TYPE, &foreignObjectPropertiesData->objectType, &p,
							sizeof (FOPRData.objectType), &remainingBytes, &FOPRObjectTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_ACTOR_TYPE, &foreignObjectPropertiesData->actorType, &p,
							sizeof (FOPRData.actorType), &remainingBytes, &FOPRActorTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_OPERATION_MODE, &foreignObjectPropertiesData->operationMode, &p,
							sizeof (FOPRData.operationMode), &remainingBytes, &FOPROperationModeDescription, debug);

	if (foreignObjectPropertiesData->isMassValid) FOPRData.mass = (uint32_t)(foreignObjectPropertiesData->mass_kg * MASS_ONE_KILOGRAM_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_MASS, &FOPRData.mass, &p,
								sizeof (FOPRData.mass), &remainingBytes, &FOPRMassDescription, debug);

	if (foreignObjectPropertiesData->isObjectXDimensionValid) FOPRData.objectLengthX = (uint32_t)(foreignObjectPropertiesData->objectXDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_X, &FOPRData.objectLengthX, &p,
								sizeof (FOPRData.objectLengthX), &remainingBytes, &FOPRObjectLengthXDescription, debug);
 
	if (foreignObjectPropertiesData->isObjectYDimensionValid) FOPRData.objectLengthY = (uint32_t)(foreignObjectPropertiesData->objectYDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_Y, &FOPRData.objectLengthY, &p,
								sizeof (FOPRData.objectLengthY), &remainingBytes, &FOPRObjectLengthYDescription, debug);

	if (foreignObjectPropertiesData->isObjectZDimensionValid) FOPRData.objectLengthZ = (uint32_t)(foreignObjectPropertiesData->objectZDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_Z, &FOPRData.objectLengthZ, &p,
								sizeof (FOPRData.objectLengthZ), &remainingBytes, &FOPRObjectLengthZDescription, debug);

	if (foreignObjectPropertiesData->isObjectXDisplacementValid) FOPRData.positionDisplacementX = (int16_t)(foreignObjectPropertiesData->positionDisplacementX_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_X, &FOPRData.positionDisplacementX, &p,
								sizeof (FOPRData.positionDisplacementX), &remainingBytes, &FOPRPositionDisplacementXDescription, debug);

	if (foreignObjectPropertiesData->isObjectYDisplacementValid) FOPRData.positionDisplacementY = (int16_t)(foreignObjectPropertiesData->positionDisplacementY_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y, &FOPRData.positionDisplacementY, &p,
								sizeof (FOPRData.positionDisplacementY), &remainingBytes, &FOPRPositionDisplacementYDescription, debug);

	if (foreignObjectPropertiesData->isObjectZDisplacementValid) FOPRData.positionDisplacementZ = (int16_t)(foreignObjectPropertiesData->positionDisplacementZ_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z, &FOPRData.positionDisplacementZ, &p,
								sizeof (FOPRData.positionDisplacementZ), &remainingBytes, &FOPRPositionDisplacementZDescription, debug);

	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}
	
	// Construct footer
	FOPRData.footer = buildISOFooter(foprDataBuffer, (size_t)(p - foprDataBuffer) + sizeof (FooterType), debug);
	memcpy(p, &FOPRData.footer, sizeof (FOPRData.footer));
	p += sizeof (FOPRData.footer);
	remainingBytes -= sizeof (FOPRData.footer);
	if(debug)
	{
		printf("FOPR message data (size = %zu):\n", sizeof (FOPRType));
		for(int i = 0; i < sizeof (FOPRType); i++) printf("%x ", *(foprDataBuffer+i));
		printf("\n");
	}
	return p - foprDataBuffer;
}

/*!
 * \brief convertOPROToHostRepresentation Converts a OPRO message to SI representation
 * \param OPROData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertOPROToHostRepresentation(const OPROType* OPROData,
		ObjectPropertiesType* objectProperties) {

	if (OPROData == NULL || objectProperties == NULL) {
		errno = EINVAL;
		fprintf(stderr, "OPRO input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	objectProperties->isMassValid = OPROData->massValueID && OPROData->mass != MASS_UNAVAILABLE_VALUE;
	objectProperties->isObjectXDimensionValid = OPROData->objectLengthXValueID && OPROData->objectLengthX != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectYDimensionValid = OPROData->objectLengthYValueID && OPROData->objectLengthY != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectZDimensionValid = OPROData->objectLengthZValueID && OPROData->objectLengthZ != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectXDisplacementValid = OPROData->objectLengthXValueID && OPROData->positionDisplacementX != POSITION_UNAVAILABLE_VALUE;
	objectProperties->isObjectYDisplacementValid = OPROData->objectLengthYValueID && OPROData->positionDisplacementY != POSITION_UNAVAILABLE_VALUE;
	objectProperties->isObjectZDisplacementValid = OPROData->objectLengthZValueID && OPROData->positionDisplacementZ != POSITION_UNAVAILABLE_VALUE;

	objectProperties->objectID = OPROData->header.transmitterID;

	objectProperties->objectType = OPROData->objectTypeValueID ? OPROData->objectType : OBJECT_CATEGORY_UNKNOWN;
	objectProperties->actorType = OPROData->actorTypeValueID ? OPROData->actorType : ACTOR_TYPE_UNKNOWN;
	objectProperties->operationMode = OPROData->operationModeValueID ? OPROData->operationMode : OPERATION_MODE_UNKNOWN;

	objectProperties->mass_kg = objectProperties->isMassValid ? (double)(OPROData->mass) / MASS_ONE_KILOGRAM_VALUE : 0;
	objectProperties->objectXDimension_m = objectProperties->isObjectXDimensionValid ?
				(double)(OPROData->objectLengthX) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->objectYDimension_m = objectProperties->isObjectYDimensionValid ?
				(double)(OPROData->objectLengthY) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->objectZDimension_m = objectProperties->isObjectZDimensionValid ?
				(double)(OPROData->objectLengthZ) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementX_m = objectProperties->isObjectXDisplacementValid ?
				(double)(OPROData->positionDisplacementX) / POSITION_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementY_m = objectProperties->isObjectYDisplacementValid ?
				(double)(OPROData->positionDisplacementY) / POSITION_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementZ_m = objectProperties->isObjectZDisplacementValid ?
				(double)(OPROData->positionDisplacementZ) / POSITION_ONE_METER_VALUE : 0;

	return MESSAGE_OK;
}


/*!
 * \brief convertFOPRToHostRepresentation Converts a FOPR message to SI representation
 * \param FOPRData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertFOPRToHostRepresentation(const FOPRType* FOPRData,
		ForeignObjectPropertiesType* foreignObjectProperties) {

	if (FOPRData == NULL || foreignObjectProperties == NULL) {
		errno = EINVAL;
		fprintf(stderr, "FOPR input pointer error");
		return ISO_FUNCTION_ERROR;
	}
	
	foreignObjectProperties->isMassValid = FOPRData->massValueID && FOPRData->mass != MASS_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectXDimensionValid = FOPRData->objectLengthXValueID && FOPRData->objectLengthX != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectYDimensionValid = FOPRData->objectLengthYValueID && FOPRData->objectLengthY != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectZDimensionValid = FOPRData->objectLengthZValueID && FOPRData->objectLengthZ != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectXDisplacementValid = FOPRData->objectLengthXValueID && FOPRData->positionDisplacementX != POSITION_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectYDisplacementValid = FOPRData->objectLengthYValueID && FOPRData->positionDisplacementY != POSITION_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectZDisplacementValid = FOPRData->objectLengthZValueID && FOPRData->positionDisplacementZ != POSITION_UNAVAILABLE_VALUE;

	foreignObjectProperties->foreignTransmitterID = FOPRData->header.transmitterID;

	foreignObjectProperties->objectType = FOPRData->objectTypeValueID ? FOPRData->objectType : OBJECT_CATEGORY_UNKNOWN;
	foreignObjectProperties->actorType = FOPRData->actorTypeValueID ? FOPRData->actorType : ACTOR_TYPE_UNKNOWN;
	foreignObjectProperties->operationMode = FOPRData->operationModeValueID ? FOPRData->operationMode : OPERATION_MODE_UNKNOWN;

	foreignObjectProperties->mass_kg = foreignObjectProperties->isMassValid ? (double)(FOPRData->mass) / MASS_ONE_KILOGRAM_VALUE : 0;
	foreignObjectProperties->objectXDimension_m = foreignObjectProperties->isObjectXDimensionValid ?
				(double)(FOPRData->objectLengthX) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->objectYDimension_m = foreignObjectProperties->isObjectYDimensionValid ?
				(double)(FOPRData->objectLengthY) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->objectZDimension_m = foreignObjectProperties->isObjectZDimensionValid ?
				(double)(FOPRData->objectLengthZ) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementX_m = foreignObjectProperties->isObjectXDisplacementValid ?
				(double)(FOPRData->positionDisplacementX) / POSITION_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementY_m = foreignObjectProperties->isObjectYDisplacementValid ?
				(double)(FOPRData->positionDisplacementY) / POSITION_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementZ_m = foreignObjectProperties->isObjectZDisplacementValid ?
				(double)(FOPRData->positionDisplacementZ) / POSITION_ONE_METER_VALUE : 0;

	return MESSAGE_OK;
}

/*!
 * \brief decodeFOPRMessage Decodes a buffer containing FOPR data into an object properties struct
 * \param foreignObjectPropertiesData Struct to be filled
 * \param foprDataBuffer Buffer containing data to be decoded
 * \param bufferLength Size of buffer containing data to be decoded
 * \param debug Parameter for enabling debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t decodeFOPRMessage(
		ForeignObjectPropertiesType * foreignObjectPropertiesData,
		const char *foprDataBuffer,
		const size_t bufferLength,
		const char debug) {
	FOPRType FOPRData;
	const char *p = foprDataBuffer;

	uint16_t valueID;
	uint16_t contentLength;

	if(debug)
	{
		printf("FOPR message data (size = %zu):\n", sizeof (FOPRType));
		for(int i = 0; i < sizeof (FOPRType); i++) printf("%x ", *(foprDataBuffer+i));
		printf("\n");
	}

	if (foreignObjectPropertiesData == NULL || foprDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	ssize_t retval = MESSAGE_OK;

	memset(foreignObjectPropertiesData, 0, sizeof (*foreignObjectPropertiesData));
	memset(&FOPRData, 0, sizeof (FOPRData));

	if ((retval = decodeISOHeader(p, bufferLength, &FOPRData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (FOPRData.header);

	// If message is not a FOPR message, generate an error
	if (FOPRData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_FOPR) {
		fprintf(stderr, "Attempted to pass non-FOPR message into FOPR parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}


	// Decode contents
	while ((size_t) (p - foprDataBuffer) < FOPRData.header.messageLength + sizeof (FOPRData.header)) {
		// Decode value ID and length
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);

		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		// Handle contents
		switch (valueID) {
		case VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID:
			memcpy(&FOPRData.foreignTransmitterIDValueID, &valueID, sizeof (FOPRData.foreignTransmitterIDValueID));
			memcpy(&FOPRData.foreignTransmitterIDContentLength, &contentLength,
				   sizeof (FOPRData.foreignTransmitterIDContentLength));
			memcpy(&FOPRData.foreignTransmitterID, p, sizeof (FOPRData.foreignTransmitterID));
			break;
		case VALUE_ID_FOPR_OBJECT_TYPE:
			memcpy(&FOPRData.objectTypeValueID, &valueID, sizeof (FOPRData.objectTypeValueID));
			memcpy(&FOPRData.objectTypeContentLength, &contentLength,
				   sizeof (FOPRData.objectTypeContentLength));
			memcpy(&FOPRData.objectType, p, sizeof (FOPRData.objectType));
			break;
		case VALUE_ID_FOPR_ACTOR_TYPE:
			memcpy(&FOPRData.actorTypeValueID, &valueID, sizeof (FOPRData.actorTypeValueID));
			memcpy(&FOPRData.actorTypeContentLength, &contentLength,
				   sizeof (FOPRData.actorTypeContentLength));
			memcpy(&FOPRData.actorType, p, sizeof (FOPRData.actorType));
			break;
		case VALUE_ID_FOPR_OPERATION_MODE:
			memcpy(&FOPRData.operationModeValueID, &valueID, sizeof (FOPRData.operationModeValueID));
			memcpy(&FOPRData.operationModeContentLength, &contentLength,
				   sizeof (FOPRData.operationModeContentLength));
			memcpy(&FOPRData.operationMode, p, sizeof (FOPRData.operationMode));
			break;
		case VALUE_ID_FOPR_MASS:
			memcpy(&FOPRData.massValueID, &valueID, sizeof (FOPRData.massValueID));
			memcpy(&FOPRData.massContentLength, &contentLength, sizeof (FOPRData.massContentLength));
			memcpy(&FOPRData.mass, p, sizeof (FOPRData.mass));
			FOPRData.mass = le32toh(FOPRData.mass);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_X:
			memcpy(&FOPRData.objectLengthXValueID, &valueID, sizeof (FOPRData.objectLengthXValueID));
			memcpy(&FOPRData.objectLengthXContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthXContentLength));
			memcpy(&FOPRData.objectLengthX, p, sizeof (FOPRData.objectLengthX));
			FOPRData.objectLengthX = le32toh(FOPRData.objectLengthX);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_Y:
			memcpy(&FOPRData.objectLengthYValueID, &valueID, sizeof (FOPRData.objectLengthYValueID));
			memcpy(&FOPRData.objectLengthYContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthYContentLength));
			memcpy(&FOPRData.objectLengthY, p, sizeof (FOPRData.objectLengthY));
			FOPRData.objectLengthY = le32toh(FOPRData.objectLengthY);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_Z:
			memcpy(&FOPRData.objectLengthZValueID, &valueID, sizeof (FOPRData.objectLengthZValueID));
			memcpy(&FOPRData.objectLengthZContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthZContentLength));
			memcpy(&FOPRData.objectLengthZ, p, sizeof (FOPRData.objectLengthZ));
			FOPRData.objectLengthZ = le32toh(FOPRData.objectLengthZ);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_X:
			memcpy(&FOPRData.positionDisplacementXValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementXValueID));
			memcpy(&FOPRData.positionDisplacementXContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementXContentLength));
			memcpy(&FOPRData.positionDisplacementX, p, sizeof (FOPRData.positionDisplacementX));
			FOPRData.positionDisplacementX = le16toh(FOPRData.positionDisplacementX);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y:
			memcpy(&FOPRData.positionDisplacementYValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementYValueID));
			memcpy(&FOPRData.positionDisplacementYContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementYContentLength));
			memcpy(&FOPRData.positionDisplacementY, p, sizeof (FOPRData.positionDisplacementY));
			FOPRData.positionDisplacementY = le16toh(FOPRData.positionDisplacementY);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z:
			memcpy(&FOPRData.positionDisplacementZValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementZValueID));
			memcpy(&FOPRData.positionDisplacementZContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementZContentLength));
			memcpy(&FOPRData.positionDisplacementZ, p, sizeof (FOPRData.positionDisplacementZ));
			FOPRData.positionDisplacementZ = le16toh(FOPRData.positionDisplacementZ);
			break;

		default:
			printf("Unable to handle FOPR value ID 0x%x\n", valueID);
			break;
		}
		p += contentLength;
	}


	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - foprDataBuffer), &FOPRData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding FOPR footer\n");
		return retval;
	}

	if ((retval = verifyChecksum(foprDataBuffer, FOPRData.header.messageLength + sizeof (FOPRData.header),
								 FOPRData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "FOPR checksum error\n");
		return retval;
	}

	if (debug) {
		printf("FOPR message:\n");
		printf("\tForeign transmitter value ID: 0x%x\n", FOPRData.foreignTransmitterIDValueID);
		printf("\tForeign transmitter ID content length: %u\n", FOPRData.foreignTransmitterIDContentLength);
		printf("\tForeign transmitter ID: %u\n", FOPRData.foreignTransmitterID);

		printf("\tObject type value ID: 0x%x\n", FOPRData.objectTypeValueID);
		printf("\tObject type content length: %u\n", FOPRData.objectTypeContentLength);
		printf("\tObject type: %u\n", FOPRData.objectType);
		printf("\tActor type value ID: 0x%x\n", FOPRData.actorTypeValueID);
		printf("\tActor type content length: %u\n", FOPRData.actorTypeContentLength);
		printf("\tActor type: %u\n", FOPRData.actorType);
		printf("\tOperation mode value ID: 0x%x\n", FOPRData.operationModeValueID);
		printf("\tOperation mode content length: %u\n", FOPRData.operationModeContentLength);
		printf("\tOperation mode: %u\n", FOPRData.operationMode);
		printf("\tObject length X value ID: 0x%x\n", FOPRData.objectLengthXValueID);
		printf("\tObject length X content length: %u\n", FOPRData.objectLengthXContentLength);
		printf("\tObject length X: %u [mm]\n", FOPRData.objectLengthX);
		printf("\tObject length Y value ID: 0x%x\n", FOPRData.objectLengthYValueID);
		printf("\tObject length Y content length: %u\n", FOPRData.objectLengthYContentLength);
		printf("\tObject length Y: %u [mm]\n", FOPRData.objectLengthY);
		printf("\tObject length Z value ID: 0x%x\n", FOPRData.objectLengthZValueID);
		printf("\tObject length Z content length: %u\n", FOPRData.objectLengthZContentLength);
		printf("\tObject length Z: %u [mm]\n", FOPRData.objectLengthZ);
		printf("\tPosition displacement X value ID: 0x%x\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement X content length: %u\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement X: %d [mm]\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement Y value ID: 0x%x\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Y content length: %u\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Y: %d [mm]\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Z value ID: 0x%x\n", FOPRData.positionDisplacementZ);
		printf("\tPosition displacement Z content length: %u\n", FOPRData.positionDisplacementZ);
		printf("\tPosition displacement Z: %d [mm]\n", FOPRData.positionDisplacementZ);
	}

	// Fill output struct with parsed data
	retval = convertFOPRToHostRepresentation(&FOPRData, foreignObjectPropertiesData);
	return retval < 0 ? retval : p - foprDataBuffer;
}

/*!
 * \brief encodeGDRMMessage Constructs an ISO GDRM message (General Data Request Message)
 * \param inputHeader data to create header with
 * \param gdrmData Struct containing relevant GDRM data
 * \param gdrmDataBuffer Data buffer in which to place encoded GDRM message
 * \param bufferLength Size of data buffer in which to place encoded GDRM message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeGDRMMessage(const MessageHeaderType *inputHeader, const GdrmMessageDataType *gdrmData, char *gdrmDataBuffer, const size_t bufferLength,
						  const char debug) {

	 GDRMType GDRMData;

	 memset(gdrmDataBuffer, 0, bufferLength);
	 char* p = gdrmDataBuffer;
	 size_t remainingBytes = bufferLength;
	 int retval = 0;

	 if (gdrmData == NULL) {
		 fprintf(stderr, "GDRM data input pointer error\n");
		 return -1;
	 }

	 // If buffer too small to hold GDRM data, generate an error
	 if (bufferLength < sizeof (GDRMType)) {
		 fprintf(stderr, "Buffer too small to hold necessary GDRM data\n");
		 return -1;
	 }

	 // Construct header
	 GDRMData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GDRM, inputHeader, sizeof (GDRMData), debug);
	 memcpy(p, &GDRMData.header, sizeof (GDRMData.header));
	 p += sizeof (GDRMData.header);
	 remainingBytes -= sizeof (GDRMData.header);

	 if (debug) {
			 printf("GDRM message:\n");
	 }
	 // Fill contents
	 retval |= encodeContent(VALUE_ID_GDRM_DATA_CODE, &gdrmData->dataCode, &p,
						   sizeof (gdrmData->dataCode), &remainingBytes, &GDRMDataCodeDescription, debug);


	 if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		 fprintf(stderr, "Buffer too small to hold necessary GDRM data\n");
		 return -1;
	 }

	 if (debug) {
		 printf("GDRM message:\n\t<<debug printout not implemented>>\n");
	 }

	 // Construct footer
	 GDRMData.footer = buildISOFooter(&GDRMData, sizeof (GDRMData), debug);
	 memcpy(p, &GDRMData.footer, sizeof (GDRMData.footer));
	 p += sizeof (GDRMData.footer);
	 remainingBytes -= sizeof (GDRMData.footer);

	 if(debug)
	 {
		 printf("GDRM message data (size = %zu):\n", sizeof (GDRMType));
		 for(int i = 0; i < sizeof (GDRMType); i++) printf("%x ", *(gdrmDataBuffer+i));
		 printf("\n");
	 }

	 return p - gdrmDataBuffer;
 }

 /*!
  * \brief decodeGDRMMessage Fills GDRM data elements from a buffer of raw data
  * \param gdrmDataBuffer Raw data to be decoded
  * \param bufferLength Number of bytes in buffer of raw data to be decoded
  * \param gdrmData Struct to be filled
  * \param debug Flag for enabling of debugging
  * \return value according to ::ISOMessageReturnValue
  */
 enum ISOMessageReturnValue decodeGDRMMessage(
										 const char *gdrmDataBuffer,
										 const size_t bufferLength,
										 GdrmMessageDataType* gdrmData,
										 const char debug) {

	 GDRMType GDRMData;
	 const char *p = gdrmDataBuffer;
	 enum ISOMessageReturnValue retval = MESSAGE_OK;
	 uint16_t valueID = 0;
	 uint16_t contentLength = 0;
	 ssize_t expectedContentLength = 0;

	 if (gdrmDataBuffer == NULL || gdrmData == NULL) {
		 errno = EINVAL;
		 fprintf(stderr, "Input pointers to GDRM parsing function cannot be null\n");
		 return ISO_FUNCTION_ERROR;
	 }

	 memset(&GDRMData, 0, sizeof (GDRMData));
	 memset(gdrmData, 0, sizeof (*gdrmData));

	 // Decode ISO header
	 if ((retval = decodeISOHeader(p, bufferLength, &GDRMData.header, debug)) != MESSAGE_OK) {
		 return retval;
	 }
	 p += sizeof (GDRMData.header);

	 // If message is not a GDRM message, generate an error
	 if (GDRMData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GDRM) {
		 fprintf(stderr, "Attempted to pass non-GDRM message into GDRM parsing function\n");
		 return MESSAGE_TYPE_ERROR;
	 }


	 if (GDRMData.header.messageLength > sizeof (GDRMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		 fprintf(stderr, "GDRM message exceeds expected message length\n");
		 return MESSAGE_LENGTH_ERROR;
	 }

	 //If data is added to the message add the retreiving code here

	 // Decode footer
	 if ((retval =
		  decodeISOFooter(p, bufferLength - (size_t) (p - gdrmDataBuffer), &GDRMData.footer,
						  debug)) != MESSAGE_OK) {
		 fprintf(stderr, "Error decoding GDRM footer\n");
		 return retval;
	 }


	 if (debug) {
		 printf("GDRM message:\n");
		 printf("\tMessage id: 0x%x\n", GDRMData.header.messageID);
	 }

	 retval = convertGDRMToHostRepresentation(&GDRMData, gdrmData);

	 return retval;
 }



 /*!
  * \brief convertGDRMToHostRepresentation Converts a GDRM message to be used by host
  * \param GDRMData Data struct containing ISO formatted data
  * \param gdrmData Output data struct, to be used by host
  * \return Value according to ::ISOMessageReturnValue
  */
 enum ISOMessageReturnValue convertGDRMToHostRepresentation(GDRMType* GDRMData,
		 GdrmMessageDataType* gdrmData) {


	 if (GDRMData == NULL || gdrmData == NULL) {
		 errno = EINVAL;
		 fprintf(stderr, "GDRM input pointer error");
		 return ISO_FUNCTION_ERROR;
	 }

	 gdrmData->dataCode = GDRMData->DataCode;


	 return MESSAGE_OK;
 }



/*!
 * \brief encodeDCTIMessage Constructs an ISO DCTI message (Direct Control Transmitter Id)
 * \param inputHeader data to create header with

 * \param dctiData Struct containing relevant DCTI data
 * \param dctiDataBuffer Data buffer in which to place encoded DCTI message
 * \param bufferLength Size of data buffer in which to place encoded DCTI message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeDCTIMessage(const MessageHeaderType *inputHeader, const DctiMessageDataType *dctiData,
						char *dctiDataBuffer, const size_t bufferLength, const char debug) {

	DCTIType DCTIData;

	memset(dctiDataBuffer, 0, bufferLength);
	char* p = dctiDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (dctiData == NULL) {
		fprintf(stderr, "DCTI data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold DCTI data, generate an error
	if (bufferLength < sizeof (DCTIType)) {
		fprintf(stderr, "Buffer too small to hold necessary DCTI data\n");
		return -1;
	}

	// Construct header
	DCTIData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCTI, inputHeader, sizeof (DCTIData), debug);
	memcpy(p, &DCTIData.header, sizeof (DCTIData.header));
	p += sizeof (DCTIData.header);
	remainingBytes -= sizeof (DCTIData.header);

	if (debug) {
			printf("DCTI message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_DCTI_TOTAL_COUNT, &dctiData->totalCount, &p,
						  sizeof (dctiData->totalCount), &remainingBytes, &DCTITotalCountDescription, debug);
	retval |= encodeContent(VALUE_ID_DCTI_COUNTER, &dctiData->counter, &p,
						  sizeof (dctiData->counter), &remainingBytes, &DCTICounterDescription, debug);
	retval |= encodeContent(VALUE_ID_DCTI_TRANSMITTER_ID, &dctiData->transmitterID, &p,
						  sizeof (dctiData->transmitterID), &remainingBytes, &DCTITransmitterIdDescription, debug);


	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary DCTI data\n");
		return -1;
	}

	if (debug) {
		printf("DCTI message:\n\t<<debug printout not implemented>>\n");
	}

	// Construct footer
	DCTIData.footer = buildISOFooter(&DCTIData, sizeof (DCTIData), debug);
	memcpy(p, &DCTIData.footer, sizeof (DCTIData.footer));
	p += sizeof (DCTIData.footer);
	remainingBytes -= sizeof (DCTIData.footer);

	if(debug)
	{
		printf("DCTI message data (size = %zu):\n", sizeof (DCTIType));
		for(int i = 0; i < sizeof (DCTIType); i++) printf("%x ", *(dctiDataBuffer+i));
		printf("\n");
	}

	return p - dctiDataBuffer;


}

/*!
 * \brief decodeDCTIMessage Fills HEAB data elements from a buffer of raw data
 * \param dctiDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param dctiData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue decodeDCTIMessage(
										const char *dctiDataBuffer,
										const size_t bufferLength,
										DctiMessageDataType* dctiData,
										const char debug) {
	DCTIType DCTIData;
	const char *p = dctiDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (dctiDataBuffer == NULL || dctiData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to DCTI parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&DCTIData, 0, sizeof (DCTIData));
	memset(dctiData, 0, sizeof (*dctiData));
	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DCTIData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (DCTIData.header);

	// If message is not a PODI message, generate an error
	if (DCTIData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCTI) {
		fprintf(stderr, "Attempted to pass non-DCTI message into DCTI parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (DCTIData.header.messageLength > sizeof (DCTIType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "PODI message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - dctiDataBuffer < DCTIData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_DCTI_TOTAL_COUNT:
			memcpy(&DCTIData.TotalCount, p, sizeof (DCTIData.TotalCount));
			DCTIData.TotalCountValueIdU16 = valueID;
			DCTIData.TotalCountContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.TotalCount);
			break;
		case VALUE_ID_DCTI_COUNTER:
			memcpy(&DCTIData.Counter, p, sizeof (DCTIData.Counter));
			DCTIData.CounterValueIdU16 = valueID;
			DCTIData.CounterContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.Counter);
			break;
		case VALUE_ID_DCTI_TRANSMITTER_ID:
			memcpy(&DCTIData.TransmitterID, p, sizeof (DCTIData.TransmitterID));
			DCTIData.TransmitterIDValueIdU16 = valueID;
			DCTIData.TransmitterIDContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.TransmitterID);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known DCTI value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - dctiDataBuffer), &DCTIData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding DCTI footer\n");
		return retval;
	}
	p += sizeof (DCTIData.footer);

	/*if ((retval = verifyChecksum(dctiDataBuffer, DCTIData.header.messageLength + sizeof (HeaderType),
								 DCTIData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "DCTI checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("DCTI message:\n");
		printf("\tTotal count value ID: 0x%x\n", DCTIData.TotalCountValueIdU16);
		printf("\tTotal count content length: %u\n", DCTIData.TotalCountContentLengthU16);
		printf("\tTotal count: %u\n", DCTIData.TotalCount);
		printf("\tCounter value ID: 0x%x\n", DCTIData.CounterValueIdU16);
		printf("\tCounter content length: %u\n", DCTIData.CounterContentLengthU16);
		printf("\tCounter: %u\n", DCTIData.Counter);
		printf("\tTransmitter id value ID: 0x%x\n", DCTIData.TransmitterIDValueIdU16);
		printf("\tTransmitter id content length: %u\n", DCTIData.TransmitterIDContentLengthU16);
		printf("\tTransmitter id: %u\n", DCTIData.TransmitterID);
	}

	retval = convertDCTIToHostRepresentation(&DCTIData, dctiData);

	return retval < 0 ? retval : p - dctiDataBuffer;
}


/*!
 * \brief convertDCTIToHostRepresentation Converts a DCTI message for client usage
 * \param DCTIData Data struct containing ISO formatted data
 * \param dctiData Output data struct
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertDCTIToHostRepresentation(DCTIType* DCTIData,
		DctiMessageDataType* dctiData) {

	if (DCTIData == NULL || dctiData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "DCTI input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	dctiData->totalCount = DCTIData->TotalCount;
	dctiData->counter = DCTIData->Counter;
	dctiData->transmitterID = DCTIData->TransmitterID;


	return MESSAGE_OK;
}

/*!
 * \brief encodeRDCAMessage Constructs an ISO RDCA message (Request Direct Control Action)
 * \param inputHeader data to create header with
 * \param rdcaData Requested action data
 * \param rdcaDataBuffer Data buffer in which to place encoded RDCA message
 * \param bufferLength Size of data buffer in which to place encoded RDCA message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeRDCAMessage(const MessageHeaderType *inputHeader,
						  const RequestControlActionType *rdcaData,
						  char *rdcaDataBuffer,
						  const size_t bufferLength, 
						  const char debug) {
	RDCAType RDCAData;

	memset(rdcaDataBuffer, 0, bufferLength);
	char* p = rdcaDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (rdcaData == NULL || rdcaDataBuffer == NULL) {
		fprintf(stderr, "RDCA data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold RDCA data, generate an error
	if (bufferLength < sizeof (RDCAType)) {
		fprintf(stderr, "Buffer too small to hold necessary RDCA data\n");
		return -1;
	}
	
	
	size_t unusedMemory = 0;
	if (!rdcaData->isSteeringActionValid) {
		unusedMemory += sizeof (RDCAData.steeringActionValueID)
				+ sizeof (RDCAData.steeringActionContentLength)
				+ sizeof (RDCAData.steeringAction);
	}
	if (!rdcaData->isSpeedActionValid) {
			unusedMemory += sizeof (RDCAData.speedActionValueID)
				+ sizeof (RDCAData.speedActionContentLength)	
				+ sizeof (RDCAData.speedAction);
	}
	// Construct header
	RDCAData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA, inputHeader, sizeof (RDCAData) - unusedMemory, debug);
	memcpy(p, &RDCAData.header, sizeof (RDCAData.header));
	p += sizeof (RDCAData.header);
	remainingBytes -= sizeof (RDCAData.header);

	if (debug) {
		printf("RDCA message:\n");
	}
	// Fill contents
	RDCAData.intendedReceiverID = rdcaData->executingID;
	retval |= encodeContent(VALUE_ID_RDCA_INTENDED_RECEIVER, &RDCAData.intendedReceiverID, &p,
							sizeof (RDCAData.intendedReceiverID), &remainingBytes, &RDCAIntendedReceiverDescription, debug);
							
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&rdcaData->dataTimestamp);
	RDCAData.gpsQmsOfWeek = GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_RDCA_GPS_QMS_OF_WEEK, &RDCAData.gpsQmsOfWeek, &p,
						  sizeof (RDCAData.gpsQmsOfWeek), &remainingBytes, &RDCAGpsQmsOfWeekDescription, debug);
	
	
	if (rdcaData->isSteeringActionValid && rdcaData->steeringUnit == ISO_UNIT_TYPE_STEERING_DEGREES) {	
		if(rdcaData->steeringAction.rad <= STEERING_ANGLE_MAX_VALUE_RAD && rdcaData->steeringAction.rad >= STEERING_ANGLE_MIN_VALUE_RAD) { 
			RDCAData.steeringAction = (int16_t) (rdcaData->steeringAction.rad * (180.0 / M_PI) * STEERING_ANGLE_ONE_DEGREE_VALUE);
			retval |= encodeContent(VALUE_ID_RDCA_STEERING_ANGLE, &RDCAData.steeringAction, &p,
								sizeof (RDCAData.steeringAction), &remainingBytes, &RDCASteeringDescriptionDeg, debug);	

		}
		else {
			fprintf(stderr, "Steering value is out of bounds for angle value\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else if (rdcaData->isSteeringActionValid && rdcaData->steeringUnit == ISO_UNIT_TYPE_STEERING_PERCENTAGE) {
		if(rdcaData->steeringAction.pct <= MAX_VALUE_PERCENTAGE && rdcaData->steeringAction.pct >= MIN_VALUE_PERCENTAGE) {
			RDCAData.steeringAction = (int16_t) (rdcaData->steeringAction.pct);
			retval |= encodeContent(VALUE_ID_RDCA_STEERING_PERCENTAGE, &RDCAData.steeringAction, &p,
								sizeof(RDCAData.steeringAction), &remainingBytes, &RDCASteeringDescriptionPct, debug);	
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}

	if(rdcaData->isSpeedActionValid && rdcaData->speedUnit == ISO_UNIT_TYPE_SPEED_METER_SECOND) {
		RDCAData.speedAction = (int16_t) (rdcaData->speedAction.m_s *SPEED_ONE_METER_PER_SECOND_VALUE);
		retval |= encodeContent(VALUE_ID_RDCA_SPEED_METER_PER_SECOND, &RDCAData.speedAction, &p,
								sizeof(RDCAData.speedAction), &remainingBytes, &RDCASpeedDescription_m_s, debug);
	}
	// TODO: Fix so error state is in else and approv in if
	else if(rdcaData->isSpeedActionValid && rdcaData->speedUnit == ISO_UNIT_TYPE_SPEED_PERCENTAGE) {
		if (rdcaData->speedAction.pct <= MAX_VALUE_PERCENTAGE && rdcaData->speedAction.pct >= MIN_VALUE_PERCENTAGE) {

			RDCAData.speedAction = (int16_t) (rdcaData->speedAction.pct);

		retval |= encodeContent(VALUE_ID_RDCA_SPEED_PERCENTAGE, &RDCAData.speedAction, &p,
								sizeof(RDCAData.speedAction), &remainingBytes, &RDCASpeedDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Speed value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary RDCA data\n");
		return -1;
	}

	// Construct footer
	RDCAData.footer = buildISOFooter(&RDCAData, (size_t) (p-rdcaDataBuffer) + sizeof(RDCAData.footer), debug);
	memcpy(p, &RDCAData.footer, sizeof (RDCAData.footer));
	p += sizeof (RDCAData.footer);
	remainingBytes -= sizeof (RDCAData.footer);

	if(debug) {
		printf("RDCA message data (size = %zu):\n", sizeof (RDCAType));
		for(size_t i = 0; i < sizeof (RDCAType); i++) printf("%x ", *(rdcaDataBuffer+i));
		printf("\n");
	}

	return p - rdcaDataBuffer;
}

/*!
 * \brief decodeRDCAMessage Fills RDCA data elements from a buffer of raw data
 * \param rdcaDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of PODI message
 * \param peerData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeRDCAMessage(
		const char *rdcaDataBuffer,
		RequestControlActionType *rdcaData,
		const size_t bufferLength,
		const struct timeval currentTime,
		const char debug) {

	RDCAType RDCAData;
	const char *p = rdcaDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (rdcaDataBuffer == NULL || rdcaData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to RDCA parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&RDCAData, 0, sizeof (RDCAData));
	memset(rdcaData, 0, sizeof (*rdcaData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &RDCAData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (RDCAData.header);

	// If message is not a RDCA message, generate an error
	if (RDCAData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA) {
		fprintf(stderr, "Attempted to pass non-RDCA message into RDA parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (RDCAData.header.messageLength > sizeof (RDCAType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "RDCA message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - rdcaDataBuffer < RDCAData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_RDCA_GPS_QMS_OF_WEEK:
			memcpy(&RDCAData.gpsQmsOfWeek, p, sizeof (RDCAData.gpsQmsOfWeek));
			RDCAData.gpsQmsOfWeek = le32toh(RDCAData.gpsQmsOfWeek);
			RDCAData.gpsQmsOfWeekValueID = valueID;
			RDCAData.gpsQmsOfWeekContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.gpsQmsOfWeek);
			break;
		case VALUE_ID_RDCA_STEERING_ANGLE:
			memcpy(&RDCAData.steeringAction, p, sizeof (RDCAData.steeringAction));
			RDCAData.steeringAction = (int16_t)le16toh(RDCAData.steeringAction);
			RDCAData.steeringActionValueID = valueID;
			RDCAData.steeringActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.steeringAction);
			break;
		case VALUE_ID_RDCA_INTENDED_RECEIVER:
			memcpy(&RDCAData.intendedReceiverID, p, sizeof (RDCAData.intendedReceiverID));
			RDCAData.intendedReceiverID = le32toh(RDCAData.intendedReceiverID);
			RDCAData.intendedReceiverIDValueID = valueID;
			RDCAData.intendedReceiverIDContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.intendedReceiverID);
			break;
		case VALUE_ID_RDCA_STEERING_PERCENTAGE:
			memcpy(&RDCAData.steeringAction, p, sizeof (RDCAData.steeringAction));
			RDCAData.steeringAction = (int16_t)le16toh(RDCAData.steeringAction);
			RDCAData.steeringActionValueID = valueID;
			RDCAData.steeringActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.steeringAction);
			break;
		case VALUE_ID_RDCA_SPEED_METER_PER_SECOND:
			memcpy(&RDCAData.speedAction, p, sizeof (RDCAData.speedAction));
			RDCAData.speedAction = (int16_t)le16toh(RDCAData.speedAction);
			RDCAData.speedActionValueID = valueID;
			RDCAData.speedActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.speedAction);
			break;
		case VALUE_ID_RDCA_SPEED_PERCENTAGE:
			memcpy(&RDCAData.speedAction, p, sizeof (RDCAData.speedAction));
			RDCAData.speedAction = (int16_t)le16toh(RDCAData.speedAction);
			RDCAData.speedActionValueID = valueID;
			RDCAData.speedActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.speedAction);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known RDCA value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - rdcaDataBuffer), &RDCAData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding RDCA footer\n");
		return retval;
	}
	p += sizeof (RDCAData.footer);

	/*if ((retval = verifyChecksum(rdcaDataBuffer, RDCAData.header.messageLength + sizeof (HeaderType),
								 RDCAData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "RDCA checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("RDCA message:\n");
		printContent(RDCAData.gpsQmsOfWeekValueID, RDCAData.gpsQmsOfWeekContentLength,
					&RDCAData.gpsQmsOfWeek, &RDCAGpsQmsOfWeekDescription);
		if (RDCAData.steeringActionValueID == VALUE_ID_RDCA_STEERING_ANGLE)
			printContent(RDCAData.steeringActionValueID, RDCAData.steeringActionContentLength,
						&RDCAData.steeringAction, &RDCASteeringDescriptionDeg);
		else if (RDCAData.steeringActionValueID == VALUE_ID_RDCA_STEERING_PERCENTAGE)
			printContent(RDCAData.steeringActionValueID, RDCAData.steeringActionContentLength,
						&RDCAData.steeringAction, &RDCASteeringDescriptionPct);
		if (RDCAData.speedActionValueID == VALUE_ID_RDCA_SPEED_METER_PER_SECOND)
			printContent(RDCAData.speedActionValueID, RDCAData.speedActionContentLength,
						&RDCAData.speedAction, &RDCASpeedDescription_m_s);
		else if (RDCAData.speedActionValueID == VALUE_ID_RDCA_SPEED_PERCENTAGE)
			printContent(RDCAData.speedActionValueID, RDCAData.speedActionContentLength,
						&RDCAData.speedAction, &RDCASpeedDescriptionPct);
	}

	retval = convertRDCAToHostRepresentation(&RDCAData, &currentTime, rdcaData);

	return retval < 0 ? retval : p - rdcaDataBuffer;
}



/*!
 * \brief convertRDCAToHostRepresentation Converts a RDCA message to SI representation
 * \param RDCAData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param rdcaData Output data struct
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertRDCAToHostRepresentation(RDCAType* RDCAData,
			const struct timeval* currentTime, RequestControlActionType* rdcaData) {

	if (RDCAData == NULL ||  rdcaData == NULL || currentTime == NULL) {
		errno = EINVAL;
		fprintf(stderr, "RDCA input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	if (!RDCAData->gpsQmsOfWeekValueID
			|| RDCAData->gpsQmsOfWeek == GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE) {
		fprintf(stderr, "Timestamp not supplied in RDCA message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	if (!RDCAData->gpsQmsOfWeekValueID) {
		fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	
	
	setToGPStime(&rdcaData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), RDCAData->gpsQmsOfWeek);
	// Fill steering values and check out of range values
	rdcaData->executingID = RDCAData->intendedReceiverID;
	if (RDCAData->steeringAction != STEERING_ANGLE_UNAVAILABLE_VALUE && RDCAData->steeringActionValueID) {
		if(RDCAData->steeringActionValueID == VALUE_ID_RDCA_STEERING_ANGLE) {
			if (RDCAData->steeringAction <= STEERING_ANGLE_MAX_VALUE_DEG
			&& RDCAData->steeringAction >= STEERING_ANGLE_MIN_VALUE_DEG) {
				rdcaData->isSteeringActionValid = 1;
				rdcaData->steeringAction.rad = RDCAData->steeringAction / STEERING_ANGLE_ONE_DEGREE_VALUE * M_PI / 180.0;
				rdcaData->steeringUnit = ISO_UNIT_TYPE_STEERING_DEGREES; 
			}
			else {
				fprintf(stderr, "Steering angle value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else if(RDCAData->steeringActionValueID == VALUE_ID_RDCA_STEERING_PERCENTAGE) {
			if (RDCAData->steeringAction <= MAX_VALUE_PERCENTAGE
			&& RDCAData->steeringAction >= MIN_VALUE_PERCENTAGE) {
				rdcaData->isSteeringActionValid = 1;
				rdcaData->steeringAction.pct = RDCAData->steeringAction;
				rdcaData->steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Steering percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else {
			fprintf(stderr, "Steering Value ID error\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}

	// Fill speed values and check out of range values
	if (RDCAData->speedAction != SPEED_UNAVAILABLE_VALUE && RDCAData->speedActionValueID) {
		if(RDCAData->speedActionValueID == VALUE_ID_RDCA_SPEED_METER_PER_SECOND) {
				rdcaData->isSpeedActionValid = 1;
				rdcaData->speedAction.m_s = RDCAData->speedAction /SPEED_ONE_METER_PER_SECOND_VALUE;
				rdcaData->speedUnit = ISO_UNIT_TYPE_SPEED_METER_SECOND; 
			}
		else if(RDCAData->speedActionValueID == VALUE_ID_RDCA_SPEED_PERCENTAGE) {
			if (RDCAData->speedAction <= MAX_VALUE_PERCENTAGE 
				&& RDCAData->speedAction >= MIN_VALUE_PERCENTAGE) {
				rdcaData->isSpeedActionValid = 1;
				rdcaData->speedAction.pct = RDCAData->speedAction;
				rdcaData->speedUnit = ISO_UNIT_TYPE_SPEED_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Speed percentage value is out of bounds");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
			
		}
		else {
			fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}
	
	return MESSAGE_OK;
}


/*!
 * \brief mapISOHeadingToHostHeading Converts between ISO NED heading to internal heading measured from the test x axis
 * \param isoHeading_rad Heading measured according to ISO specification, in radians
 * \return Heading, in radians, measured from the test x axis
 */
double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = isoHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}

/*!
 * \brief mapHostHeadingToISOHeading Converts between internal heading measured from the test x axis to ISO NED heading
 * \param isoHeading_rad Heading measured form test x axis, in radians
 * \return Heading, in radians, measured as specified by ISO 22133
 */
double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = hostHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}


/*!
 * \brief encodeDCMMessage Fills an ISO vendor specific (AstaZero) DCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param inputHeader data to create header with
 * \param command Struct containing relevant DCMM data
 * \param dcmmDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeDCMMMessage(const MessageHeaderType *inputHeader,
		const RemoteControlManoeuvreMessageType* command,
		char* dcmmDataBuffer,
		const size_t bufferLength,
		const char debug) {

	HeaderType DCMMHeader;
	FooterType DCMMFooter;
	RCMMType DCMMData;

	DCMMHeader = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM, inputHeader, sizeof (DCMMData), debug);

	ssize_t retval =  encodeRCMMMessage(inputHeader, command, dcmmDataBuffer, bufferLength, debug);
	if (retval < 0) {
		fprintf(stderr, "DCMM wrapper error\n");
		return retval;
	}
	memcpy(dcmmDataBuffer, &DCMMHeader, sizeof(DCMMHeader) );

	DCMMFooter = buildISOFooter(dcmmDataBuffer, retval, debug);
	memcpy(dcmmDataBuffer + retval - sizeof(DCMMFooter), &DCMMFooter, sizeof(DCMMFooter));

	return retval;
}

/**
 * @brief decodeDCMMessage Fills an ISO vendor specific (AstaZero) DCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * @param dcmmDataBuffer Data buffer to which message is to be printed
 * @param bufferLength Available memory in data buffer
 * @param dcmmData Struct containing relevant DCMM data
 * @param debug Flag for enabling debugging
 * @return Number of bytes written to buffer, or -1 in case of error 
 */
ssize_t decodeDCMMMessage(
		const char * dcmmDataBuffer,
		const size_t bufferLength,
		RemoteControlManoeuvreMessageType* dcmmData,
		const char debug) {

	RCMMType DCMMData;
	const char *p = dcmmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;
	
	if (dcmmDataBuffer == NULL || dcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to DCMM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&DCMMData, 0, sizeof (DCMMData));
	memset(dcmmData, 0, sizeof(*dcmmData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DCMMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (DCMMData.header);


	// If message is not a RCMM message, generate an error
	if (DCMMData.header.messageID != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM) {
		fprintf(stderr, "Attempted to pass non-DCMM message into DCMM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (DCMMData.header.messageLength > sizeof (RCMMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "DCMM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - dcmmDataBuffer < DCMMData.header.messageLength + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);
		switch (valueID) {
		case VALUE_ID_RCMM_CONTROL_STATUS:
			memcpy(&DCMMData.controlStatus, p, sizeof (DCMMData.controlStatus));
			DCMMData.controlStatusValueID = valueID;
			DCMMData.controlStatusContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.controlStatus);
			break;
		case VALUE_ID_RCMM_SPEED_METER_PER_SECOND:
			memcpy(&DCMMData.speed, p, sizeof (DCMMData.speed));
			DCMMData.speed = (int16_t)le16toh (DCMMData.speed);
			DCMMData.speedValueID = valueID;
			DCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.speed);
			break;
		case VALUE_ID_RCMM_STEERING_ANGLE:
			memcpy(&DCMMData.steering, p, sizeof (DCMMData.steering));
			DCMMData.steering = (int16_t)le16toh (DCMMData.steering);
			DCMMData.steeringValueID = valueID;
			DCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.steering);
			break;
		case VALUE_ID_RCMM_STEERING_PERCENTAGE:
			memcpy(&DCMMData.steering, p, sizeof (DCMMData.steering));
			DCMMData.steering = (int16_t)le16toh (DCMMData.steering); 
			DCMMData.steeringValueID = valueID;
			DCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.steering);
			break;
		case VALUE_ID_RCMM_SPEED_PERCENTAGE:
			memcpy(&DCMMData.speed, p, sizeof (DCMMData.speed));
			DCMMData.speed = (int16_t)le16toh (DCMMData.speed);
			DCMMData.speedValueID = valueID;
			DCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.speed);
			break;
		case VALUE_ID_RCMM_CONTROL:
			memcpy(&DCMMData.command, p, sizeof (DCMMData.command));
			DCMMData.commandValueID = valueID;
			DCMMData.commandContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.command);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known DCMM value IDs\n", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld\n",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - dcmmDataBuffer), &DCMMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding DCMM footer\n");
		return retval;
	}

	p += sizeof (DCMMData.footer);
	if ((retval = verifyChecksum(dcmmDataBuffer, DCMMData.header.messageLength + sizeof (HeaderType),
								 DCMMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "DCMM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("DCMM message:\n");
		printContent(DCMMData.controlStatusValueID, DCMMData.controlStatusContentLength,
					 &DCMMData.controlStatus, &RCMMControlStatusDescription);
		if (DCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE)
			printContent(DCMMData.steeringValueID, DCMMData.steeringContentLength,
					 	&DCMMData.steering, &RCMMSteeringDescriptionDeg);
		else if (DCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE)
			printContent(DCMMData.steeringValueID, DCMMData.steeringContentLength,
					 	&DCMMData.steering, &RCMMSteeringDescriptionPct);
		if (DCMMData.speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND)
			printContent(DCMMData.speedValueID, DCMMData.speedContentLength,
					 	&DCMMData.speed, &RCMMSpeedDescription_m_s);
		else if (DCMMData.speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE)
			printContent(DCMMData.speedValueID, DCMMData.speedContentLength,
				&DCMMData.speed, &RCMMSpeedDescriptionPct);
	}

	retval = convertRCMMToHostRepresentation(&DCMMData, dcmmData);

	return retval < 0 ? retval : p - dcmmDataBuffer;
	
}