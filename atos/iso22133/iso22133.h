#ifndef ISO22133_H
#define ISO22133_H
#ifdef __cplusplus
extern "C" {
#endif
/*! ------------------------------------------------------------------------------
 *  -- Copyright	: (C) AstaZero AB
 *  ------------------------------------------------------------------------------
 *  -- File			: iso22133.h
 *  -- Author		: Lukas Wikander
 *  -- Description	: This file specifies an interface for converting from network
 *					  messages in the ISO 22133 format to native data types.
 *  -- Purpose		: Reduce the amount of boilerplate needed to read an ISO 22133
 *					  message.
 *  -- Reference	: ISO/TC 22/SC 33/WG 16 - ISO/WD 22133-1
 *  ------------------------------------------------------------------------------
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "positioning.h"

#define ISO_22133_OBJECT_UDP_PORT 53240
#define ISO_22133_DEFAULT_OBJECT_TCP_PORT 53241

typedef enum {
	TEST_MODE_PREPLANNED = 0,
	TEST_MODE_ONLINE = 1,
	TEST_MODE_SCENARIO = 2,
	TEST_MODE_UNAVAILABLE = 255
} TestModeType;

/*! Message Header for building/encoding headers */
typedef struct {
	uint32_t transmitterID;
	uint32_t receiverID;
	uint8_t messageCounter;
} MessageHeaderType;

/*! OSEM settings */
//Extracted to be swig compatible
typedef struct {
	uint32_t transmitter;
	uint32_t subTransmitter;
	uint32_t controlCentre;
} OSEMIds;
typedef struct {
	double_t position_m;
	double_t lateral_m;
	double_t yaw_rad;
} OSEMDeviations;
typedef struct {
	double_t monr;
	double_t monr2;
	double_t heab;
} OSEMMonrRate; 
typedef struct {
		uint32_t ip;
		uint16_t port;
} OSEMTimeServer;
typedef struct {
	OSEMIds desiredID;
	GeographicPositionType coordinateSystemOrigin;
	double coordinateSystemRotation_rad;
	CoordinateSystemType coordinateSystemType;
	struct timeval currentTime;
	OSEMDeviations maxDeviation;
	double_t minRequiredPositioningAccuracy_m;
	TestModeType testMode;
	struct timeval heabTimeout;
	OSEMMonrRate rate;
	OSEMTimeServer timeServer;
} ObjectSettingsType;

/*! ISO message constants */
enum ISOConstantsType {
	ISO_TRAJ_HEADER_SIZE = 91,
	ISO_TRAJ_WAYPOINT_SIZE = 70
};

typedef enum {
	TRAJECTORY_INFO_RELATIVE_TO_OBJECT = 1,
	TRAJECTORY_INFO_RELATIVE_TO_ORIGIN = 2,
	TRAJECTORY_INFO_DELETE_TRAJECTORY = 3
} TrajectoryInfoType;

/*! Trajectory header */
typedef struct {
	uint16_t trajectoryID;
	char trajectoryName[64];
	TrajectoryInfoType trajectoryInfo;
	uint32_t trajectoryLength;
	uint32_t nWaypoints;
} TrajectoryHeaderType;

/*! Trajectory WayPoint */
typedef struct {
	struct timeval relativeTime;
	CartesianPosition pos;
	SpeedType spd;
	AccelerationType acc;
	float_t curvature;
} TrajectoryWaypointType;

/*! OSTM commands */
enum ObjectCommandType {
	OBJECT_COMMAND_ARM = 0x02,				//!< Request to arm the target object
	OBJECT_COMMAND_DISARM = 0x03,			//!< Request to disarm the target object
    OBJECT_COMMAND_REMOTE_CONTROL = 0x06,	//!< Request for remote control of the target object
    OBJECT_COMMAND_ALL_CLEAR = 0x0A         //!< Signal that abort no longer necessary
};


#define HEAB_FREQUENCY_HZ 100
/*! HEAB control center statuses */
enum ControlCenterStatusType {
	CONTROL_CENTER_STATUS_INIT = 0x00,			//!<
	CONTROL_CENTER_STATUS_READY = 0x01,			//!<
	CONTROL_CENTER_STATUS_ABORT = 0x02,			//!<
	CONTROL_CENTER_STATUS_RUNNING = 0x03,		//!<
	CONTROL_CENTER_STATUS_TEST_DONE = 0x04,		//!<
    CONTROL_CENTER_STATUS_NORMAL_STOP = 0x05	//!<
};

#define MONR_EXPECTED_FREQUENCY_HZ 100

/*! GDRM dataCodes */
enum GeneralDataRequestDataCodeType{
	DIRECT_CONTROL_TRANSMITTER_ID_REQUEST = 0x01			//!<
} ;

enum TriggerType_t {
	TRIGGER_UNDEFINED               = 0x0000,
	TRIGGER_TYPE_1                  = 0x0001,
	TRIGGER_SPEED                   = 0x0010,
	TRIGGER_DISTANCE                = 0x0020,
	TRIGGER_ACCELERATION            = 0x0030,
	TRIGGER_LANE_CHANGED            = 0x0040,
	TRIGGER_LANE_OFFSET             = 0x0050,
	TRIGGER_POSITION_REACHED        = 0x0060,
	TRIGGER_POSITION_LEFT           = 0x0061,
	TRIGGER_POSITION_OFFSET         = 0x0062,
	TRIGGER_STEERING_ANGLE          = 0x0070,
	TRIGGER_THROTTLE_VALUE          = 0x0080,
	TRIGGER_BRAKE                   = 0x0090,
	TRIGGER_ACTIVE_TRAJECTORY       = 0x00A0,
	TRIGGER_OTHER_OBJECT_FEATURE    = 0x00B0,
	TRIGGER_INFRASTRUCTURE          = 0x00C0,
	TRIGGER_TEST_SCENARIO_EVENT     = 0x00D0,
	TRIGGER_MISC_DIGITAL_INPUT      = 0x00E0,
	TRIGGER_MISC_ANALOG_INPUT       = 0x00F0,
	TRIGGER_TIMER_EVENT_OCCURRED    = 0x0100,
	TRIGGER_MODE_CHANGED            = 0x0110,
	TRIGGER_UNAVAILABLE             = 0xFFFF
};

enum TriggerTypeParameter_t {
	TRIGGER_PARAMETER_FALSE                     = 0x00000000,
	TRIGGER_PARAMETER_TRUE                      = 0x00000001,
	TRIGGER_PARAMETER_RELEASED                  = 0x00000010,
	TRIGGER_PARAMETER_PRESSED                   = 0x00000011,
	TRIGGER_PARAMETER_LOW                       = 0x00000020,
	TRIGGER_PARAMETER_HIGH                      = 0x00000021,
	TRIGGER_PARAMETER_RISING_EDGE               = 0x00000022,
	TRIGGER_PARAMETER_FALLING_EDGE              = 0x00000023,
	TRIGGER_PARAMETER_ANY_EDGE                  = 0x00000024,
	TRIGGER_PARAMETER_RELATIVE                  = 0x00000030,
	TRIGGER_PARAMETER_ABSOLUTE                  = 0x00000031,
	TRIGGER_PARAMETER_VALUE                     = 0x00000040,
	TRIGGER_PARAMETER_MIN                       = 0x00000050,
	TRIGGER_PARAMETER_MAX                       = 0x00000051,
	TRIGGER_PARAMETER_MEAN                      = 0x00000052,
	TRIGGER_PARAMETER_EQUAL_TO                  = 0x00000060,
	TRIGGER_PARAMETER_GREATER_THAN              = 0x00000061,
	TRIGGER_PARAMETER_GREATER_THAN_OR_EQUAL_TO  = 0x00000062,
	TRIGGER_PARAMETER_LESS_THAN                 = 0x00000063,
	TRIGGER_PARAMETER_LESS_THAN_OR_EQUAL_TO     = 0x00000064,
	TRIGGER_PARAMETER_NOT_EQUAL_TO              = 0x00000065,
	TRIGGER_PARAMETER_X                         = 0x00000070,
	TRIGGER_PARAMETER_Y                         = 0x00000071,
	TRIGGER_PARAMETER_Z                         = 0x00000072,
	TRIGGER_PARAMETER_TIME                      = 0x00000080,
	TRIGGER_PARAMETER_DATE                      = 0x00000081,
	TRIGGER_PARAMETER_RULE                      = 0x000000A0,
	TRIGGER_PARAMETER_UNAVAILABLE               = 0xFFFFFFFF
};


enum ActionType_t {
	ACTION_NONE                     = 0x0000,
	ACTION_TYPE_1                   = 0x0001,
	ACTION_TYPE_2                   = 0x0002,
	ACTION_SET_SPEED                = 0x0010,
	ACTION_SET_DISTANCE             = 0x0020,
	ACTION_SET_ACCELERATION         = 0x0030,
	ACTION_LANE_CHANGE              = 0x0040,
	ACTION_LANE_OFFSET              = 0x0050,
	ACTION_SET_POSITION             = 0x0060,
	ACTION_SET_STEERING_ANGLE       = 0x0070,
	ACTION_SET_TRHOTTLE_VALUE       = 0x0080,
	ACTION_BRAKE                    = 0x0090,
	ACTION_FOLLOW_TRAJECTORY        = 0x00A0,
	ACTION_OTHER_OBJECT_FEATURE     = 0x00B0,
	ACTION_INFRASTRUCTURE           = 0x00C0,
	ACTION_TEST_SCENARIO_COMMAND    = 0x00D0,
	ACTION_MISC_DIGITAL_OUTPUT      = 0x00E0,
	ACTION_MISC_ANALOG_OUTPUT       = 0x00F0,
	ACTION_START_TIMER              = 0x0100,
	ACTION_MODE_CHANGE              = 0x0110,
	ACTION_UNAVAILABLE              = 0xFFFF
};

enum ActionTypeParameter_t {
	ACTION_PARAMETER_SET_FALSE          = 0x00000000,
	ACTION_PARAMETER_SET_TRUE           = 0x00000001,
	ACTION_PARAMETER_RELEASE            = 0x00000010,
	ACTION_PARAMETER_PRESS              = 0x00000011,
	ACTION_PARAMETER_SET_VALUE          = 0x00000020,
	ACTION_PARAMETER_MIN                = 0x00000040,
	ACTION_PARAMETER_MAX                = 0x00000041,
	ACTION_PARAMETER_X                  = 0x00000070,
	ACTION_PARAMETER_Y                  = 0x00000071,
	ACTION_PARAMETER_Z                  = 0x00000072,
	ACTION_PARAMETER_VS_BRAKE_WARNING   = 0xA0000000,
	ACTION_PARAMETER_VS_SEND_START		= 0xA0000100,
	ACTION_PARAMETER_UNAVAILABLE        = 0xFFFFFFFF
};


enum ISOMessageReturnValue {
	MESSAGE_OK, // TODO delete
	MESSAGE_LENGTH_ERROR = -1,
	MESSAGE_TYPE_ERROR = -2,
	MESSAGE_CRC_ERROR = -3,
	MESSAGE_VERSION_ERROR = -4,
	MESSAGE_VALUE_ID_ERROR = -5,
	MESSAGE_SYNC_WORD_ERROR = -6,
	MESSAGE_CONTENT_OUT_OF_RANGE = -7,
	ISO_FUNCTION_ERROR = -8
};

/*! Valid ISO message identifiers */
enum ISOMessageID {
	MESSAGE_ID_INVALID = 0x0000,
	MESSAGE_ID_TRAJ = 0x0001,
	MESSAGE_ID_OSEM = 0x0002,
	MESSAGE_ID_OSTM = 0x0003,
	MESSAGE_ID_STRT = 0x0004,
	MESSAGE_ID_HEAB = 0x0005,
	MESSAGE_ID_MONR = 0x0006,
	MESSAGE_ID_MONR2 = 0x0007,
	MESSAGE_ID_SOWM = 0x0008,
	MESSAGE_ID_INFO = 0x0009,
	MESSAGE_ID_RCMM = 0x000A,
	MESSAGE_ID_SYPM = 0x000B,
	MESSAGE_ID_MTSP = 0x000C,
	MESSAGE_ID_DREQ = 0x0010,
	MESSAGE_ID_DRES = 0x0011,
	MESSAGE_ID_ACCM = 0x0012,
	MESSAGE_ID_TREO = 0x0013,
	MESSAGE_ID_EXAC = 0x0014,
	MESSAGE_ID_CATA = 0x0015,
	MESSAGE_ID_GREM = 0x0018,
	MESSAGE_ID_RCCM = 0x0020,
	MESSAGE_ID_TRCM = 0x0021,
	MESSAGE_ID_RCRT = 0x0033, //This is a temporary value, it was conflicting with TRCM
	MESSAGE_ID_PIME = 0x0030,
	MESSAGE_ID_COSE = 0x0031,
	MESSAGE_ID_MOMA = 0x0032,
	MESSAGE_ID_RESERVE_RANGE_1_LOWER_LIMIT = 0x0100,
	MESSAGE_ID_RESERVE_RANGE_1_UPPER_LIMIT = 0x0FFF,
	MESSAGE_ID_RESERVE_RANGE_2_LOWER_LIMIT = 0xF000,
	MESSAGE_ID_RESERVE_RANGE_2_UPPER_LIMIT = 0xFFFF,
	MESSAGE_ID_VENDOR_SPECIFIC_LOWER_LIMIT = 0xA100,
	MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT = 0xBFFF,
	MESSAGE_ID_VENDOR_SPECIFIC_RISE_INSUP = 0xA102,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO = 0xA100,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_FOPR = 0xA101,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_PODI = 0xA206,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCTI = 0xA121,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GDRM = 0xA120,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA = 0xA122,
	MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM = 0xA124
};

/*! Supervisor command */
enum SupervisorCommandType {
	SUPERVISOR_COMMAND_NORMAL = 1,	//!< Place supervisor in normal mode
	SUPERVISOR_COMMAND_DEBUG = 2	//!< Place supervisor in debug mode
};

/*! OPRO object type */
enum ObjectCategoryType {
	OBJECT_CATEGORY_UNKNOWN = 0,
	OBJECT_CATEGORY_CAR = 1,
	OBJECT_CATEGORY_BICYCLE = 2,
	OBJECT_CATEGORY_PEDESTRIAN = 30
};

/*! OPRO actor type */
enum ActorType {
	ACTOR_TYPE_UNKNOWN = 0,
	ACTOR_TYPE_VIRTUAL_OBJECT = 1,
	ACTOR_TYPE_REAL_OBJECT = 2
};

/*! OPRO operation mode */
enum OperationMode {
	OPERATION_MODE_UNKNOWN = 0,
	OPERATION_MODE_STATIC = 1,
	OPERATION_MODE_PREDEFINED_TRAJECTORY = 2,
	OPERATION_MODE_DYNAMIC_TRAJECTORY = 3,
	OPERATION_MODE_AUTONOMOUS_VUT = 4
};


enum ISOUnitType {
	ISO_UNIT_TYPE_STEERING_DEGREES = 0,
	ISO_UNIT_TYPE_STEERING_PERCENTAGE = 1,
	ISO_UNIT_TYPE_SPEED_METER_SECOND = 2,
	ISO_UNIT_TYPE_SPEED_PERCENTAGE = 3,
	ISO_UNIT_TYPE_THROTTLE_PERCENTAGE = 5,
	ISO_UNIT_TYPE_BRAKE_PERCENTAGE = 7
};

/*! GREM status */
enum GeneralResponseStatus {
    UNDEFIEND_0 = 0,
	GREM_OK = 1,
	GREM_GENERAL_ERROR = 2,
	GREM_NOT_SUPPORTED = 3,
	GREM_NO_MEMORY = 4,
	GREM_INVALID_DATA = 5,
	GREM_CHUNK_RECEIVED = 6,
    UNDEFIEND_1 = 7
};

enum RemoteControlManoeuvreCommandType {
	MANOEUVRE_NONE = 0,
	MANOEUVRE_BACK_TO_START = 3
};

/**
 * @brief struct for RCMM
 * 
 */
typedef struct{
	uint8_t status;
	union steeringMan{
		double_t pct;
		double_t rad;
	} steeringManoeuvre;
	union speedMan{
		double_t pct;
		double_t m_s;
	} speedManoeuvre;
	union throttleMan{
		double_t pct;
	} throttleManoeuvre;
	union brakeMan{
		double_t pct;
	} brakeManoeuvre;
	enum RemoteControlManoeuvreCommandType command;
	enum ISOUnitType steeringUnit;
	enum ISOUnitType speedUnit;
	enum ISOUnitType throttleUnit;
	enum ISOUnitType brakeUnit;
	bool isSteeringManoeuvreValid;
	bool isSpeedManoeuvreValid;
	bool isThrottleManoeuvreValid;
	bool isBrakeManoeuvreValid;
} RemoteControlManoeuvreMessageType;

/*! HEAB message contents */
typedef struct {
	uint32_t transmitterID;
	struct timeval dataTimestamp;
	enum ControlCenterStatusType controlCenterStatus;
} HeabMessageDataType;

/*! DCTI message contents */
typedef struct {
	uint16_t totalCount;
	uint16_t counter;
	uint32_t transmitterID;
} DctiMessageDataType;

/*! RDCA message contents */
typedef struct {
	uint32_t executingID;
	struct timeval dataTimestamp;
	union steeringActions{
		double_t pct;
		double_t rad;
	} steeringAction;
	union speedActions{
		double_t pct;
		double_t m_s;
	} speedAction;
	bool isSteeringActionValid;
	bool isSpeedActionValid;
	enum ISOUnitType steeringUnit;
	enum ISOUnitType speedUnit;
} RequestControlActionType;

/*! GDRM message contents */
typedef struct {
	enum GeneralDataRequestDataCodeType dataCode;
} GdrmMessageDataType;

/*! PODI message contents */
typedef struct {
	uint32_t foreignTransmitterID;
	struct timeval dataTimestamp;
	ObjectStateType state;
	CartesianPosition position;
	SpeedType speed;
	double pitch_rad;
	double roll_rad;
	bool isPitchValid;
	bool isRollValid;
} PeerObjectInjectionType;

/*! OPRO message contents */
typedef struct {
	uint32_t objectID;
	enum ObjectCategoryType objectType;
	enum ActorType actorType;
	enum OperationMode operationMode;
	double mass_kg;
	double objectXDimension_m;
	double objectYDimension_m;
	double objectZDimension_m;
	double positionDisplacementX_m;
	double positionDisplacementY_m;
	double positionDisplacementZ_m;
	bool isMassValid;
	bool isObjectXDimensionValid;
	bool isObjectYDimensionValid;
	bool isObjectZDimensionValid;
	bool isObjectXDisplacementValid;
	bool isObjectYDisplacementValid;
	bool isObjectZDisplacementValid;
} ObjectPropertiesType;


/*! FOPR message contents */
typedef struct {
	uint32_t foreignTransmitterID;
	enum ObjectCategoryType objectType;
	enum ActorType actorType;
	enum OperationMode operationMode;
	double mass_kg;
	double objectXDimension_m;
	double objectYDimension_m;
	double objectZDimension_m;
	double positionDisplacementX_m;
	double positionDisplacementY_m;
	double positionDisplacementZ_m;
	bool isMassValid;
	bool isObjectXDimensionValid;
	bool isObjectYDimensionValid;
	bool isObjectZDimensionValid;
	bool isObjectXDisplacementValid;
	bool isObjectYDisplacementValid;
	bool isObjectZDisplacementValid;
} ForeignObjectPropertiesType;

/*! STRT message contents */ 
typedef struct {
	struct timeval startTime;
	bool isTimestampValid;
} StartMessageType;

typedef struct{
    uint32_t receivedHeaderTransmitterID;
    uint8_t receivedHeaderMessageCounter;
    uint16_t receivedHeaderMessageID; 
    uint8_t responseCode;
    uint16_t payloadLength;
    uint8_t payload;
} GeneralResponseMessageType;



/*! Object type */
enum TestObjectType {
	OBJECT_TYPE_MOVEABLE = 0,
	OBJECT_TYPE_STATIONARY = 1
};

/*! DRES message contents */
typedef struct {
	char vendor[64];
	char productName[64];
	char firmwareVersion[64];
	char testObjectName[64];
	enum TestObjectType testObjectTypeCode;
	uint32_t subDeviceId;
} TestObjectDiscoveryType;

/*! DREQ status type */
enum DreqStatusType {
	DREQ_NOT_RECEIVED = 0,
	DREQ_RECEIVED = 1
};

/*! DREQ message contents */
typedef struct {
	enum DreqStatusType requestStatus;
} TestObjectDiscoveryRequestType;

ssize_t encodeMONRMessage(const MessageHeaderType *inputHeader, const struct timeval* objectTime, const CartesianPosition position, const SpeedType speed, const AccelerationType acceleration, const unsigned char driveDirection, const unsigned char objectState, const unsigned char readyToArm, const unsigned char objectErrorState, const unsigned short errorCode, char * monrDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeMONRMessage(const char * monrDataBuffer, const size_t bufferLength, const struct timeval currentTime, ObjectMonitorType * MonitorData, const char debug);
ssize_t encodeTRAJMessageHeader(const MessageHeaderType *inputHeader, const uint16_t trajectoryID, const TrajectoryInfoType trajectoryInfo, const char* trajectoryName, const size_t nameLength,	const uint32_t numberOfPointsInTraj, char *trajDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeTRAJMessagePoint(const struct timeval * pointTimeFromStart, const CartesianPosition position, const SpeedType speed, const AccelerationType acceleration, const float curvature, char * trajDataBufferPointer, const size_t remainingBufferLength, const char debug);
ssize_t decodeTRAJMessagePoint(TrajectoryWaypointType* wayPoints, const char* trajDataBuffer, const char debug);
ssize_t encodeTRAJMessageFooter(char * trajDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeTRAJMessageHeader(TrajectoryHeaderType* trajHeader, const char* trajDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeSTRTMessage(const MessageHeaderType *inputHeader, const StartMessageType* startData, char * strtDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeSTRTMessage(const char *strtDataBuffer, const size_t bufferLength, const struct timeval* currentTime, StartMessageType * startData, const char debug) ;
ssize_t encodeOSEMMessage(const MessageHeaderType *inputHeader, const ObjectSettingsType* objectSettingsData, char * osemDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeOSEMMessage(ObjectSettingsType *objectSettingsData, const char * osemDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeOSTMMessage(const MessageHeaderType *inputHeader, const enum ObjectCommandType command, char * ostmDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeOSTMMessage(const char* ostmDataBuffer, const size_t bufferLength, enum ObjectCommandType* command, const char debug);
ssize_t encodeHEABMessage(const MessageHeaderType *inputHeader, const struct timeval* heabTime, const enum ControlCenterStatusType status, char * heabDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeHEABMessage(const char *heabDataBuffer, const size_t bufferLength, const struct timeval currentTime, HeabMessageDataType* heabData, const char debug);
ssize_t encodeSYPMMessage(const MessageHeaderType *inputHeader, const struct timeval synchronizationTime, const struct timeval freezeTime, char * sypmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeMTSPMessage(const MessageHeaderType *inputHeader, const struct timeval * estSyncPointTime, char * mtspDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeTRCMMessage(const MessageHeaderType *inputHeader, const uint16_t* triggerID, const enum TriggerType_t* triggerType, const enum TriggerTypeParameter_t* param1, const enum TriggerTypeParameter_t* param2, const enum TriggerTypeParameter_t* param3, char * trcmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeACCMMessage(const MessageHeaderType *inputHeader, const uint16_t* actionID, const enum ActionType_t* actionType, const enum ActionTypeParameter_t* param1, const enum ActionTypeParameter_t* param2, const enum ActionTypeParameter_t* param3, char * accmDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeEXACMessage(const MessageHeaderType *inputHeader, const uint16_t* actionID, const struct timeval * executionTime, char * exacDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeRCMMMessage(const char *rcmmDataBuffer, const size_t bufferLength, RemoteControlManoeuvreMessageType* rcmmData, const char debug);
ssize_t encodeRCMMMessage(const MessageHeaderType *inputHeader, const RemoteControlManoeuvreMessageType* rcmmObjectData, char* rcmmDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeGREMMessage(const char *gremDataBuffer, const size_t bufferLength, GeneralResponseMessageType* gremData, const char debug);
ssize_t encodeGREMMessage(const MessageHeaderType *inputHeader, const GeneralResponseMessageType* gremObjectData, char* gremDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeDRESMessage(const MessageHeaderType *inputHeader, const TestObjectDiscoveryType *testObjectDiscoveryData, char *dresDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeDRESMessage(const char* dresDataBuffer, const size_t bufferLength, TestObjectDiscoveryType *testObjectDiscoveryData, const char debug);
ssize_t encodeDREQMessage(const MessageHeaderType *inputHeader, char *dreqDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeDREQMessage(const char* dreqDataBuffer, const size_t bufferLength, TestObjectDiscoveryRequestType *testObjectDiscoveryRequestData, const char debug);
ssize_t encodeINSUPMessage(const MessageHeaderType *inputHeader, const enum SupervisorCommandType, char * insupDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeDCTIMessage(const MessageHeaderType *inputHeader, const DctiMessageDataType *dctiData, char *dctiDataBuffer, const size_t bufferLength, const char debug);
enum ISOMessageReturnValue decodeDCTIMessage(const char *dctiDataBuffer, const size_t bufferLength, DctiMessageDataType* dctiData, const char debug);
enum ISOMessageID getISOMessageType(const char * messageData, const size_t length, const char debug);
void setISOCRCVerification(const int8_t enabled);

/* AstaZero vendor specific messages - TODO move to a separate repository */
ssize_t encodePODIMessage(const MessageHeaderType *inputHeader, const PeerObjectInjectionType* peerObjectData, char* podiDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodePODIMessage(const char *podiDataBuffer, const size_t bufferLength, const struct timeval currentTime, PeerObjectInjectionType* peerData, const char debug);
ssize_t encodeOPROMessage(const MessageHeaderType *inputHeader, const ObjectPropertiesType *objectPropertiesData, char * oproDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeOPROMessage(ObjectPropertiesType *objectPropertiesData, const char * oproDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeFOPRMessage(const MessageHeaderType *inputHeader, const ForeignObjectPropertiesType* foreignObjectPropertiesData, char *foprDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeFOPRMessage(ForeignObjectPropertiesType * foreignObjectPropertiesData, const char *foprDataBuffer, const size_t bufferLength, const char debug);
ssize_t encodeRDCAMessage(const MessageHeaderType *inputHeader, const RequestControlActionType* requestControlActionData, char *rdcaDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeRDCAMessage(const char *rdcaDataBuffer, RequestControlActionType* requestControlActionData, const size_t bufferLength, const struct timeval currentTime, const char debug);
ssize_t encodeGDRMMessage(const MessageHeaderType *inputHeader, const GdrmMessageDataType *gdrmData, char *gdrmDataBuffer, const size_t bufferLength, const char debug);
enum ISOMessageReturnValue decodeGDRMMessage(const char *gdrmDataBuffer, const size_t bufferLength, GdrmMessageDataType* gdrmData, const char debug);
ssize_t encodeDCMMMessage(const MessageHeaderType *inputHeader, const RemoteControlManoeuvreMessageType* command, char* dcmmDataBuffer, const size_t bufferLength, const char debug);
ssize_t decodeDCMMMessage(const char * dcmmDataBuffer, const size_t bufferLenght, RemoteControlManoeuvreMessageType* dcmmData, const char debug);
#ifdef __cplusplus
}
#endif
#endif
