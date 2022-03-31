/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#pragma once



/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <inttypes.h>
#include <math.h>
#include <linux/limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>
#include "mqbus.h"
#include "iso22133.h"
#include "logging.h"
#include "positioning.h"

#ifdef __cplusplus
extern "C"{
#endif

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define MaestroVersion  "0.5.0"

#define DEFAULT_ORIGIN_LAT 57.777073115
#define DEFAULT_ORIGIN_LOG 12.781295498333
#define DEFAULT_ORIGIN_ALT 193.114
#define DEFAULT_ORIGIN_HEADING 0
#define DEFAULT_VISUALISATION_SERVER_NAME 0
#define DEFAULT_ASP_MAX_TIME_DIFF 2.5
#define DEFAULT_ASP_MAX_TRAJ_DIFF 1.52
#define DEFAULT_ASP_STEP_BACK_COUNT 0
#define DEFAULT_ASP_FILTER_LEVEL 5
#define DEFAULT_ASP_MAX_DELTA_TIME 0.05
#define DEFAULT_TIMESERVER_IP "10.130.23.110"
#define DEFAULT_TIME_SERVER_PORT 53000
#define DEFAULT_SIMULATOR_SIMULATOR_IP 0
#define DEFAULT_SIMULATOR_TCP_PORT 53001
#define DEFAULT_SIMULATOR_UDP_PORT 53002
#define DEFAULT_SIMULATOR_MODE 1
#define DEFAULT_VOIL_RECEIVERS 0
#define DEFAULT_DTM_RECEIVERS 0
#define DEFAULT_SUPERVISOR_IP 0
#define DEFAULT_SUPERVISOR_TCP_PORT 53010
#define DEFAULT_RVSS_CONF 3
#define DEFAULT_RVSS_RATE 1
#define DEFAULT_MAX_PACKETS_LOST 20
#define DEFAULT_TRANSMITTER_ID 0

#define MBUS_MAX_DATALEN (MQ_MSG_SIZE-9) // Message queue data minus one byte for the command and 8 for the data length

#define MAX_OBJECTS 10
#define MAX_FILE_PATH PATH_MAX

#define MAX_UTIL_VARIBLE_SIZE 512

#define ORIGO_DISTANCE_CALC_ITERATIONS 14
#define TRAJECTORY_LINE_LENGTH 100
#define NUMBER_CHAR_LENGTH 20

#define SYNC_POINT_BUFFER 8000
#define TRAJ_RES 0.001   //m
#define TRAJ_POSITION_NOT_FOUND -1
#define TRAJ_MASTER_LATE -2

#define TIME_COMPENSATE_LAGING_VM 0
#define VIRTUAL_MACHINE_LAG_COMPENSATION_S 106
#define VIRTUAL_MACHINE_LAG_COMPENSATION_US 407000

#define MAX_ROW_SIZE 1024

#define TCP_RX_BUFFER 1024
#define MAX_ADAPTIVE_SYNC_POINTS  512

#define USE_LOCAL_USER_CONTROL  0
#define LOCAL_USER_CONTROL_IP "192.168.0.10"
#define USE_TEST_HOST 0
#define TESTHOST_IP LOCAL_USER_CONTROL_IP
#define TESTSERVER_IP LOCAL_USER_CONTROL_IP
#define LOCAL_USER_CONTROL_PORT 54240
#define TEST_SYNC_POINTS 0



/* Calculation:
  34 years between 1970 and 2004
  8 days for leap year between 1970 and 2004
*/

/* Calculation: 13 * 365 * 24 * 3600 * 1000 + 4 * 24 * 3600 * 1000 = 755568000 */
#define MS_FROM_2004_TO_2017_NO_LEAP_SECS 755568000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

// 7 * 24 * 3600 * 1000
#define WEEK_TIME_MS 604800000
// 24 * 3600 * 1000
#define DAY_TIME_MS 86400000
// 3600 * 1000
#define HOUR_TIME_MS 3600000
// 60 * 1000
#define MINUTE_TIME_MS 60000

#define CONF_FILE_NAME "test.conf"
#define ADAPTIVE_SYNC_FILE_NAME "adaptivesync.conf"
#define TRIGGER_ACTION_FILE_NAME "triggeraction.conf"

#define MASTER_FILE_EXTENSION ".sync.m"
#define SLAVE_FILE_EXTENSION ".sync.s"

enum ConfigurationFileParameter {
	CONFIGURATION_PARAMETER_SCENARIO_NAME,
	CONFIGURATION_PARAMETER_ORIGIN_LATITUDE,
	CONFIGURATION_PARAMETER_ORIGIN_LONGITUDE,
	CONFIGURATION_PARAMETER_ORIGIN_ALTITUDE,
	CONFIGURATION_PARAMETER_VISUALIZATION_SERVER_NAME,
	CONFIGURATION_PARAMETER_ASP_MAX_TIME_DIFF,
	CONFIGURATION_PARAMETER_ASP_MAX_TRAJ_DIFF,
	CONFIGURATION_PARAMETER_ASP_STEP_BACK_COUNT,
	CONFIGURATION_PARAMETER_ASP_FILTER_LEVEL,
	CONFIGURATION_PARAMETER_ASP_MAX_DELTA_TIME,
	CONFIGURATION_PARAMETER_TIME_SERVER_IP,
	CONFIGURATION_PARAMETER_TIME_SERVER_PORT,
	CONFIGURATION_PARAMETER_SIMULATOR_IP,
	CONFIGURATION_PARAMETER_SIMULATOR_PORT_TCP,
	CONFIGURATION_PARAMETER_SIMULATOR_PORT_UDP,
	CONFIGURATION_PARAMETER_SIMULATOR_MODE,
	CONFIGURATION_PARAMETER_VOIL_RECEIVERS,
	CONFIGURATION_PARAMETER_DTM_RECEIVERS,
	CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_IP,
	CONFIGURATION_PARAMETER_EXTERNAL_SUPERVISOR_PORT_TCP,
	CONFIGURATION_PARAMETER_RVSS_CONFIG,
	CONFIGURATION_PARAMETER_RVSS_RATE,
	CONFIGURATION_PARAMETER_MAX_PACKETS_LOST,
	CONFIGURATION_PARAMETER_TRANSMITTER_ID,
	CONFIGURATION_PARAMETER_MISC_DATA,
	CONFIGURATION_PARAMETER_INVALID
};

enum ObjectFileParameter {
	OBJECT_SETTING_ID,
	OBJECT_SETTING_IP,
	OBJECT_SETTING_TRAJ,
	OBJECT_SETTING_IS_ANCHOR,
	OBJECT_SETTING_INJECTOR_IDS,
	OBJECT_SETTING_ORIGIN_LATITUDE,
	OBJECT_SETTING_ORIGIN_LONGITUDE,
	OBJECT_SETTING_ORIGIN_ALTITUDE,
	OBJECT_SETTING_TURNING_DIAMETER,
	OBJECT_SETTING_MAX_SPEED,
	OBJECT_SETTING_IS_OSI_COMPATIBLE
};

#define C8 uint8_t
#define U8 uint8_t
#define I8 int8_t
#define U16 uint16_t
#define I16 int16_t
#define U32 uint32_t
#define I32 int32_t
#define U64 uint64_t
#define I64 int64_t
#define dbl double
#define flt float

#define SERVER_PREPARED 0x01
#define SERVER_PREPARED_BIG_PACKET_SIZE 0x02
#define PATH_INVALID_MISSING 0x03
#define FILE_UPLOADED 0x04
#define TIME_OUT 0x05
#define FILE_EXIST 0x01
#define FOLDER_EXIST 0x02
#define FILE_EXISTED 0x01
#define FOLDER_EXISTED 0x02
#define SUCCEDED_CREATE_FOLDER 0x03
#define FAILED_CREATE_FOLDER 0x04
#define SUCCEEDED_DELETE 0x01
#define FAILED_DELETE 0x02
#define FILE_TO_MUCH_DATA 0x06

#define GetCurrentDir getcwd
#define MAX_PATH_LENGTH 255

#define DD_CONTROL_BUFFER_SIZE_1024 1024
#define DD_CONTROL_BUFFER_SIZE_20 20
#define DD_CONTROL_BUFFER_SIZE_52 52
#define DD_CONTROL_TASK_PERIOD_MS 1

//! Internal message queue communication identifiers
enum COMMAND
{
COMM_STRT = 1,
COMM_ARM = 2,
COMM_STOP = 3,
COMM_EXIT = 5,
COMM_REPLAY = 6,
COMM_ABORT = 8,
COMM_ABORT_DONE = 9,
COMM_INIT = 10,
COMM_CONNECT = 11,
COMM_OBC_STATE = 12,
COMM_DISCONNECT = 13,
COMM_VIOP = 15,
COMM_TRAJ = 16,
COMM_TRAJ_TOSUP = 17,
COMM_TRAJ_FROMSUP = 18,
COMM_ASP = 19,
COMM_OSEM = 20,
COMM_DATA_DICT = 21,
COMM_EXAC = 22,
COMM_TREO = 23,
COMM_ACCM = 24,
COMM_TRCM = 25,
COMM_DISARM = 26,
COMM_GETSTATUS = 27,
COMM_GETSTATUS_OK = 28,
COMM_BACKTOSTART_CALL = 29,
COMM_BACKTOSTART_RESPONSE = 30,
COMM_REMOTECTRL_ENABLE = 31,
COMM_REMOTECTRL_DISABLE = 32,
COMM_REMOTECTRL_MANOEUVRE = 33,
COMM_ENABLE_OBJECT = 34,
COMM_CONTROL_SIGNAL_PERCENTAGE = 35,
COMM_OBJECTS_CONNECTED = 111,
COMM_FAILURE = 254,
COMM_INV = 255
};

typedef struct {
	enum RemoteControlManoeuvreCommandType manoeuvre;
	in_addr_t objectIP;
} ManoeuvreCommandType;

typedef struct
{
  double Latitude;
  double Longitude;
  double Altitude;
  double Heading;
} GeoPositionType;


typedef enum {
	OBJECT_ENABLED = 1,
	OBJECT_DISABLED = 2,
	OBJECT_UNDEFINED = 3
} ObjectEnabledType;

typedef enum {
	BTS_FAIL = -2,
	BTS_ERROR = -1,
	BTS_PASS = 0
} BTSResponse;


typedef struct {
	uint32_t ClientID;
	in_addr_t ClientIP;
	ObjectEnabledType Enabled;
	ObjectMonitorType MonrData;
	struct timeval lastPositionUpdate;
	ObjectPropertiesType properties;
	bool propertiesReceived;
	struct timeval lastDataUpdate;
  GeoPositionType origin;
	RequestControlActionType requestedControlAction;
} ObjectDataType;

typedef struct {
	in_addr_t objectIP;
	ObjectEnabledType Enabled;
} ObjectEnabledCommandType;

typedef struct {
	unsigned int ID;
	char name[128];
	unsigned short majorVersion;
	unsigned short minorVersion;
	unsigned int numberOfLines;
} TrajectoryFileHeader;

typedef struct {
	double time;
	double xCoord;
	double yCoord;
	double *zCoord;
	double heading;
	double *longitudinalVelocity;
	double *lateralVelocity;
	double *longitudinalAcceleration;
	double *lateralAcceleration;
	double curvature;
	uint8_t mode;
} TrajectoryFileLine;

typedef struct
{
  volatile U8 isGPSenabled;
  volatile U8 ProtocolVersionU8;
  volatile U16 YearU16;
  volatile U8 MonthU8;
  volatile U8 DayU8;
  volatile U8 HourU8;
  volatile U8 MinuteU8;
  volatile U8 SecondU8;
  volatile U16 MillisecondU16;
  volatile U16 MicroSecondU16;
  volatile U32 SecondCounterU32;
  volatile U64 GPSMillisecondsU64;
  volatile U32 GPSMinutesU32;
  volatile U16 GPSWeekU16;
  volatile U32 GPSSecondsOfWeekU32;
  volatile U32 GPSSecondsOfDayU32;
  volatile U64 ETSIMillisecondsU64;
  volatile U32 LatitudeU32;
  volatile U32 LongitudeU32;
  volatile U16 LocalMillisecondU16;
  volatile U8 FixQualityU8;
  volatile U8 NSatellitesU8;
  volatile U8 isTimeInitializedU8;
} TimeType;


typedef struct
{
  U32 MessageLengthU32;
  U32 ChannelCodeU32;
  U32 MTSPU32;
  dbl CurrentTimeDbl;
  dbl TimeToSyncPointDbl;
  dbl PrevTimeToSyncPointDbl;
  I32 SyncPointIndexI32;
  U32 CurrentTimeU32;
  I32 BestFoundIndexI32;
  U16 IterationTimeU16;
} ASPType;

/*! Object control states */
typedef enum {
    OBC_STATE_UNDEFINED,
    OBC_STATE_IDLE,
    OBC_STATE_INITIALIZED,
    OBC_STATE_CONNECTED,
    OBC_STATE_ARMED,
    OBC_STATE_RUNNING,
	  OBC_STATE_REMOTECTRL,
    OBC_STATE_ERROR,
    OBC_STATE_ABORTING
} OBCState_t;

typedef struct
{

  U16 TimeControlExecTimeU16;
  U16 SystemControlExecTimeU16;
  U16 ObjectControlExecTimeU16;
  U16 SimulatorControlExecTimeU16;
  U8 ExitU8;
  U32 ScenarioStartTimeU32;
  U8 VOILData[400];
  U32 ChunkSize;
  U8 Chunk[6200];
  U8 ASPDebugDataSetU8;
  U8 ASPDebugDataU8[sizeof(ASPType)];
  U32 SupChunkSize;
  U8 SupChunk[6200];
  U8 MONRSizeU8;
  U8 MONRData[100];
  U8 HEABSizeU8;
  U8 HEABData[100];

  U8 numberOfObjects;
  char *memory;
  volatile dbl OriginLatitudeDbl;
  C8 OriginLatitudeC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile dbl OriginLongitudeDbl;
  C8 OriginLongitudeC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile dbl OriginAltitudeDbl;
  C8 OriginAltitudeC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile U32 VisualizationServerU32;
  C8 VisualizationServerC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile dbl ASPMaxTimeDiffDbl;
  volatile dbl ASPMaxTrajDiffDbl;
  volatile U32 ASPStepBackCountU32;
  volatile dbl ASPFilterLevelDbl;
  volatile dbl ASPMaxDeltaTimeDbl;
  volatile U32 TimeServerIPU32;
  C8 TimeServerIPC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile U16 TimeServerPortU16;
  volatile U32 SimulatorIPU32;
  C8 SimulatorIPC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile U16 SimulatorTCPPortU16;
  volatile U16 SimulatorUDPPortU16;
  volatile U8 SimulatorModeU8;
  C8 VOILReceiversC8[DD_CONTROL_BUFFER_SIZE_1024];
  C8 DTMReceiversC8[DD_CONTROL_BUFFER_SIZE_1024];
  volatile U32 ExternalSupervisorIPU32;
  C8 ExternalSupervisorIPC8[DD_CONTROL_BUFFER_SIZE_20];
  volatile U16 SupervisorTCPPortU16;
  U32 DataDictionaryRVSSConfigU32;
  U32 DataDictionaryRVSSRateU8;
  ASPType ASPData;
  C8 MiscDataC8[DD_CONTROL_BUFFER_SIZE_1024];
} GSDType;


typedef struct
{
  int Index;
  float Time;
  float OrigoDistance;
  double Bearing;
  float x;
  float y;

} SpaceTime;


typedef struct
{
    char Type;
  double Latitude;
    double Longitude;
    double OrigoDistance;
  double OldOrigoDistance;
    double DeltaOrigoDistance;
  double x;
    double y;
    double z;
    int CalcIterations;
    double ForwardAzimuth1;
    double ForwardAzimuth2;
  int TrajectoryPositionCount;
  I32 SyncIndex;
  double SyncTime;
  double SyncStopTime;
  int BestFoundTrajectoryIndex;
  int SpaceTimeFoundIndex;
  float TimeToSyncPoint;
  float* SpaceArr;
  float* TimeArr;
  float CurrentAltitude;
  float InitialAltitude;
  float ReferenceAltitude;
  SpaceTime* SpaceTimeArr;
  char IP[16];
  int Id;
  float Speed;
} ObjectPosition;


//#master_ip;slave_ip;time_on_traj_master,time_on_traj_slave;slave_stop;
typedef struct
{
  char MasterIP[16];
  char SlaveIP[16];
  float MasterTrajSyncTime;
  float SlaveTrajSyncTime;
  float SlaveSyncStopTime;
  uint32_t TestPort;
} AdaptiveSyncPoint;


typedef struct
{
    uint16_t actionID;
    uint32_t executionTime_qmsoW;
    in_addr_t ip;
} EXACData; //!< Data type for MQ message



typedef struct
{
    uint16_t triggerID;
    uint32_t timestamp_qmsow;
    in_addr_t ip;
} TREOData; //!< Data type for MQ message

typedef struct
{
    uint16_t actionID;
    uint16_t actionType;
    uint32_t actionTypeParameter1;
    uint32_t actionTypeParameter2;
    uint32_t actionTypeParameter3;
    in_addr_t ip;
} ACCMData; //!< Data type for MQ message

typedef struct
{
    uint16_t triggerID;
    uint16_t triggerType;
    uint32_t triggerTypeParameter1;
    uint32_t triggerTypeParameter2;
    uint32_t triggerTypeParameter3;
    in_addr_t ip;
} TRCMData; //!< Data type for MQ message



typedef struct
{
  U32 SessionIdU32;
  U32 UserIdU32;
  U8 UserTypeU8;
} ServiceSessionType; //9 bytes


#define HTTP_HEADER_MAX_LENGTH 64
typedef struct {
    char AcceptCharset[HTTP_HEADER_MAX_LENGTH];
    char AcceptEncoding[HTTP_HEADER_MAX_LENGTH];
    char AcceptLanguage[HTTP_HEADER_MAX_LENGTH];
    char Authorization[HTTP_HEADER_MAX_LENGTH];
    char Expect[HTTP_HEADER_MAX_LENGTH];
    char From[HTTP_HEADER_MAX_LENGTH];
    char Host[HTTP_HEADER_MAX_LENGTH];
    char IfMatch[HTTP_HEADER_MAX_LENGTH];
    char IfModifiedSince[HTTP_HEADER_MAX_LENGTH];
    char IfNoneMatch[HTTP_HEADER_MAX_LENGTH];
    char IfRange[HTTP_HEADER_MAX_LENGTH];
    char IfUnmodifiedSince[HTTP_HEADER_MAX_LENGTH];
    char MaxForwards[HTTP_HEADER_MAX_LENGTH];
    char ProxyAuthorization[HTTP_HEADER_MAX_LENGTH];
    char Range[HTTP_HEADER_MAX_LENGTH];
    char Referer[HTTP_HEADER_MAX_LENGTH];
    char TE[HTTP_HEADER_MAX_LENGTH];
    char UserAgent[HTTP_HEADER_MAX_LENGTH];
} HTTPHeaderContent;

/*! Data dictionary read/write return codes. */
typedef enum {
	UNDEFINED = -3, /*!< Undefined result */
	WRITE_OK = 1,  /*!< Write successful */
	READ_OK = 2,   /*!< Read successful */
	WRITE_FAIL = -1, /*! Write unsuccessful */
	READ_FAIL = -2,  /*! Read unsuccessful */
	UNINITIALIZED = -4,		/*!< Read successful but data not initialized */
	READ_WRITE_OK = 0,		/*!< Combined read/write successful */
	PARAMETER_NOTFOUND = -5, /*!< Read/write not successful */
	OUT_OF_RANGE = -6		/*!< Attempted to read out of range */
} ReadWriteAccess_t;

#pragma pack(push,1)

typedef struct
{
  U32 MessageLengthU32;
  U32 ChannelCodeU32;
  U16 YearU16;
  U8 MonthU8;
  U8 DayU8;
  U8 HourU8;
  U8 MinuteU8;
  U8 SecondU8;
  U16 MillisecondU16;
  U32 SecondCounterU32;
  U64 GPSMillisecondsU64;
  U32 GPSMinutesU32;
  U16 GPSWeekU16;
  U32 GPSSecondsOfWeekU32;
  U32 GPSSecondsOfDayU32;
  U8 FixQualityU8;
  U8 NSatellitesU8;
} RVSSTimeType;


typedef struct
{
  U32 MessageLengthU32;
  U32 ChannelCodeU32;
  U8 OBCStateU8;
  U8 SysCtrlStateU8;
} RVSSMaestroType;

#pragma pack(pop)

typedef enum {
    NORTHERN,
    SOUTHERN
} Hemisphere;



/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

// HTTP decoding functions
void UtilDecodeHTTPRequestHeader(char* request, HTTPHeaderContent* header);

// GPS TIME FUNCTIONS
uint64_t UtilgetGPSmsFromUTCms(uint64_t UTCms);
uint64_t UtilgetUTCmsFromGPSms(uint64_t GPSms);
uint64_t UtilgetMSfromGPStime(uint16_t GPSweek,uint32_t GPSquarterMSofWeek);
void UtilgetGPStimeFromMS(uint64_t GPSms, uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek);
// GPS collective functions
void UtilgetGPStimeFromUTCms(uint64_t UTCms,uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek);
uint64_t UtilgetUTCmsFromGPStime(uint16_t GPSweek,uint32_t GPSquarterMSofWeek);
void UtilgetCurrentGPStime(uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek);

// Get time function
uint64_t UtilgetCurrentUTCtimeMS();
uint32_t UtilgetIntDateFromMS(uint64_t ms);
uint64_t UtilgetETSIfromUTCMS(uint64_t utc_sec, uint64_t utc_usec);

void UtilgetDateTimeFromUTCtime(int64_t utc_ms, char *buffer, int size_t);
void UtilgetDateTimefromUTCCSVformat(int64_t utc_ms, char *buffer, int size_t);
void UtilgetDateTimeFromUTCForMapNameCreation(int64_t utc_ms, char *buffer, int size_t);

void util_error(const char *message);
int iUtilGetParaConfFile(char* pcParameter, char* pcValue);
int iUtilGetIntParaConfFile(char* pcParameter, int* iValue);

// Message bus functions
int iCommInit(void);
int iCommClose(void);
ssize_t iCommRecv(enum COMMAND *command, char* data, const size_t messageSize, struct timeval *timeRecv);
int iCommSend(const enum COMMAND iCommand, const char* data, size_t dataLength);

int iCommSendTREO(TREOData data);
int iCommSendTRCM(TRCMData data);
int iCommSendEXAC(EXACData data);
int iCommSendACCM(ACCMData data);

// File system functions
int UtilVerifyTestDirectory();
int UtilCopyFile(const char* source, const size_t sourceLen, const char* dest, const size_t destLen);
void UtilGetTestDirectoryPath(char* path, size_t pathLen);
void UtilGetJournalDirectoryPath(char* path, size_t pathLen);
void UtilGetConfDirectoryPath(char* path, size_t pathLen);
void UtilGetTrajDirectoryPath(char* path, size_t pathLen);
void UtilGetGeofenceDirectoryPath(char* path, size_t pathLen);
void UtilGetObjectDirectoryPath(char* path, size_t pathLen);

int UtilDeleteTrajectoryFiles(void);
int UtilDeleteGeofenceFiles(void);
int UtilDeleteObjectFiles(void);

int UtilDeleteTrajectoryFile(const char * geofencePath, const size_t nameLen);
int UtilDeleteGeofenceFile(const char * geofencePath, const size_t nameLen);
int UtilDeleteObjectFile(const char * geofencePath, const size_t nameLen);
int UtilDeleteGenericFile(const char * genericFilePath, const size_t nameLen);

// File parsing functions
int UtilCheckTrajectoryFileFormat(const char *path, size_t pathLen);
int UtilParseTrajectoryFileHeader(char *headerLine, TrajectoryFileHeader * header);
int UtilParseTrajectoryFileFooter(char *footerLine);
int UtilParseTrajectoryFileLine(char *fileLine, TrajectoryFileLine * line);


int UtilObjectDataToString(const ObjectDataType monrData, char* monrString, size_t stringLength);
int UtilStringToMonitorData(const char* monrString, size_t stringLength, ObjectDataType * monrData);
uint8_t UtilIsPositionNearTarget(CartesianPosition position, CartesianPosition target, double tolerance_m);
uint8_t UtilIsAngleNearTarget(CartesianPosition position, CartesianPosition target, double tolerance);
double UtilCalcPositionDelta(double P1Lat, double P1Long, double P2Lat, double P2Long, ObjectPosition *OP);
int UtilVincentyDirect(double refLat, double refLon, double a1, double distance, double *resLat, double *resLon, double *a2);
double UtilDegToRad(double Deg);
double UtilRadToDeg(double Rad);
int UtilPopulateSpaceTimeArr(ObjectPosition *OP, char* TrajFile);
int UtilSortSpaceTimeAscending(ObjectPosition *OP);
int UtilFindCurrentTrajectoryPosition(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug);
int UtilFindCurrentTrajectoryPositionNew(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug);
int UtilSetSyncPoint(ObjectPosition *OP, double x, double y, double z, double time);
float UtilCalculateTimeToSync(ObjectPosition *OP);

char UtilIsPointInPolygon(const CartesianPosition point, const CartesianPosition* polygonPoints, unsigned int nPtsInPolygon);

int UtilCountFileRows(FILE *fd);
int UtilCountFileRowsInPath(const char *path, const size_t pathlen);
int UtilGetRowInFile(const char *path, const size_t pathLength, I32 rowIndex, char *rowBuffer, const size_t bufferLength);
int UtilReadLineCntSpecChars(FILE *fd, char *Buffer);
int UtilReadLine(FILE *fd, char *Buffer);
char UtilGetch();
int UtilAddEightBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data);
int UtilAddSixBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data);
int UtilAddFourBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned int Data);
int UtilAddTwoBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned short Data);
int UtilAddOneByteMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned char Data);
int UtilAddNBytesMessageData(unsigned char *MessageBuffer, int StartIndex, int Length, unsigned char *Data);
C8 * UtilSearchTextFile(C8 *Filename, C8 *Text1, C8 *Text2, C8 *Result);
int UtilSetMasterObject(ObjectPosition *OP, char *Filename, char debug);
int UtilSetSlaveObject(ObjectPosition *OP, char *Filename, char debug);
int UtilSetAdaptiveSyncPoint(AdaptiveSyncPoint *ASP, FILE *filefd, char debug);
void UtilSetObjectPositionIP(ObjectPosition *OP, char *IP);

void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z);
void enuToLlh(const double *iLlh, const double *xyz, double *llh);
void createEnuMatrix(double lat, double lon, double *enuMat);
void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height);
void llhToEnu(const double *iLlh, const double *llh, double *xyz);
uint16_t crc_16( const unsigned char *input_str, uint16_t num_bytes );

U16 SwapU16(U16 val);
I16 SwapI16(I16 val);
U32 SwapU32(U32 val);
I32 SwapI32(I32 val);
I64 SwapI64(I64 val);
U64 SwapU64(U64 val);

I32 UtilConnectTCPChannel(const C8* Module, I32* Sockfd, const C8* IP, const U32 Port);
void UtilSendTCPData(const C8* Module, const C8* Data, I32 Length, const int* Sockfd, U8 Debug);
I32 UtilReceiveTCPData(const C8* Module, I32* Sockfd, C8* Data, I32 Length, U8 Debug);
void UtilCreateUDPChannel(const C8* Module, I32 *Sockfd, const C8* IP, const U32 Port, struct sockaddr_in* Addr);
void UtilSendUDPData(const C8* Module, I32 *Sockfd, struct sockaddr_in* Addr, C8 *Data, size_t Length, U8 Debug);
void UtilReceiveUDPData(const C8* Module, I32* Sockfd, C8* Buffer, I32 Length, I32* ReceivedNewData, U8 Debug);
U32 UtilIPStringToInt(C8 *IP);
U32 UtilBinaryToHexText(U32 DataLength, C8 *Binary, C8 *Text, U8 Debug);
U32 UtilHexTextToBinary(U32 DataLength, C8 *Text, C8 *Binary, U8 Debug);

U32 UtilCreateDirContent(C8* DirPath, C8* TempPath);
U16 UtilGetMillisecond(TimeType *GPSTime);
int32_t UtilWriteConfigurationParameter(const enum ConfigurationFileParameter parameter, const char* newValue, const size_t bufferLength);
int32_t UtilReadConfigurationParameter(const enum ConfigurationFileParameter parameter, char* returnValue, const size_t bufferLength);
char* UtilGetConfigurationParameterAsString(const enum ConfigurationFileParameter parameter, char* returnValue, const size_t bufferLength);
enum ConfigurationFileParameter UtilParseConfigurationParameter(const char* parameter, const size_t bufferLength);
char *UtilGetObjectParameterAsString(const enum ObjectFileParameter parameter, char *returnValue, const size_t bufferLength);
int UtilGetObjectFileSetting(const enum ObjectFileParameter setting, const char* objectFilePath,
							 const size_t filePathLength, char* objectSetting,
							 const size_t objectSettingSize);
int UtilReadOriginConfiguration(GeoPositionType* origin);

int UtilPopulateMonitorDataStruct(const char * rawMONR, const size_t rawMONRsize, ObjectDataType *monitorData);
int UtilPopulateTREODataStructFromMQ(char* rawTREO, size_t rawTREOsize, TREOData *treoData);
int UtilPopulateEXACDataStructFromMQ(char* rawEXAC, size_t rawEXACsize, EXACData *exacData);
int UtilPopulateTRCMDataStructFromMQ(char* rawTRCM, size_t rawTRCMsize, TRCMData *trcmData);
int UtilPopulateACCMDataStructFromMQ(char* rawACCM, size_t rawACCMsize, ACCMData *accmData);

struct timeval UtilGetPIDUptime(pid_t pID);
double UtilGetDistance(double lat1, double lon1, double lat2, double lon2);

typedef struct {
  uint64_t timestamp;
  int32_t  latitude;
  int32_t  longitude;
  int32_t  altitude;
  uint16_t speed;
  uint16_t heading;
  uint8_t  drivedirection;
} monitor_t;


/*------------------------------------------------------------
  -- Function traj2ldm
  --  converts a traj file format to a ldm:monitor_t
  ------------------------------------------------------------*/

void traj2ldm ( float      time ,
                double     x    ,
                double     y    ,
                double     z    ,
                float      hdg  ,
                float      vel  ,
                monitor_t* ldm  );


#ifdef __cplusplus
}
#endif

