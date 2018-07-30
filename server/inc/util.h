/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.h
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#ifndef __UTIL_H_INCLUDED__
#define __UTIL_H_INCLUDED__

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <inttypes.h>
#include <mqueue.h>
#include <math.h>

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
#define ISO_PROTOCOL_VERSION 2
#define ACK_REQ 0

#define MQ_LG     "/TEServer-LG"
#define MQ_SV     "/TEServer-SV"
#define MQ_OC     "/TEServer-OC"
#define MQ_VA     "/TEServer-VA"
#define MQ_SC     "/TEServer-SC"  

#define MQ_MAX_MESSAGE_LENGTH 4096
#define MQ_MAX_MSG            10
#define MQ_PERMISSION         0660

#define IPC_RECV       0x01
#define IPC_SEND       0x02
#define IPC_RECV_SEND  0x03

#define COMM_STRT 1
#define COMM_STOP 2
#define COMM_MONI 3
#define COMM_EXIT 4
#define COMM_ARMD 5
#define COMM_REPLAY 6
#define COMM_CONTROL 7
#define COMM_ABORT 8
#define COMM_TOM 9
#define COMM_INIT 10
#define COMM_CONNECT 11
#define COMM_OBC_STATE 12
#define COMM_DISCONNECT 13  
#define COMM_LOG 14
#define COMM_INV 255

#define SAFETY_CHANNEL_PORT 53240
#define CONTROL_CHANNEL_PORT 53241

#define MAX_OBJECTS 10
#define MAX_FILE_PATH 256

#define MAX_UTIL_VARIBLE_SIZE 512

#define a	6378137.0							            //meters in WGS84
#define k	298.257223563 						        //in WGS84, f = 1/298.257223563
#define b	6356752.3142451794975639665996337	//b = (1-f)*a
#define l	1e-12
#define PI	3.141592653589793
#define ORIGO_DISTANCE_CALC_ITERATIONS 14
#define TRAJECTORY_LINE_LENGTH 100
#define NUMBER_CHAR_LENGTH 20

#define SYNC_POINT_BUFFER 8000
#define TRAJ_RES 0.001   //m
#define TRAJ_POSITION_NOT_FOUND -1
#define TRAJ_MASTER_LATE -2

#define TIME_COMPENSATE_LAGING_VM 0
#define TIME_COMPENSATE_LAGING_VM_VAL 106407

#define MAX_ROW_SIZE 1024

#define MAX_ADAPTIVE_SYNC_POINTS  512

#define USE_TEST_HOST 0
#define TESTHOST_IP "10.168.10.136"
#define TESTSERVER_IP "10.168.10.136"
#define USE_LOCAL_USER_CONTROL  0
#define LOCAL_USER_CONTROL_IP "10.168.10.136" 
#define LOCAL_USER_CONTROL_PORT 54240  
#define TEST_SYNC_POINTS 0



/* Calculation: 	
  34 years between 1970 and 2004 
  8 days for leap year between 1970 and 2004 
*/

/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Calculation: 13 * 365 * 24 * 3600 * 1000 + 4 * 24 * 3600 * 1000 = 755568000 */
#define MS_FROM_2004_TO_2017_NO_LEAP_SECS 755568000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

#define TEST_CONF_FILE "./conf/test.conf"
#define TRAJECTORY_PATH "./traj/"

#define ADAPTIVE_SYNC_POINT_CONF "./conf/adaptivesync.conf"
#define TRIGG_ACTION_CONF "./conf/triggeraction.conf"

#define MAX_TRIGG_ACTIONS 20  


#define TAA_ACTION_EXT_START 1
#define TAA_ACTION_TEST_SIGNAL 2  

#define TAA_TRIGGER_DI_LOW  1
#define TAA_TRIGGER_DI_HIGH  2
#define TAA_TRIGGER_DI_RISING_EDGE 3 
#define TAA_TRIGGER_DI_FALLING_EDGE 4


#define MASTER_FILE_EXTENSION ".sync.m"
#define SLAVE_FILE_EXTENSION ".sync.s"


#define UNKNOWN 0
#define C8_CODE  1
#define U8_CODE  2
#define I8_CODE  3
#define U16_CODE  4
#define I16_CODE  5
#define U32_CODE  6
#define I32_CODE  7
#define U48_CODE  8
#define I48_CODE  9
#define U64_CODE  10
#define I64_CODE  11
#define DBL_CODE  12
#define FLT_CODE  13
#define STRUCT_CODE  254
#define RESERVED_CODE  255

#define VALUE_ID_NOT_DEF                    0
#define VALUE_ID_RELATIVE_TIME              0x1
#define VALUE_ID_GPS_SECOND_OF_WEEK         0x2
#define VALUE_ID_GPS_WEEK                   0x3
#define VALUE_ID_DATE_ISO8601               0x4
#define VALUE_ID_X_POSITION                 0x10
#define VALUE_ID_Y_POSITION                 0x11
#define VALUE_ID_Z_POSITION                 0x12
#define VALUE_ID_LATITUDE                   0x20
#define VALUE_ID_LONGITUDE                  0x21
#define VALUE_ID_ALTITUDE                   0x22
#define VALUE_ID_HEADING                    0x30
#define VALUE_ID_LONGITUDINAL_SPEED         0x40
#define VALUE_ID_LATERAL_SPEED              0x41
#define VALUE_ID_LONGITUDINAL_ACCELERATION  0x50
#define VALUE_ID_LATERAL_ACCELERATION       0x51
#define VALUE_ID_STATE_CHANGE_REQUEST       0x64
#define VALUE_ID_MAX_WAY_DEVIATION          0x70
#define VALUE_ID_MAX_LATERAL_DEVIATION      0x72
#define VALUE_ID_MIN_POS_ACCURACY           0x74
#define VALUE_ID_CURVATURE                  0xA000


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

#pragma pack(1) // #pragma pack ( 1 ) directive can be used for arranging memory for structure members very next to the end of other structure members.

#define SYNC_WORD 0x7e7e
//#define DEBUG

typedef struct
{
  double Latitude;
  double Longitude;
  double Altitude;
  double Heading;
} GeoPosition;


typedef struct
{
  U16 SyncWordU16;
  U8 TransmitterIdU8;
  U8 MessageCounterU8;
  U8 AckReqProtVerU8;
  U16 MessageIdU16;
  U32 MessageLengthU32;
} HeaderType; //11 bytes

typedef struct
{
  U16 Crc;
} FooterType; //2 bytes

typedef struct
{
  HeaderType Header;
  U16 LatitudeValueIdU16;
  U16 LatitudeContentLengthU16;
  I64 LatitudeI64;
  U16 LongitudeValueIdU16;
  U16 LongitudeContentLengthU16;
  I64 LongitudeI64;
  U16 AltitudeValueIdU16;
  U16 AltitudeContentLengthU16;
  I32 AltitudeI32;
  U16 DateValueIdU16;
  U16 DateContentLengthU16;
  U32 DateU32;
  U16 GPSWeekValueIdU16;
  U16 GPSWeekContentLengthU16;
  U16 GPSWeekU16;
  U16 GPSSOWValueIdU16;
  U16 GPSSOWContentLengthU16;
  U32 GPSSOWU32;
  U16 MaxWayDeviationValueIdU16;
  U16 MaxWayDeviationContentLengthU16;
  U16 MaxWayDeviationU16;
  U16 MaxLateralDeviationValueIdU16;
  U16 MaxLateralDeviationContentLengthU16;
  U16 MaxLateralDeviationU16;
  U16 MinPosAccuracyValueIdU16;
  U16 MinPosAccuracyContentLengthU16;
  U16 MinPosAccuracyU16;
} OSEMType; //85 bytes

typedef struct
{
  HeaderType Header;
  U16 StartTimeValueIdU16;
  U16 StartTimeContentLengthU16;
  U32 StartTimeU32;
  U16 DelayStartValueIdU16;
  U16 DelayStartContentLengthU16;
  U32 DelayStartU32;
} STRTType; //27 bytes

typedef struct
{
  HeaderType Header;
  U16 StateValueIdU16;
  U16 StateContentLengthU16;
  U8 StateU8;
} OSTMType; //16 bytes

typedef struct
{
  HeaderType Header;
  U32 GPSSOWU32;
  U8 CCStatusU8;
} HEABType; //16 bytes

typedef struct
{
  HeaderType Header;
  U32 GPSSOWU32;
  I32 XPositionI32;
  I32 YPositionI32;
  I32 ZPositionI32;
  U16 HeadingU16;
  I16 LongitudinalSpeedI16;
  I16 LateralSpeedI16;
  I16 LongitudinalAccI16;
  I16 LateralAccI16;
  U8 DriveDirectionU8;
  U8 StateU8;
  U8 ReadyToArmU8;
  U8 ErrorStatusU8;
} MONRType; //41 bytes


typedef struct
{
  U16 RelativeTimeValueIdU16;
  U16 RelativeTimeContentLengthU16;
  U32 RelativeTimeU32;
  U16 XPositionValueIdU16;
  U16 XPositionContentLengthU16;
  I32 XPositionI32;
  U16 YPositionValueIdU16;
  U16 YPositionContentLengthU16;
  I32 YPositionI32;
  U16 ZPositionValueIdU16;
  U16 ZPositionContentLengthU16;
  I32 ZPositionI32;
  U16 HeadingValueIdU16;
  U16 HeadingContentLengthU16;
  U16 HeadingU16;
  U16 LongitudinalSpeedValueIdU16;
  U16 LongitudinalSpeedContentLengthU16;
  I16 LongitudinalSpeedI16;
  U16 LateralSpeedValueIdU16;
  U16 LateralSpeedContentLengthU16;
  I16 LateralSpeedI16;
  U16 LongitudinalAccValueIdU16;
  U16 LongitudinalAccContentLengthU16;
  I16 LongitudinalAccI16;
  U16 LateralAccValueIdU16;
  U16 LateralAccContentLengthU16;
  I16 LateralAccI16;
  //U16 CurvatureValueIdU16;
  //U16 CurvatureContentLengthU16;
  //I32 CurvatureI32;
} DOTMType; //62 bytes


typedef struct
{
  U8 ProtocolVersionU8;
  U16 YearU16;
  U8 MonthU8;
  U8 DayU8;
  U8 HourU8;
  U8 MinuteU8;
  U8 SecondU8;
  U16 MillisecondU16;
  U16 MicroSecondU16;
  U32 SecondCounterU32;
  U64 GPSMillisecondsU64;
  U32 GPSMinutesU32;
  U16 GPSWeekU16;
  U32 GPSSecondsOfWeekU32;
  U32 GPSSecondsOfDayU32;
  U64 ETSIMillisecondsU64;
  U32 LatitudeU32;
  U32 LongitudeU32;
  U16 LocalMillisecondU16;
} TimeType;


typedef struct
{
  U16 TimeControlExecTimeU16;
  U16 SystemControlExecTimeU16;
  U16 ObjectControlExecTimeU16;
  U16 SimulatorControlExecTimeU16;
  U8 ExitU8;
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
  int SyncIndex;
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
  char TriggerIP[16];
  char TriggerType[8];
  char TriggerTypeVar[16];
  char ActionType[24];
  char ActionTypeVar[16];
  char ActionDelay[8];
  uint8_t TriggerId;
  int32_t Action;
} TriggActionType;

typedef struct
{
  U32 SessionIdU32;
  U32 UserIdU32;
  U8 UserTypeU8;
} ServiceSessionType; //9 bytes



/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
void util_error(char* message);
int iUtilGetParaConfFile(char* pcParameter, char* pcValue);
int iUtilGetIntParaConfFile(char* pcParameter, int* iValue);

int iCommInit(const unsigned int, const char*, const int);
int iCommClose();
int iCommRecv(int*, char*, const int);
int iCommSend(const int,const char*);

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

int UtilCountFileRows(FILE *fd);
int UtilReadLineCntSpecChars(FILE *fd, char *Buffer);
char UtilGetch();
int UtilAddEightBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data);
int UtilAddSixBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data);
int UtilAddFourBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned int Data);
int UtilAddTwoBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned short Data);
int UtilAddOneByteMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned char Data);
int UtilAddNBytesMessageData(unsigned char *MessageBuffer, int StartIndex, int Length, unsigned char *Data);
char * UtilSearchTextFile(char *Filename, char *Text1, char *Text2, char *Result);
int UtilSetMasterObject(ObjectPosition *OP, char *Filename, char debug);
int UtilSetSlaveObject(ObjectPosition *OP, char *Filename, char debug);
int UtilSetAdaptiveSyncPoint(AdaptiveSyncPoint *ASP, FILE *filefd, char debug);
void UtilSetObjectPositionIP(ObjectPosition *OP, char *IP);
int UtilSetTriggActions(TriggActionType *TAA, FILE *filefd, char debug);

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



#endif //__UTIL_H_INCLUDED__
