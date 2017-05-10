/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.h
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
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
#define MQ_LG     "/TEServer-LG"
#define MQ_SV     "/TEServer-SV"
#define MQ_OC     "/TEServer-OC"
#define MQ_VA     "/TEServer-VA"
#define MQ_SC     "/DUMMY"

#define MQ_MAX_MESSAGE_LENGTH 4096
#define MQ_MAX_MSG            10
#define MQ_PERMISSION         0660

#define IPC_RECV       0x01
#define IPC_SEND       0x02
#define IPC_RECV_SEND  0x03

#define COMM_TRIG 1
#define COMM_STOP 2
#define COMM_MONI 3
#define COMM_EXIT 4
#define COMM_ARMD 5
#define COMM_REPLAY 6
#define COMM_CONTROL 7
#define COMM_ABORT 8
#define COMM_INV 255

#define SAFETY_CHANNEL_PORT 53240
#define CONTROL_CHANNEL_PORT 53241

#define MAX_OBJECTS 10
#define MAX_FILE_PATH 256

#define MAX_UTIL_VARIBLE_SIZE 512

#define a	6378137.0							//meters in WGS84
#define k	298.257223563 						//in WGS84, f = 1/298.257223563
#define b	6356752.3142451794975639665996337	//b = (1-f)*a
#define l	1e-12
#define PI	3.141592653589793
#define ORIGO_DISTANCE_CALC_ITERATIONS 14
#define TRAJECTORY_LINE_LENGTH 100
#define NUMBER_CHAR_LENGTH 20

#define MAX_TIME_DIFF 5.0
#define SYNC_POINT_BUFFER 30


/* Calculation: 
  34 years between 1970 and 2004 
  8 days for leap year between 1970 and 2004 
*/

/* Calculation: 34 * 365 * 24 * 3600 * 1000 + 8 * 24 * 3600 * 1000 = 1072915200000 */
#define MS_FROM_1970_TO_2004_NO_LEAP_SECS 1072915200000

/* Difference of leap seconds between UTC and ETSI */
#define DIFF_LEAP_SECONDS_UTC_ETSI 5

#define TEST_CONF_FILE "./conf/test.conf"
#define TRAJECTORY_PATH "./traj/"

//#define DEBUG

typedef struct
{
  int Index;
  float Time;
  float OrigoDistance;

} SpaceTime;


typedef struct 
{
	double Latitude;
	double Longitude;
	double OrigoDistance;
	double x;
	double y;
	double z;
	int CalcIterations;
	double ForwardAzimuth1;
	double ForwardAzimuth2;
  int TrajectoryPositionCount;
  int SyncIndex;
  int BestFoundTrajectoryIndex;
  float TimeToSyncPoint;
  float* SpaceArr;
  float* TimeArr;
  float CurrentAltitude;
  float InitialAltitude;
  float ReferenceAltitude;
  SpaceTime* SpaceTimeArr;
} ObjectPosition; 





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
double UtilDegToRad(double Deg);
double UtilRadToDeg(double Rad);
int UtilPopulateSpaceTimeArr(ObjectPosition *OP, char* TrajFile);
int UtilSortSpaceTimeAscending(ObjectPosition *OP);
int UtilFindCurrentTrajectoryPosition(ObjectPosition *OP, int StartIndex, float CurrentTime, float DistanceThreshold);
int UtilSetSyncPoint(ObjectPosition *OP, double x, double y, double z, double time);
float UtilCalculateTimeToSync(ObjectPosition *OP);

int UtilCountFileRows(FILE *fd);
int UtilReadLineCntSpecChars(FILE *fd, char *Buffer);
char UtilGetch();


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
