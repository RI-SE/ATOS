/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : util.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/

#include <stdio.h>
#include <dirent.h>
#include <math.h>
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>
#include <netinet/tcp.h>
#include <float.h>

#include "util.h"
#include "logging.h"
#include "maestroTime.h"


/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/

#define FE_WGS84        (1.0/298.257223563) // earth flattening (WGS84)
#define RE_WGS84        6378137.0           // earth semimajor axis (WGS84) (m)

// Message priorities on message queue
#define PRIO_COMM_STRT 100
#define PRIO_COMM_ARMD 110
#define PRIO_COMM_STOP 120
#define PRIO_COMM_MONI 80
#define PRIO_COMM_EXIT 140
#define PRIO_COMM_REPLAY 160
#define PRIO_COMM_CONTROL 180
#define PRIO_COMM_ABORT 60
#define PRIO_COMM_TOM 90
#define PRIO_COMM_INIT 110
#define PRIO_COMM_CONNECT 110
#define PRIO_COMM_OBC_STATE 160
#define PRIO_COMM_DISCONNECT 110
#define PRIO_COMM_LOG 160
#define PRIO_COMM_VIOP 80
#define PRIO_COMM_TRAJ 80
#define PRIO_COMM_TRAJ_TOSUP 80
#define PRIO_COMM_TRAJ_FROMSUP 80
#define PRIO_COMM_ASP 110
#define PRIO_COMM_OSEM 160

/*------------------------------------------------------------
-- Public variables
------------------------------------------------------------*/

//static int debug = DEBUG_LEVEL_HIGH;

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
-- Private functions
------------------------------------------------------------*/

static char rayFromPointIntersectsLine(double pointX, double pointY, double polyPointAX, double polyPointAY, double polyPointBX, double polyPointBY);

/*---------------------------------------------s---------------
  -- Public functions
  ------------------------------------------------------------*/

// GPS TIME FUNCTIONS
uint64_t UtilgetGPSmsFromUTCms(uint64_t UTCms)
{
    return UTCms - MS_TIME_DIFF_UTC_GPS + MS_LEAP_SEC_DIFF_UTC_GPS;
}
uint64_t UtilgetUTCmsFromGPSms(uint64_t GPSms)
{
    return GPSms + MS_TIME_DIFF_UTC_GPS - MS_LEAP_SEC_DIFF_UTC_GPS;
}

uint64_t UtilgetMSfromGPStime(uint16_t GPSweek,uint32_t GPSquarterMSofWeek)
{
    return (uint64_t)GPSweek * WEEK_TIME_MS + (uint64_t)(GPSquarterMSofWeek >> 2);
}

void UtilgetGPStimeFromMS(uint64_t GPSms, uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek)
{
    uint16_t tempGPSweek = (uint16_t)(GPSms / WEEK_TIME_MS);
    if (GPSweek) *GPSweek = tempGPSweek;
    uint64_t remainder = GPSms - (uint64_t)tempGPSweek * WEEK_TIME_MS;
    if (GPSquarterMSofWeek) *GPSquarterMSofWeek = (uint32_t)(remainder << 2);
/*
    uint16_t GPSday = remainder / DAY_TIME_MS;
    remainder -= (uint64_t)GPSday * DAY_TIME_MS;
    uint16_t GPShour = remainder / HOUR_TIME_MS;
    remainder -= (uint64_t)GPShour * HOUR_TIME_MS;
    uint16_t GPSminute = remainder / MINUTE_TIME_MS;
    remainder -= (uint64_t)GPSminute * HOUR_TIME_MS;

    qDebug() << "GPSWEEK:" << GPSweek <<
                "\nGPSSec" << GPSquarterMSofWeek;
    qDebug() << "GPSTIME: " << GPSweek <<
                ":" << GPSday <<
                ":" << GPShour <<
                ":" << GPSminute;

    qDebug() << "GPS WEEK: " << GPSweek <<
                "\nGPS DAY: " << GPSday <<
                "\nGPS HOUR:" << GPShour <<
                "\nGPS MINUTE:" << GPSminute;*/
}

void UtilgetGPStimeFromUTCms(uint64_t UTCms,uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek)
{
    UtilgetGPStimeFromMS(UtilgetGPSmsFromUTCms(UTCms),
                     GPSweek,
                     GPSquarterMSofWeek);
}
uint64_t UtilgetUTCmsFromGPStime(uint16_t GPSweek,uint32_t GPSquarterMSofWeek)
{
    return UtilgetUTCmsFromGPSms(UtilgetMSfromGPStime(GPSweek,GPSquarterMSofWeek));
}


void UtilgetCurrentGPStime(uint16_t *GPSweek, uint32_t *GPSquarterMSofWeek)
{
    UtilgetGPStimeFromUTCms(UtilgetCurrentUTCtimeMS(),GPSweek,GPSquarterMSofWeek);
}

uint64_t UtilgetCurrentUTCtimeMS()
{
    struct timeval CurrentTimeStruct;
    gettimeofday(&CurrentTimeStruct, 0);
    return (uint64_t)CurrentTimeStruct.tv_sec*1000 +
            (uint64_t)CurrentTimeStruct.tv_usec/1000;
}

uint32_t UtilgetIntDateFromMS(uint64_t ms)
{
    struct tm date_time;
    time_t seconds = (time_t)(ms / 1000);
    localtime_r(&seconds,&date_time);
    return  (uint32_t)((date_time.tm_year+1900)*10000 + (date_time.tm_mon+1)*100 + date_time.tm_mday);
}
uint64_t UtilgetETSIfromUTCMS(uint64_t utc_sec, uint64_t utc_usec)
{
  return  utc_sec*1000 + utc_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;

}

void UtilgetDateTimeFromUTCtime(int64_t utc_ms, char *buffer, int size_t)
{
    time_t time_seconds = utc_ms / 1000;
    if (size_t < 26) return;
    strcpy(buffer,ctime(&time_seconds));
}
void UtilgetDateTimefromUTCCSVformat(int64_t utc_ms, char *buffer, int size_t)
{
  struct tm date_time;
  char tmp_buffer_ms[10];
  int64_t ms;
  double tmp_ms;
  time_t time_seconds = (time_t) (utc_ms/1000);
  localtime_r(&time_seconds,&date_time);
  tmp_ms = (double) (utc_ms)/1000;
  tmp_ms = tmp_ms -utc_ms/1000;

  ms = round(tmp_ms*1000);
  strftime(buffer, size_t, "%Y;%m;%d;%H;%M;%S;",&date_time);
  sprintf(tmp_buffer_ms,"%" PRIi64,ms);
  strcat(buffer,tmp_buffer_ms);
}
void UtilgetDateTimeFromUTCForMapNameCreation(int64_t utc_ms, char *buffer, int size_t)
{
  struct tm date_time;
  char tmp_buffer_ms[10];
  int64_t ms;
  double tmp_ms;
  time_t time_seconds = (time_t) (utc_ms/1000);
  localtime_r(&time_seconds,&date_time);
  tmp_ms = (double) (utc_ms)/1000;
  tmp_ms = tmp_ms -utc_ms/1000;

  ms = round(tmp_ms*1000);
  strftime(buffer, size_t, "%Y-%m-%d_%H:%M:%S:",&date_time);
  sprintf(tmp_buffer_ms,"%" PRIi64,ms);
  strcat(buffer,tmp_buffer_ms);
}
void util_error(char* message)
{
  LogMessage(LOG_LEVEL_ERROR,message);
  exit(EXIT_FAILURE);
}

void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height)
{
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double r2 = x * x + y * y;
    double za = z;
    double zk = 0.0;
    double sinp = 0.0;
    double v = RE_WGS84;

    while (fabs(za - zk) >= 1E-4) {
        zk = za;
        sinp = za / sqrt(r2 + za * za);
        v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
        za = z + v * e2 * sinp;
    }

    *lat = (r2 > 1E-12 ? atan(za / sqrt(r2)) : (z > 0.0 ? M_PI / 2.0 : -M_PI / 2.0)) * 180.0 / M_PI;
    *lon = (r2 > 1E-12 ? atan2(y, x) : 0.0) * 180.0 / M_PI;
    *height = sqrt(r2 + za * za) - v;
}

void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z)
{
    double sinp = sin(lat * M_PI / 180.0);
    double cosp = cos(lat * M_PI / 180.0);
    double sinl = sin(lon * M_PI / 180.0);
    double cosl = cos(lon * M_PI / 180.0);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    *x = (v + height) * cosp * cosl;
    *y = (v + height) * cosp * sinl;
    *z = (v * (1.0 - e2) + height) * sinp;
}

void llhToEnu(const double *iLlh, const double *llh, double *xyz)
{
    double ix, iy, iz;
    llhToXyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

    double x, y, z;
    llhToXyz(llh[0], llh[1], llh[2], &x, &y, &z);

    double enuMat[9];
    createEnuMatrix(iLlh[0], iLlh[1], enuMat);

    double dx = x - ix;
    double dy = y - iy;
    double dz = z - iz;

    xyz[0] = enuMat[0] * dx + enuMat[1] * dy + enuMat[2] * dz;
    xyz[1] = enuMat[3] * dx + enuMat[4] * dy + enuMat[5] * dz;
    xyz[2] = enuMat[6] * dx + enuMat[7] * dy + enuMat[8] * dz;
}


void enuToLlh(const double *iLlh, const double *xyz, double *llh)
{
    double ix, iy, iz;
    llhToXyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

    double enuMat[9];
    createEnuMatrix(iLlh[0], iLlh[1], enuMat);

    double x = enuMat[0] * xyz[0] + enuMat[3] * xyz[1] + enuMat[6] * xyz[2] + ix;
    double y = enuMat[1] * xyz[0] + enuMat[4] * xyz[1] + enuMat[7] * xyz[2] + iy;
    double z = enuMat[2] * xyz[0] + enuMat[5] * xyz[1] + enuMat[8] * xyz[2] + iz;

    xyzToLlh(x, y, z, &llh[0], &llh[1], &llh[2]);
}

void createEnuMatrix(double lat, double lon, double *enuMat)
{
    double so = sin(lon * M_PI / 180.0);
    double co = cos(lon * M_PI / 180.0);
    double sa = sin(lat * M_PI / 180.0);
    double ca = cos(lat * M_PI / 180.0);

    // ENU
    enuMat[0] = -so;
    enuMat[1] = co;
    enuMat[2] = 0.0;

    enuMat[3] = -sa * co;
    enuMat[4] = -sa * so;
    enuMat[5] = ca;

    enuMat[6] = ca * co;
    enuMat[7] = ca * so;
    enuMat[8] = sa;
}



int UtilAddEightBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data)
{

  int i;
  for(i = 0; i < 8; i++)
  {
    *(MessageBuffer+StartIndex+i) = (char) (Data >> (7-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return (StartIndex+i);
}

int UtilAddSixBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned long Data)
{

  int i;
  for(i = 0; i < 6; i++)
  {
    *(MessageBuffer+StartIndex+i) = (char) (Data >> (5-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return (StartIndex+i);
}


int UtilAddFourBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned int Data)
{
  int i;
  for(i = 0; i < 4; i++)
  {
    *(MessageBuffer+StartIndex+i) = (unsigned char) (Data >> (3-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}

int UtilAddTwoBytesMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned short Data)
{
  int i;
  for(i = 0; i < 2; i++)
  {
    *(MessageBuffer+StartIndex+i) = (unsigned char) (Data >> (1-i)*8);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}

int UtilAddOneByteMessageData(unsigned char *MessageBuffer, int StartIndex, unsigned char Data)
{

  *(MessageBuffer+StartIndex) = Data;
  //printf("[%d]=%x\n", (StartIndex), *(MessageBuffer+StartIndex));
  return StartIndex+1;
}


int UtilAddNBytesMessageData(unsigned char *MessageBuffer, int StartIndex, int Length, unsigned char *Data)
{
  int i;
  for(i = 0; i < Length; i++)
  {
    *(MessageBuffer+StartIndex+i) = *(Data+i);
    //printf("[%d]=%x\n", (StartIndex+i), *(MessageBuffer+StartIndex+i));
  }

  return StartIndex+i;
}


int iUtilGetParaConfFile(char* pcParameter, char* pcValue)
{
  FILE *filefd;
  int iFindResult;
  char pcTemp[512];

  iFindResult = 0;

  filefd = fopen (TEST_CONF_FILE, "rb");

  if (filefd == NULL)
  {
    return 0;
  }

  while(fgets(pcTemp, 512, filefd) != NULL)
  {
    if((strstr(pcTemp, pcParameter)) != NULL)
    {

      /* Does contain any value? */
      if(strlen(pcTemp) > (strlen(pcParameter)+1))
      {
        /* replace new line */
        if(pcTemp[strlen(pcTemp)-1] == '\n')
        {
          pcTemp[strlen(pcTemp)-1] = 0;
        }
        strcpy(pcValue,&pcTemp[strlen(pcParameter)+1]);
      }
      iFindResult = 1;
    }
  }

  if(filefd)
  {
    fclose(filefd);
  }

  return 1;
}


int UtilSetAdaptiveSyncPoint(AdaptiveSyncPoint *ASP, FILE *filefd, char debug)
{

  char DataBuffer[MAX_FILE_PATH];
  char RowBuffer[MAX_FILE_PATH];
  char *ptr, *ptr1;
  bzero(DataBuffer,MAX_FILE_PATH);
  bzero(RowBuffer,MAX_FILE_PATH);

  bzero(RowBuffer, MAX_FILE_PATH);
  UtilReadLineCntSpecChars(filefd, RowBuffer);
  ptr = strchr(RowBuffer, ';');
  strncpy(DataBuffer, RowBuffer, (uint64_t)ptr - (uint64_t)RowBuffer);
  strncpy(ASP->MasterIP, DataBuffer, strlen(DataBuffer));
  //if(USE_TEST_HOST == 1) strncpy(ASP->MasterIP, TESTHOST_IP, sizeof(TESTHOST_IP));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(ASP->SlaveIP, DataBuffer, strlen(DataBuffer));
  //if(USE_TEST_HOST == 1) strncpy(ASP->SlaveIP, TESTHOST_IP, sizeof(TESTHOST_IP));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  ASP->MasterTrajSyncTime = atof(DataBuffer);

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  ASP->SlaveTrajSyncTime = atof(DataBuffer);

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  ASP->SlaveSyncStopTime = atof(DataBuffer);

  ASP->TestPort = 0;

  if(debug)
  {
    LogPrint("MasterIP: %s", ASP->MasterIP);
    LogPrint("SlaveIP: %s", ASP->SlaveIP);
    LogPrint("MasterTrajSyncTime %3.2f", ASP->MasterTrajSyncTime);
    LogPrint("SlaveTrajSyncTime %3.2f", ASP->SlaveTrajSyncTime);
    LogPrint("SlaveSyncStopTime %3.2f", ASP->SlaveSyncStopTime);
  }

  return 0;
}

int UtilSetTriggActions(TriggActionType *TAA, FILE *filefd, char debug)
{

  char DataBuffer[MAX_FILE_PATH];
  char RowBuffer[MAX_FILE_PATH];
  char *ptr, *ptr1, *ptr2;
  bzero(DataBuffer,MAX_FILE_PATH);
  bzero(RowBuffer,MAX_FILE_PATH);

  bzero(RowBuffer, MAX_FILE_PATH);
  UtilReadLineCntSpecChars(filefd, RowBuffer);

  ptr = strchr(RowBuffer, ';');
  strncpy(DataBuffer, RowBuffer, (uint64_t)ptr - (uint64_t)RowBuffer);
  strncpy(TAA->TriggerIP, DataBuffer, strlen(DataBuffer));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+1, ';');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  TAA->TriggerId = (uint8_t)atoi(DataBuffer);

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, '[');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(TAA->TriggerType, DataBuffer, strlen(DataBuffer));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ']');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(TAA->TriggerTypeVar, DataBuffer, strlen(DataBuffer));
  ptr = strchr(ptr, ';');

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, '[');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(TAA->ActionType, DataBuffer, strlen(DataBuffer));

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ']');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(TAA->ActionTypeVar, DataBuffer, strlen(DataBuffer));
  ptr = strchr(ptr, ';');

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, '[');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  if(strstr(DataBuffer,"SEND_START") != NULL) TAA->Action = TAA_ACTION_EXT_START;
  else if(strstr(DataBuffer,"TEST_SIGNAL") != NULL) TAA->Action = TAA_ACTION_TEST_SIGNAL;

  bzero(DataBuffer, strlen(DataBuffer));
  ptr1 = ptr+1;
  ptr = strchr(ptr+2, ']');
  strncpy(DataBuffer, ptr1, (uint64_t)ptr - (uint64_t)ptr1);
  strncpy(TAA->ActionDelay, DataBuffer, strlen(DataBuffer));


  if(debug)
  {
    LogPrint("TriggerIP: %s", TAA->TriggerIP);
    LogPrint("TriggerId: %d", TAA->TriggerId);
    LogPrint("TriggerType: %s", TAA->TriggerType);
    LogPrint("TriggerTypeVar: %s", TAA->TriggerTypeVar);
    LogPrint("ActionType: %s", TAA->ActionType);
    LogPrint("ActionTypeVar: %s", TAA->ActionTypeVar);
    LogPrint("Action: %d", TAA->Action);
    LogPrint("ActionDelay: %s", TAA->ActionDelay);
  }

  return 0;
}



void UtilSetObjectPositionIP(ObjectPosition *OP, char *IP) { strncpy(OP->IP, IP, strlen(IP));}

int UtilSetMasterObject(ObjectPosition *OP, char *Filename, char debug)
{

  FILE *filefd;
  char FilenameBuffer[MAX_FILE_PATH];
  char DataBuffer[20];
  double Time;
  bzero(FilenameBuffer,MAX_FILE_PATH);
  bzero(DataBuffer,20);
  strcat(FilenameBuffer,Filename);
  strcat(FilenameBuffer, MASTER_FILE_EXTENSION);
  filefd = fopen (FilenameBuffer, "r");

  if(filefd)
  {
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->Type = 'm';
    OP->SyncTime = Time;
    OP->SyncStopTime = 0;
    fclose(filefd);

    if(debug) LogPrint("Master object set: %s, SyncTime: %3.4f", FilenameBuffer, Time);

  } else {
    OP->Type = 'u';
    OP->SyncTime = 0;
  }


  return 0;
}


int UtilSetSlaveObject(ObjectPosition *OP, char *Filename, char debug)
{
  FILE *filefd;
  char FilenameBuffer[MAX_FILE_PATH];
  char DataBuffer[20];
  double Time;
  bzero(FilenameBuffer,MAX_FILE_PATH);
  bzero(DataBuffer,20);
  strcat(FilenameBuffer,Filename);
  strcat(FilenameBuffer, SLAVE_FILE_EXTENSION);
  filefd = fopen (FilenameBuffer, "r");

  if(filefd)
  {
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->Type = 's';
    OP->SyncTime = Time;
    UtilReadLineCntSpecChars(filefd, DataBuffer);
    Time = atof(DataBuffer);
    OP->SyncStopTime = Time;
    fclose(filefd);
    if(debug) LogPrint("Slave object set: %s, SyncTime: %3.4f", FilenameBuffer, Time);
  }


  return 0;
}


/*!
 * \brief UtilIsPositionNearTarget Checks if position lies within or on a sphere with radius equal to tolerance_m
 * and centre at target.
 * \param position Position to verify
 * \param target Target position
 * \param tolerance_m Radius around target position defining "near"
 * \return true if position is within tolerance_m of target, false otherwise
 */
char UtilIsPositionNearTarget(CartesianPosition position, CartesianPosition target, double tolerance_m)
{
    double distance = 0;
    distance = sqrt( pow(position.xCoord_m-target.xCoord_m, 2)
                   + pow(position.yCoord_m-target.yCoord_m, 2)
                   + pow(position.zCoord_m-target.zCoord_m, 2) );
    return distance <= tolerance_m;
}

double UtilCalcPositionDelta(double P1Lat, double P1Long, double P2Lat, double P2Long, ObjectPosition *OP)
{

	double f, d, P1LatRad, P1LongRad, P2LatRad, P2LongRad, U1, U2, L, lambda, sins, coss, sigma, sinalpha, cosalpha2, cossm, C, lambdaprim, u2, A, B, dsigma, s, alpha1, alpha2, cosU2, sinlambda, cosU1, sinU1, sinU2, coslambda;

  double iLlh[3] = { P1Lat , P1Long , 202.934115075};
  double xyz[3] = {0, 0, 0};
  double Llh[3] = {P2Lat, P2Long, 202.934115075};

 	OP->Latitude = P2Lat;
	OP->Longitude = P2Long;
	P1LatRad = UtilDegToRad(P1Lat);
	P1LongRad = UtilDegToRad(P1Long);
	P2LatRad = UtilDegToRad(P2Lat);
	P2LongRad = UtilDegToRad(P2Long);

	f = 1/k;
	U1 = atan((1-f)*tan(P1LatRad));
	U2 = atan((1-f)*tan(P2LatRad));
	L = P2LongRad - P1LongRad;
	lambda = L;
	lambdaprim = lambda;
	//printf("Lambdadiff: %1.15f\n", fabs(lambda-lambdaprim));

	int i = ORIGO_DISTANCE_CALC_ITERATIONS;
	OP->CalcIterations = 0;
	/*
  do
	{
		sins = sqrt( pow((cos(U2)*sin(lambda)),2) + pow((cos(U1)*sin(U2) - sin(U1)*cos(U2)*cos(lambda)),2) );
		if (sins==0) return 0; //co-incident points
		coss = sin(U1)*sin(U2) + cos(U1)*cos(U2)*cos(lambda);
		sigma = atan(sins/coss);
		sinalpha = (cos(U1)*cos(U2)*sin(lambda))/sins;
		cosalpha2 = 1 - pow(sinalpha,2);
		cossm = coss - (2*sin(U1)*sin(U2) / cosalpha2);
		C = (f/16) * cosalpha2 * ( 4 + f*(4 - 3 * cosalpha2) );
		lambdaprim = lambda;
		lambda = L + (1-C)*f*sinalpha*(sigma+C*sins*(cossm + C*coss*(-1+ 2*pow(cossm,2))));
		OP->CalcIterations ++;
		//printf("Lambdadiff: %1.15f\n", fabs(lambda-lambdaprim));

	} while(fabs(lambda-lambdaprim) > l  && --i > 0);

	if (i == 0)
	{
		//printf("Failed to converge!\n");
		OP->OrigoDistance = -1;
	}
	else
	{*/
		//u2 = cosalpha2*((pow(a,2) - pow(b,2))/pow(b,2));
		//A = 1 +(u2/16384)*(4096+u2*(-768+u2*(320-175*u2)));
		//B = (u2/1024)*(256 + u2*(-128*u2*(74-47*u2)));
		//dsigma = B*sins*(cossm+0.25*B*(coss*(-1+2*pow(cossm,2)) - (1/6)*B*cossm*(-3+4*pow(sins,2))*(-3+4*pow(cossm,2))));
		//s = b*A*(sigma-dsigma);
    s = sqrt(pow(OP->x,2) + pow(OP->y,2));
    OP->DeltaOrigoDistance = s - OP->OrigoDistance;
		OP->OrigoDistance = s;


		/*
    cosU2 = cos(U2);
		sinU2 = sin(U2);
		cosU1 = cos(U1);
		sinU1 = sin(U1);
		sinlambda = sin(lambda);
		coslambda = cos(lambda);
    */

    //OP->ForwardAzimuth1 = atan2(cosU2*sinlambda,(cosU1*sinU2-sinU1*cosU2*coslambda));
		//OP->ForwardAzimuth2 = atan2(cosU1*sinlambda,(sinU1*cosU2*-1+cosU1*sinU2*coslambda));

    //llhToEnu(iLlh, Llh, xyz);


    //OP->x = xyz[0];
    //OP->y = xyz[1];


	//}
	return s;
}

int UtilVincentyDirect(double refLat, double refLon, double a1, double distance, double *resLat, double *resLon, double *a2)
{
    /* Algorithm based on 07032018 website https://en.wikipedia.org/wiki/Vincenty%27s_formulae */


    // Variables only calculated once
    double U1, f = 1/k, sigma1, sina, pow2cosa, pow2u, A, B, C, L, lambda;
    // Iterative variables
    double sigma, deltaSigma, sigma2m;
    // Temporary iterative variables
    double prev_sigma;

    U1 = atan((1-f)*tan(UtilDegToRad(refLat)));
    sigma1 = atan2(tan(U1),cos(a1));

    sina = cos(U1)*sin(a1);

    pow2cosa = 1-pow(sina,2);
    pow2u = pow2cosa * (pow(a,2)-pow(b,2))/pow(b,2);

    A = 1 + pow2u/16384.0*(4096.0 + pow2u*(-768.0 + pow2u*(320.0-175.0*pow2u)));
    B = pow2u / 1024.0 * (256.0 + pow2u*(-128.0 + pow2u*(74.0 - 47.0*pow2u)));


    int iterations = 0;
    double init_sigma = distance / (b*A);
    sigma = init_sigma;
    do
    {
        if (++iterations > 100) {
            return -1;
        }
        prev_sigma = sigma;

        sigma2m = 2*sigma1 + sigma;
        deltaSigma = B*sin(sigma) * (
                    cos(sigma2m) + 0.25 * B * (
                        cos(sigma) * (-1.0 + 2.0*pow(cos(sigma2m),2)) -
                        B/6*cos(sigma2m) * (-3.0 + 4.0*pow(sin(sigma),2.0)) * (-3.0 + 4.0*pow(cos(sigma2m),2))
                        )
                    );
        sigma = init_sigma + deltaSigma;
    } while(fabs(sigma-prev_sigma) > l);


    *resLat = UtilRadToDeg(atan2(
                sin(U1) * cos(sigma) + cos(U1) * sin(sigma) * cos(a1),
                (1-f)*sqrt(pow(sina,2) + pow(sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(a1),2))
                ));

    lambda = atan2(
                sin(sigma) * sin(a1),
                cos(U1)*cos(sigma) - sin(U1)*sin(sigma)*cos(a1)
                );

    C = f/16*pow2cosa*(4 + f*(4-3*pow2cosa));

    L = lambda - (1 - C)*f*sina*(
                sigma + C*sin(sigma)*(
                    cos(sigma2m) + C*cos(sigma)*(-1 + 2*pow(cos(sigma2m),2))
                    )
                );

    *resLon = UtilRadToDeg(L) + refLon;

    *a2 = atan2(
                sina,
                -sin(U1)*sin(sigma) + cos(U1)*cos(sigma)*cos(a1)
                );

    return 0;
}

double UtilDegToRad(double Deg){return (PI*Deg/180);}
double UtilRadToDeg(double Rad){return (180*Rad/PI);}


int UtilPopulateSpaceTimeArr(ObjectPosition *OP, char* TrajFile)
{

  FILE *Trajfd;
  int Rows, j = 0;
  char TrajRow[TRAJECTORY_LINE_LENGTH];

  Trajfd = fopen (TrajFile, "r");
  if(Trajfd)
  {
    Rows = OP->TrajectoryPositionCount;
    //printf("Rows = %d\n", Rows);
    double x, y, z;
    float t;
    char ValueStr[NUMBER_CHAR_LENGTH];
    char* src1; char* src2;
    do
    {
      bzero(TrajRow,TRAJECTORY_LINE_LENGTH);
      if( UtilReadLineCntSpecChars(Trajfd, TrajRow) >= 10)
      {
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(TrajRow, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        t = atof(ValueStr);
        //printf("%d :t = %3.3f\n", j, t);
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        x = atof(ValueStr);

        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        y = atof(ValueStr);

        /*
        bzero(ValueStr, NUMBER_CHAR_LENGTH);
        src1 = strchr(src2, ';');
        src2 = strchr(src1+1, ';');
        strncpy(ValueStr, src1+1, (uint64_t)src2 - (uint64_t)src1);
        z = atof(ValueStr);
        */

        OP->SpaceArr[j]= (float)sqrt(pow(x,2) + pow(y,2));
        OP->TimeArr[j]= (float)t;

        OP->SpaceTimeArr[j].Index = j;
        OP->SpaceTimeArr[j].Time = (float)t;
        OP->SpaceTimeArr[j].OrigoDistance = (float)sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        OP->SpaceTimeArr[j].Bearing = tan(y/x);
        OP->SpaceTimeArr[j].x = x;
        OP->SpaceTimeArr[j].y = y;
        //printf("t = %5.3f\n", OP->TimeArr[j]);
        //printf("t = %5.3f\n", OP->TimeArr[j]);
        j ++;
      }


    } while (--Rows >= 0 /*j < 10*/);

    //UtilSortSpaceTimeAscending(OP);

    //for(int g=0; g < OP->TrajectoryPositionCount; g ++)
    //{
    //  printf("OrigoDistance=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceTimeArr[g].OrigoDistance, OP->SpaceTimeArr[g].Time, OP->SpaceTimeArr[g].Index);
    //}


    fclose(Trajfd);
  }
  else
  {
    LogMessage(LOG_LEVEL_ERROR,"Failed to open file:%s",  TrajFile);
  }

  return 0;
}

int UtilSetSyncPoint(ObjectPosition *OP, double x, double y, double z, double Time)
{

  int i = 0;
  int Gate1Reached = 0, Gate2Reached = 0;

  if(Time == 0)
  {
    float R = (float)sqrt(pow(x,2) + pow(y,2));
    while(i < (OP->TrajectoryPositionCount-1) && Gate1Reached == 0)
    {
        if( OP->SpaceArr[i] == R)
        {
          Gate1Reached = 1;
          OP->SyncIndex = i;
          //printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceArr[i], OP->TimeArr[i], i);
         }
        else
        {
          OP->SyncIndex = -1;
        }
        i ++;
    }
  } else {

    while(i < (OP->TrajectoryPositionCount-1) && Gate1Reached == 0)
    {
        //printf("%4.3f, %4.3f\n", OP->TimeArr[i], OP->TimeArr[i+1]);

        if(Time >= OP->TimeArr[i]  &&  Time <= OP->TimeArr[i+1] )
        {
          Gate1Reached = 1;
          OP->SyncIndex = i;
         // printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP->SpaceArr[i], OP->TimeArr[i], i);
         }
        else
        {
          OP->SyncIndex = -1;
        }
        i ++;
    }

  }

}

float UtilCalculateTimeToSync(ObjectPosition *OP)
{

  float t = OP->SpaceTimeArr[OP->SyncIndex].Time - OP->SpaceTimeArr[OP->BestFoundTrajectoryIndex].Time;
  OP->TimeToSyncPoint = t;
  //printf("%3.3f %d %d\n", t , OP->SyncIndex, OP->BestFoundTrajectoryIndex);
  return t;
}




/*!
 * \brief UtilIsPointInPolygon Checks if a point lies within a polygon specified by a number of vertices using
 * the ray casting algorithm (see http://rosettacode.org/wiki/Ray-casting_algorithm).
 * \param pointX X coordinate of the point to check
 * \param pointY Y coordinate of the point to check
 * \param verticesX X coordinates of the vertex points defining the polygon to check
 * \param verticesY Y coordinates of the vertex points defining the polygon to check
 * \param nPtsInPolygon length of the two vertex arrays
 * \return true if the point lies within the polygon, false otherwise
 */
char UtilIsPointInPolygon(double pointX, double pointY, double * verticesX, double * verticesY, int nPtsInPolygon)
{
    int nIntersects = 0;

    // Count the number of intersections with the polygon
    for (int i = 0; i < nPtsInPolygon-1; ++i)
    {
        if (rayFromPointIntersectsLine(pointX, pointY, verticesX[i], verticesY[i], verticesX[i+1], verticesY[i+1]))
        {
            nIntersects++;
        }
    }

    // If the first and last points are different, the polygon segment between them must also be included
    if (fabs(verticesX[0] - verticesX[nPtsInPolygon-1]) > (double)(2*FLT_EPSILON) || fabs(verticesY[0] - verticesY[nPtsInPolygon-1]) > (double)(2*FLT_EPSILON) )
    {
        if (rayFromPointIntersectsLine(pointX, pointY,
                  verticesX[0], verticesY[0], verticesX[nPtsInPolygon-1], verticesY[nPtsInPolygon-1]))
        {
            nIntersects++;
        }
    }

    // If the number of ray intersections with the polygon is even, the point must be outside the polygon
    if (nIntersects % 2 == 0)
        return 0;
    else
        return 1;
}

/*!
 * \brief rayFromPointIntersectsLine Checks whether a ray in positive x direction, parallel to the x axis,
 * from a point intersects a line AB between two points A and B
 * \param pointX X coordinate of the point from which ray originates
 * \param pointY Y coordinate of the point from which ray originates
 * \param linePointAX X coordinate of the point A
 * \param linePointAY Y coordinate of the point A
 * \param linePointBX X coordinate of the point B
 * \param linePointBY Y coordinate of the point B
 * \return true if ray intersects line AB, false otherwise
 */
static char rayFromPointIntersectsLine(double pointX, double pointY,
           double linePointAX, double linePointAY, double linePointBX, double linePointBY)
{
    double tmp, maxVal, minVal, slopeAB, slopeAP;

    // Need 2x eps because machine epsilon is too small
    // Need to use float epsilon in case the double points are just promoted floats
    const double algorithmEpsilon = (double)(2*FLT_EPSILON);

    // The algorithm assumes the Y coordinate of A is smaller than that of B
    // -> if this is not the case, swap the points
    if (linePointBY < linePointAY)
    {
        tmp = linePointAX;
        linePointAX = linePointBX;
        linePointBX = tmp;

        tmp = linePointAY;
        linePointAY = linePointBY;
        linePointBY = tmp;
    }

    // If the ray casting would cause intersection with a vertex the result is undefined,
    // so we perform a heuristic to avoid this situation:
    if (pointY == linePointAY || pointY == linePointBY)
    {
        pointY += algorithmEpsilon;
    }

    // A ray in X axis direction will not intersect if Y coordinates are outside of the range
    // spanned by the line AB
    if (pointY < linePointAY || pointY > linePointBY)
        return 0;

    // A ray in X axis direction will not intersect if it starts at a larger X coordinate than
    // any in the range of the line AB
    maxVal = linePointAX > linePointBX ? linePointAX : linePointBX;
    if (pointX >= maxVal)
        return 0;

    // A ray in X axis direction will intersect if it starts at a smaller X coordinate than
    // any in the range of the line AB
    minVal = linePointAX < linePointBX ? linePointAX : linePointBX;
    if (pointX < minVal)
        return 1;

    // Having filtered away all the situations where the point lies outside the rectangle
    // defined by the two points A and B, it is necessary to do some more costly processing

    // Compare the slopes of the lines AB and AP - if the slope of AB is larger than that of AP
    // then a ray in X axis direction will not intersect AB
    if (linePointAX != linePointBX)
        slopeAB = (linePointBY - linePointAY) / (linePointBX - linePointAX);
    else
        slopeAB = (double)INFINITY;
    if (linePointAX != pointX)
        slopeAP = (pointY - linePointAY) / (pointX - linePointAX);
    else
        slopeAP = (double)INFINITY;

    if (slopeAP >= slopeAB)
        return 1;
    else
        return 0;
}


int UtilSortSpaceTimeAscending(ObjectPosition *OP)
{
    int i, j, index;
    float r,t;
    for (i = 0; i < OP->TrajectoryPositionCount; ++i)
    {
        for (j = i + 1; j < OP->TrajectoryPositionCount; ++j)
        {
            if (OP->SpaceTimeArr[i].OrigoDistance > OP->SpaceTimeArr[j].OrigoDistance)
            {
                index = OP->SpaceTimeArr[i].Index;
                r = OP->SpaceTimeArr[i].OrigoDistance;
                t = OP->SpaceTimeArr[i].Time;
                OP->SpaceTimeArr[i].Index = OP->SpaceTimeArr[j].Index;
                OP->SpaceTimeArr[i].OrigoDistance = OP->SpaceTimeArr[j].OrigoDistance;
                OP->SpaceTimeArr[i].Time = OP->SpaceTimeArr[j].Time;
                OP->SpaceTimeArr[j].Index = index;
                OP->SpaceTimeArr[j].OrigoDistance = r;
                OP->SpaceTimeArr[j].Time = t;
            }
        }
    }
}


int UtilFindCurrentTrajectoryPosition(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug)
{

  int i = StartIndex, j=0, ErrorDecreasing = 1, PositionFound=-1, Init, Q1, Q2;
  double Angle1, Angle2, R1, R2, RDiff, AngleDiff, PrevAngleDiff;

  if(i <= -1) i = 0;
  //OP->BestFoundTrajectoryIndex = 0;
  if(debug) LogPrint("OPOrigoDistance=%4.3f, x=%4.3f, y=%4.3f, SyncIndex=%d", OP->OrigoDistance, OP->x, OP->y, OP->SyncIndex);

  Init = 1;
  while(i < (OP->TrajectoryPositionCount-1) && i <= OP->SyncIndex)
  {

    Angle1 = PI/2 - atan(fabs(OP->SpaceTimeArr[i].y)/fabs(OP->SpaceTimeArr[i].x));
    Q1 = 0;
    if(OP->SpaceTimeArr[i].y >= 0 && OP->SpaceTimeArr[i].x >= 0) Q1 = 1;
    else if(OP->SpaceTimeArr[i].y > 0 && OP->SpaceTimeArr[i].x < 0) Q1 = 2;
    else if(OP->SpaceTimeArr[i].y < 0 && OP->SpaceTimeArr[i].x < 0) Q1 = 3;
    else if(OP->SpaceTimeArr[i].y < 0 && OP->SpaceTimeArr[i].x > 0) Q1 = 4;

    Angle2 = PI/2 - atan(fabs(OP->y)/fabs(OP->x));
    Q2 = 0;
    if(OP->y >= 0 && OP->x >= 0) Q2 = 1;
    else if(OP->y > 0 && OP->x < 0) Q2 = 2;
    else if(OP->y < 0 && OP->x < 0) Q2 = 3;
    else if(OP->y < 0 && OP->x > 0) Q2 = 4;

    if(debug == 2)
    {
      R1 = sqrt(pow(OP->SpaceTimeArr[i].x,2)+pow(OP->SpaceTimeArr[i].y,2));
      R2 = sqrt(pow(OP->x,2) + pow(OP->y,2));
      LogPrint("%d, %3.5f, %3.5f, %3.5f, %d, %d, %3.6f", i, fabs(R1-R2), fabs(R1-OP->OrigoDistance) ,fabs(Angle1-Angle2), Q1, Q2, fabs(Angle1 - OP->ForwardAzimuth1));
    }


    if(Q1 == Q2)
    {
      R1 = sqrt(pow(OP->SpaceTimeArr[i].x,2)+pow(OP->SpaceTimeArr[i].y,2));
      R2 = sqrt(pow(OP->x,2) + pow(OP->y,2));
      //RDiff = fabs(R1-R2);
      RDiff = fabs(R1-OP->OrigoDistance);
      AngleDiff = fabs(Angle1 - Angle2);
      if(Init == 0)
      {
        if((AngleDiff < PrevAngleDiff) && (i > OP->BestFoundTrajectoryIndex) && RDiff <= MaxTrajDiff)
        {
            PositionFound = i;
            if(debug == 2) LogPrint("Minimum: %d, %3.6f, %3.6f", i, AngleDiff, RDiff);
            PrevAngleDiff = AngleDiff;
        }
      }
      else
      {
       PrevAngleDiff = AngleDiff;
      }
      Init = 0;
    }
    i ++;
  }

  if(debug) LogPrint("Selected time: %3.3f", OP->SpaceTimeArr[PositionFound].Time);

  if(PositionFound == -1)  OP->BestFoundTrajectoryIndex = TRAJ_POSITION_NOT_FOUND;
  else if(PositionFound > TRAJ_POSITION_NOT_FOUND)
  {
    OP->BestFoundTrajectoryIndex = PositionFound;
    OP->SpaceTimeFoundIndex = PositionFound;
  }

  if(debug == 2)
  {
    LogPrint("BestFoundTrajectoryIndex=%d", OP->BestFoundTrajectoryIndex);
    LogPrint("Current origo distance=%4.3f m", OP->OrigoDistance);
    LogPrint("Current time=%4.3f s", CurrentTime);
    LogPrint("Matched origo distance=%4.3f m", OP->SpaceTimeArr[PositionFound].OrigoDistance);
    LogPrint("Distance error=%4.3f m", OP->OrigoDistance - OP->SpaceTimeArr[PositionFound].OrigoDistance);
    LogPrint("Expected time=%4.3f s (index=%d)", OP->SpaceTimeArr[PositionFound].Time, OP->SpaceTimeArr[PositionFound].Index);
    LogPrint("Time error=%4.3f s", CurrentTime - OP->SpaceTimeArr[PositionFound].Time);
  }

  return PositionFound;
}


int UtilFindCurrentTrajectoryPositionNew(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug)
{

  int i = StartIndex, j=0, ErrorDecreasing = 1, PositionFound=-1, Init, Q1, Q2;
  int Gate1Reached = 0, Gate2Reached = 0, SampledSpaceIndex[SYNC_POINT_BUFFER];
  double Diff, PrevDiff, FutDiff, MinDiff=-1, BearingDiff, MinBearingDiff, Angle1, Angle2, R1, R2, RDiff, AngleDiff, PrevRDiff, PrevAngleDiff;

  if(i <= -1) i = 0;
  OP->BestFoundTrajectoryIndex = 0;
  LogMessage(LOG_LEVEL_DEBUG,"OPOrigoDistance=%4.3f, x=%4.3f, y=%4.3f, SyncIndex=%d", OP->OrigoDistance, OP->x, OP->y, OP->SyncIndex);

  Init = 1;
  while(i < (OP->TrajectoryPositionCount-1) && i <= OP->SyncIndex)
  {

    PrevDiff = (fabs(OP->SpaceTimeArr[i-2].OrigoDistance - OP->OrigoDistance));
    Diff = (fabs(OP->SpaceTimeArr[i].OrigoDistance - OP->OrigoDistance));
    FutDiff = (fabs(OP->SpaceTimeArr[i+2].OrigoDistance - OP->OrigoDistance));
    BearingDiff = fabs(OP->SpaceTimeArr[i].Bearing - OP->ForwardAzimuth2);

    Angle1 = PI/2 - atan(fabs(OP->SpaceTimeArr[i].y)/fabs(OP->SpaceTimeArr[i].x));
    Q1 = 0;
    if(OP->SpaceTimeArr[i].y >= 0 && OP->SpaceTimeArr[i].x >= 0) Q1 = 1;
    else if(OP->SpaceTimeArr[i].y > 0 && OP->SpaceTimeArr[i].x < 0) Q1 = 2;
    else if(OP->SpaceTimeArr[i].y < 0 && OP->SpaceTimeArr[i].x < 0) Q1 = 3;
    else if(OP->SpaceTimeArr[i].y < 0 && OP->SpaceTimeArr[i].x > 0) Q1 = 4;

    Angle2 = PI/2 - atan(fabs(OP->y)/fabs(OP->x));
    Q2 = 0;
    if(OP->y >= 0 && OP->x >= 0) Q2 = 1;
    else if(OP->y > 0 && OP->x < 0) Q2 = 2;
    else if(OP->y < 0 && OP->x < 0) Q2 = 3;
    else if(OP->y < 0 && OP->x > 0) Q2 = 4;

    R1 = sqrt(pow(OP->SpaceTimeArr[i].x,2)+pow(OP->SpaceTimeArr[i].y,2));
    R2 = sqrt(pow(OP->x,2) + pow(OP->y,2));
    if(debug == 2) LogPrint("%d, %3.5f, %3.5f, %3.5f, %d, %d, %3.6f", i, fabs(R1-R2), fabs(R1-OP->OrigoDistance) ,fabs(Angle1-Angle2), Q1, Q2, fabs(Angle1 - OP->ForwardAzimuth1));

    if(Q1 == Q2)
    {
      R1 = sqrt(pow(OP->SpaceTimeArr[i].x,2)+pow(OP->SpaceTimeArr[i].y,2));
      R2 = sqrt(pow(OP->x,2) + pow(OP->y,2));
      RDiff = fabs(R1-R2);
      AngleDiff = fabs(Angle1 - Angle2);
      if(Init == 0)
      {
        if((AngleDiff < PrevAngleDiff) && (i > OP->BestFoundTrajectoryIndex))
        {
            PositionFound = i;
            //SampledSpaceIndex[j] = i;
            //j++ ;
            if(debug == 2) LogPrint("Minimum: %d, %3.6f, %3.6f", i, AngleDiff, RDiff);
            PrevAngleDiff = AngleDiff;
        }

      }
      else
      {
       PrevAngleDiff = AngleDiff;
      }
      Init = 0;
    }
    i ++;
  }

  if(debug) LogPrint("Selected time: %3.3f", OP->SpaceTimeArr[PositionFound].Time);

  if(PositionFound == -1)  OP->BestFoundTrajectoryIndex = TRAJ_POSITION_NOT_FOUND;
  else if(PositionFound > TRAJ_POSITION_NOT_FOUND)
  {
    OP->BestFoundTrajectoryIndex = PositionFound;
    OP->SpaceTimeFoundIndex = PositionFound;

    /*
    printf("BestFoundTrajectoryIndex=%d\n", OP->BestFoundTrajectoryIndex);
    printf("Current origo distance=%4.3f m\n", OP->OrigoDistance);
    printf("Current time=%4.3f s\n", CurrentTime);
    printf("Matched origo distance=%4.3f m\n", OP->SpaceTimeArr[PositionFound].OrigoDistance);
    printf("Distance error=%4.3f m\n", OP->OrigoDistance - OP->SpaceTimeArr[PositionFound].OrigoDistance);
    printf("Expected time=%4.3f s (index=%d)\n", OP->SpaceTimeArr[PositionFound].Time, OP->SpaceTimeArr[PositionFound].Index);
    printf("Time error=%4.3f s\n", CurrentTime - OP->SpaceTimeArr[PositionFound].Time);
    */
  }

  return PositionFound;
}



int UtilFindCurrentTrajectoryPositionPrev(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug)
{

  int i = StartIndex, j=0, ErrorDecreasing = 1, PositionFound=-1, Init;
  int Gate1Reached = 0, Gate2Reached = 0, SampledSpaceIndex[SYNC_POINT_BUFFER];
  double Diff, PrevDiff, FutDiff, MinDiff=-1, BearingDiff, MinBearingDiff;

  if(i <= -1) i = 2;
  OP->BestFoundTrajectoryIndex = 0;
  LogMessage(LOG_LEVEL_DEBUG,"OPOrigoDistance=%4.3f, x=%4.3f, y=%4.3f, SyncIndex=%d", OP->OrigoDistance, OP->x, OP->y, OP->SyncIndex);

  Init = 1;
  while(i < (OP->TrajectoryPositionCount-1) && i <= OP->SyncIndex)
  {

    PrevDiff = (fabs(OP->SpaceTimeArr[i-2].OrigoDistance - OP->OrigoDistance));
    Diff = (fabs(OP->SpaceTimeArr[i].OrigoDistance - OP->OrigoDistance));
    FutDiff = (fabs(OP->SpaceTimeArr[i+2].OrigoDistance - OP->OrigoDistance));
    BearingDiff = fabs(OP->SpaceTimeArr[i].Bearing - OP->ForwardAzimuth2);

    if(debug == 2) LogPrint("%d, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.6f", i, PrevDiff,Diff, FutDiff, OP->SpaceTimeArr[i].x,  OP->x, OP->SpaceTimeArr[i].y, OP->y, fabs(OP->SpaceTimeArr[i].Bearing - tan(OP->y/OP->x)));

    if(Init == 0)
    {
      if((Diff <= PrevDiff) && (Diff <= FutDiff) && (i > OP->BestFoundTrajectoryIndex))
      {
          MinDiff = Diff;
          MinBearingDiff = BearingDiff;
          PositionFound = i;
          SampledSpaceIndex[j] = i;
          j++ ;
          if(debug == 1) LogPrint("Minimum: %d, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.6f", i, PrevDiff,Diff, FutDiff, OP->SpaceTimeArr[i].x,  OP->x, OP->SpaceTimeArr[i].y, OP->y, fabs(OP->SpaceTimeArr[i].Bearing - tan(OP->y/OP->x)));
      }
    }

    Init = 0;
    i ++;
  }

  Init = 1;
  PositionFound = -1;
  for(i = 0; i < j; i++)
  {
    Diff = fabs(CurrentTime - OP->SpaceTimeArr[SampledSpaceIndex[i]].Time);
    if(Init == 0)
    {
      if(Diff < MinDiff)
      {
        PositionFound = SampledSpaceIndex[i];
      }
    }
    Init = 0;
    MinDiff = Diff;
  }

  if(debug) LogPrint("Selected time: %3.3f", OP->SpaceTimeArr[PositionFound].Time);

  if(PositionFound == -1)  OP->BestFoundTrajectoryIndex = TRAJ_POSITION_NOT_FOUND;
  else if(PositionFound > TRAJ_POSITION_NOT_FOUND)
  {
    OP->BestFoundTrajectoryIndex = PositionFound;
    OP->SpaceTimeFoundIndex = PositionFound;

    /*
    printf("BestFoundTrajectoryIndex=%d\n", OP->BestFoundTrajectoryIndex);
    printf("Current origo distance=%4.3f m\n", OP->OrigoDistance);
    printf("Current time=%4.3f s\n", CurrentTime);
    printf("Matched origo distance=%4.3f m\n", OP->SpaceTimeArr[PositionFound].OrigoDistance);
    printf("Distance error=%4.3f m\n", OP->OrigoDistance - OP->SpaceTimeArr[PositionFound].OrigoDistance);
    printf("Expected time=%4.3f s (index=%d)\n", OP->SpaceTimeArr[PositionFound].Time, OP->SpaceTimeArr[PositionFound].Index);
    printf("Time error=%4.3f s\n", CurrentTime - OP->SpaceTimeArr[PositionFound].Time);
    */
  }

  return PositionFound;
}



//TODO THIS IS UNUSED - DELETE?
int UtilFindCurrentTrajectoryPositionOld(ObjectPosition *OP, int StartIndex, double CurrentTime, double MaxTrajDiff, double MaxTimeDiff, char debug)
{

  int i = StartIndex, j=0;
  int Gate1Reached = 0, Gate2Reached = 0, SampledSpaceIndex[SYNC_POINT_BUFFER];
  double Diff;

  if(i <= -1) i = 0;
  OP->BestFoundTrajectoryIndex = 0;
  if(debug) printf("OPOrigoDistance=%4.3f, x=%4.3f, y=%4.3f\n", OP->OrigoDistance, OP->x, OP->y);

  while(i < (OP->TrajectoryPositionCount-1) && Gate2Reached == 0)
  {

      Diff = fabs(OP->SpaceTimeArr[i].OrigoDistance - OP->OrigoDistance);
     //printf("%4.3f, %4.3f, %4.3f\n ", Diff,OP->SpaceTimeArr[i].OrigoDistance,OP->OrigoDistance);

      if( Diff < MaxTrajDiff  && Gate1Reached == 0)
      {
        Gate1Reached = 1;
      }

      if(Diff > MaxTrajDiff  && Gate1Reached == 1)
      {
        Gate2Reached = 1;
      }

      if(Gate1Reached == 1)
      {
          if(j < SYNC_POINT_BUFFER-1 && OP->SpaceTimeArr[i].Index <= OP->SyncIndex && OP->SpaceTimeArr[i].Index > OP->BestFoundTrajectoryIndex)
          {
            SampledSpaceIndex[j] = i;
            j++;
            if(debug) printf("i=%d, j=%d ,ArrOrigoDistance=%4.3f, Diff=%4.3f, ArrTime=%4.3f, Index=%d, CurrentTime=%4.3f \n",i , j, OP->SpaceTimeArr[i].OrigoDistance, Diff, OP->SpaceTimeArr[i].Time, OP->SpaceTimeArr[i].Index, CurrentTime);
          }
          else if(j >= SYNC_POINT_BUFFER)
          {
            printf("Sync point buffer overrun j=%d\n", j);
          }
      }

   i ++;
  }

  if(j == 0)  OP->BestFoundTrajectoryIndex = TRAJ_POSITION_NOT_FOUND; //No trajectory position found

  int PositionFound = -1, kc = 0;
  if(OP->BestFoundTrajectoryIndex > TRAJ_POSITION_NOT_FOUND)
  {
    i = 0;
    int SampledTimeIndex[SYNC_POINT_BUFFER];
    if(MaxTimeDiff > 0)
    {
      while(i < j)
      {
        Diff = fabs(OP->SpaceTimeArr[SampledSpaceIndex[i]].Time - CurrentTime);
        if(debug) printf("%4.3f, ", Diff);
        if(Diff < MaxTimeDiff)
        //if(OP->SpaceTimeArr[SampledSpaceIndex[i]].Time > OP->SpaceTimeArr[OP->BestFoundTrajectoryIndex].Time)
        {
          SampledTimeIndex[kc] = SampledSpaceIndex[i];
          kc ++;
        }
        i ++;
      }
    } else for(i = 0; i < j; i++) SampledTimeIndex[i] = SampledSpaceIndex[i];

    if(debug) printf("\n");

    i = 0;
    int Init = 1;
    double PrevDiff = 0;
    while(i < kc)
    {
      if(Init == 1) PositionFound = SampledTimeIndex[i];
      Diff = fabs(OP->SpaceTimeArr[SampledTimeIndex[i]].OrigoDistance - OP->OrigoDistance); //+ fabs(OP->SpaceTimeArr[SampledSpaceIndex[i]].Time - CurrentTime);
      if(debug) printf("%4.3f, ", Diff);
      if(Diff < PrevDiff && Init == 0)
      {
       PositionFound = SampledTimeIndex[i];
      }
      Init = 0;
      PrevDiff = Diff;
      i ++;
    }
    if(debug) printf("\n");

    if(PositionFound > TRAJ_POSITION_NOT_FOUND)
    {
      OP->BestFoundTrajectoryIndex = OP->SpaceTimeArr[PositionFound].Index;
      OP->SpaceTimeFoundIndex = PositionFound;

      /*
      printf("BestFoundTrajectoryIndex=%d\n", OP->BestFoundTrajectoryIndex);
      printf("Current origo distance=%4.3f m\n", OP->OrigoDistance);
      printf("Current time=%4.3f s\n", CurrentTime);
      printf("Matched origo distance=%4.3f m\n", OP->SpaceTimeArr[PositionFound].OrigoDistance);
      printf("Distance error=%4.3f m\n", OP->OrigoDistance - OP->SpaceTimeArr[PositionFound].OrigoDistance);
      printf("Expected time=%4.3f s (index=%d)\n", OP->SpaceTimeArr[PositionFound].Time, OP->SpaceTimeArr[PositionFound].Index);
      printf("Time error=%4.3f s\n", CurrentTime - OP->SpaceTimeArr[PositionFound].Time);
      */
    }
    else
    {
      //OP->BestFoundTrajectoryIndex = TRAJ_MASTER_LATE;
      //printf("Not in time\n");
    }
  }
  return PositionFound;
}

int UtilCountFileRows(FILE *fd)
{
  int c = 0;
  int rows = 0;

  while(c != EOF)
  {

    c = fgetc(fd);
    //printf("%x-", c);
    if(c == '\n') rows++;
  }

  return rows;
}

int UtilReadLineCntSpecChars(FILE *fd, char *Buffer)
{
  int c = 0;
  int d = 0;
  int SpecChars = 0;
  int comment = 0;

  while( (c != EOF) && (c != '\n') )
  {
    c = fgetc(fd);
    //printf("%x-", c);
    if(c != '\n')
    {
      if(c == '/')
      {
       comment++;
       d = c;
      }

      if(comment == 0)
      {
        *Buffer = (char)c;
        Buffer++;
        if(c == ';' || c == ':') SpecChars++;
      }
      else if (comment == 1 && c != '/')
      {
        *Buffer = (char)d;
        Buffer++;
        if(d == ';' || d == ':') SpecChars++;
        *Buffer = (char)c;
        Buffer++;
        if(c == ';' || c == ':') SpecChars++;
        comment = 0;
      }
      else if(comment == 2 && c == '\n')
      {
        c = 1;
        comment = 0;
      }
      else if(comment == 2 && c != '\n')
      {
        //just continue
      }

    }
  }
  return SpecChars;
}


int UtilReadLine(FILE *fd, char *Buffer)
{
  int c = 0;
  int d = 0;
  int SpecChars = 0;
  int comment = 0;

  while( (c != EOF) && (c != '\n') )
  {
    c = fgetc(fd);
    //printf("%x-", c);
    if(c != '\n')
    {
        *Buffer = (char)c;
        Buffer++;
        d++;
     }
  }
  return d;
}



C8 * UtilSearchTextFile(C8 *Filename, C8 *Text1, C8 *Text2, C8 *Result)
{

  FILE *fd;

  char RowBuffer[MAX_ROW_SIZE];
  char DataBuffer[MAX_ROW_SIZE];
  char *PtrText1;
  char *PtrText2;
  int Length;
  U8 Found = 0;

  fd = fopen (Filename, "r");
  int RowCount = UtilCountFileRows(fd);
  fclose(fd);

  fd = fopen (Filename, "r");
  if(fd > 0)
  {
     do
    {
      bzero(RowBuffer, MAX_ROW_SIZE);
      UtilReadLineCntSpecChars(fd, RowBuffer);
      bzero(DataBuffer, MAX_ROW_SIZE);
      PtrText1 = strstr(RowBuffer, (const char *)Text1);
      if(PtrText1 != NULL)
      {
        if(strlen(Text2) > 0)
        {
          PtrText2 = strstr((const char *)(PtrText1+1), (const char *)Text2);
          if(PtrText2 != NULL)
          {
            strncpy(Result, PtrText1+strlen(Text1), strlen(RowBuffer) - strlen(Text1) - strlen(Text2));
          }
        }
        else
        {
          strncpy(Result, PtrText1+strlen(Text1), strlen(RowBuffer) - strlen(Text1));
        }
        Found = 1;
      }
      RowCount--;

    } while(Found == 0 && RowCount >= 0);

    fclose(fd);
  }

  //printf("String found: %s\n", Result);
  return Result;

}


int iUtilGetIntParaConfFile(char* pcParameter, int* iValue)
{
  int iResult;
  char pcValue[512];

  bzero(pcValue,512);
  iResult = iUtilGetParaConfFile(pcParameter,pcValue);

  if (iResult > 0)
  {
    *iValue = atoi(pcValue);
  }

  return iResult;
}

/*******************************************************
* Message queue bus wrapper functions
********************************************************/
/*!
 * \brief iCommInit Connects to the message queue bus
 * \return 0 upon success, -1 if there was an error
 */
int iCommInit(void)
{
    enum MQBUS_ERROR result = MQBusConnect();
    switch (result)
    {
    case MQBUS_OK:
        LogMessage(LOG_LEVEL_DEBUG, "Connected to message queue bus");
        return 0;
    case MQBUS_QUEUE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Unable to fetch available message queue bus slots");
        break;
    case MQBUS_OPEN_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to open message queue");
        break;
    case MQBUS_MAX_QUEUES_LIMIT_REACHED:
        LogMessage(LOG_LEVEL_ERROR, "Unable to find an available message queue slot");
        break;
    case MQBUS_EMPTY:
        LogMessage(LOG_LEVEL_ERROR, "Message queue empty");
        break;
    case MQBUS_CLOSE_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to close message queue");
        break;
    case MQBUS_NO_READABLE_MQ:
        LogMessage(LOG_LEVEL_ERROR, "Unable to find assigned message queue slot");
        break;
    case MQBUS_RESOURCE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Resource queue unavailable");
        break;
    case MQBUS_DESCRIPTOR_NOT_FOUND:
        LogMessage(LOG_LEVEL_ERROR, "Message queue descriptor not found");
        break;
    case MQBUS_INVALID_INPUT_ARGUMENT:
        LogMessage(LOG_LEVEL_ERROR, "Invalid argument to message queue connect");
        break;
    case MQBUS_MQ_COULD_NOT_BE_DESTROYED:
        LogMessage(LOG_LEVEL_ERROR, "Unable to delete message queue");
        break;
    case MQBUS_MQ_FULL:
        LogMessage(LOG_LEVEL_ERROR, "Message queue full");
        break;
    case MQBUS_WRITE_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Message queue write failed");
        break;
    }
    return -1;
}


/*!
 * \brief iCommClose Releases the claimed slot on the message bus so that it can be claimed by others.
 * \return 0 upon success, -1 on error
 */
int iCommClose()
{
    enum MQBUS_ERROR result = MQBusDisconnect();

    switch (result)
    {
    case MQBUS_OK:
        return 0;
    case MQBUS_RESOURCE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Unable to release claimed message bus slot");
        return -1;
    case MQBUS_NO_READABLE_MQ:
        LogMessage(LOG_LEVEL_WARNING, "Attempted to release message bus slot when none was claimed");
        return 0;
    default:
        LogMessage(LOG_LEVEL_WARNING, "Unexpected message bus error when closing");
        return -1;
    }
}


/*!
 * \brief iCommRecv Poll message queue bus for received data in a nonblocking way, returning the oldest message
 * with lowest priority value found on the queue, as well as the command identifier and time of receipt in UTC
 * \param command Received command output variable
 * \param data Received command data output variable
 * \param messageSize Size of data array
 * \param timeRecvUTC Receive time output variable
 * \return Size (in bytes) of received data
 */
ssize_t iCommRecv(enum COMMAND *command, char* data, const int messageSize, char* timeRecvUTC)
{
    char message[MQ_MSG_SIZE];
    struct timeval tvTime;
    ssize_t result = MQBusRecv(message, MQ_MSG_SIZE);


    if (result < 1 && errno != EAGAIN)
        util_error("Message queue error when receiving");

    if (timeRecvUTC != NULL)
    {
        // Store time of receipt
        TimeSetToCurrentSystemTime(&tvTime);
        sprintf(timeRecvUTC, "%" PRId64, TimeGetAsUTCms(&tvTime));
    }

    if (result > 0)
    {
        // A message was received: extract the command and data
        *command = message[0];
        if((strlen(message) > 1) && (data != NULL))
        {
            if(messageSize < result )
            {
                LogMessage(LOG_LEVEL_WARNING, "Array too small to hold received message data");
                result = messageSize;
            }
            strncat(data, &message[1], (unsigned long)result);
        }
    }
    else
    {
        // Result was less than 0, and errno is EAGAIN meaning no data was received
        *command = COMM_INV;
        result = 0;
    }
    return result;
}

/*!
 * \brief iCommSend Sends a command over the message bus
 * \param iCommand Command number
 * \param cpData Command data
 * \return 0 upon success, 1 upon partial success (e.g. a message queue was full), -1 on error
 */
int iCommSend(const enum COMMAND iCommand, const char* cpData)
{
    unsigned int uiMessagePrio = 0;
    char cpMessage[MQ_MSG_SIZE];
    unsigned long dataLength = strlen(cpData);
    enum MQBUS_ERROR sendResult;

    if (dataLength > MQ_MSG_SIZE+1)
    {
        LogMessage(LOG_LEVEL_ERROR, "Cannot send message %d of size %lu: maximum size is %lu", iCommand, dataLength+1, MQ_MSG_SIZE);
        return -1;
    }

    bzero(cpMessage, MQ_MSG_SIZE);

    // Map command to a priority
    switch (iCommand)
    {
    case COMM_STRT:
        uiMessagePrio = PRIO_COMM_STRT;
        cpMessage[0] = (char)COMM_STRT;
        break;
    case COMM_ARMD:
        uiMessagePrio = PRIO_COMM_ARMD;
        cpMessage[0] = (char)COMM_ARMD;
        break;
    case COMM_STOP:
        uiMessagePrio = PRIO_COMM_STOP;
        cpMessage[0] = (char)COMM_STOP;
        break;
    case COMM_MONI:
        uiMessagePrio = PRIO_COMM_MONI;
        cpMessage[0] = (char)COMM_MONI;
        break;
    case COMM_EXIT:
        uiMessagePrio = PRIO_COMM_EXIT;
        cpMessage[0] = (char)COMM_EXIT;
        break;
    case COMM_REPLAY:
        uiMessagePrio = PRIO_COMM_REPLAY;
        cpMessage[0] = (char)COMM_REPLAY;
        break;
    case COMM_CONTROL:
        uiMessagePrio = PRIO_COMM_CONTROL;
        cpMessage[0] = (char)COMM_CONTROL;
        break;
    case COMM_ABORT:
        uiMessagePrio = PRIO_COMM_ABORT;
        cpMessage[0] = (char)COMM_ABORT;
        break;
    case COMM_TOM:
        uiMessagePrio = PRIO_COMM_TOM;
        cpMessage[0] = (char)COMM_TOM;
        break;
    case COMM_INIT:
        uiMessagePrio = PRIO_COMM_INIT;
        cpMessage[0] = (char)COMM_INIT;
        break;
    case COMM_CONNECT:
        uiMessagePrio = PRIO_COMM_CONNECT;
        cpMessage[0] = (char)COMM_CONNECT;
        break;
    case COMM_OBC_STATE:
        uiMessagePrio = PRIO_COMM_OBC_STATE;
        cpMessage[0] = (char)COMM_OBC_STATE;
        break;
    case COMM_DISCONNECT:
        uiMessagePrio = PRIO_COMM_DISCONNECT;
        cpMessage[0] = (char)COMM_DISCONNECT;
        break;
    case COMM_LOG:
        uiMessagePrio = PRIO_COMM_LOG;
        cpMessage[0] = (char)COMM_LOG;
        break;
    case COMM_OSEM:
        uiMessagePrio = PRIO_COMM_OSEM;
        cpMessage[0] = (char)COMM_OSEM;
        break;
    case COMM_VIOP:
        uiMessagePrio = PRIO_COMM_VIOP;
        cpMessage[0] = (char)COMM_VIOP;
        break;
    case COMM_TRAJ:
        uiMessagePrio = PRIO_COMM_TRAJ;
        cpMessage[0] = (char)COMM_TRAJ;
        break;
    case COMM_ASP:
        uiMessagePrio = PRIO_COMM_ASP;
        cpMessage[0] = (char)COMM_ASP;
        break;
    case COMM_TRAJ_TOSUP:
        uiMessagePrio = PRIO_COMM_TRAJ_TOSUP;
        cpMessage[0] = (char)COMM_TRAJ_TOSUP;
        break;
    case COMM_TRAJ_FROMSUP:
        uiMessagePrio = PRIO_COMM_TRAJ_FROMSUP;
        cpMessage[0] = (char)COMM_TRAJ_FROMSUP;
        break;
    default:
        util_error("Unknown command");
    }

    // Append message to command
    if(cpData != NULL)
    {
        (void)strncat(&cpMessage[1], cpData, strlen(cpData));
    }

    // Send message
    sendResult = MQBusSend(cpMessage, strlen(cpMessage), uiMessagePrio);

    // Check for send success
    switch (sendResult)
    {
    case MQBUS_OK:
        return 0;
    case MQBUS_MQ_FULL:
        LogMessage(LOG_LEVEL_WARNING, "Attempted to write to full message queue - messages may be lost");
        return 1;
    case MQBUS_INVALID_INPUT_ARGUMENT:
        LogMessage(LOG_LEVEL_WARNING, "Invalid message queue message length");
        return 1;
    case MQBUS_WRITE_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Failed to write to message queue");
        break;
    case MQBUS_QUEUE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Unable to fetch available message queue bus slots");
        break;
    case MQBUS_OPEN_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to open message queue");
        break;
    case MQBUS_MAX_QUEUES_LIMIT_REACHED:
        LogMessage(LOG_LEVEL_ERROR, "Unable to find an available message queue slot");
        break;
    case MQBUS_EMPTY:
        LogMessage(LOG_LEVEL_ERROR, "Message queue empty");
        break;
    case MQBUS_CLOSE_FAIL:
        LogMessage(LOG_LEVEL_ERROR, "Unable to close message queue");
        break;
    case MQBUS_NO_READABLE_MQ:
        LogMessage(LOG_LEVEL_ERROR, "Unable to find assigned message queue slot");
        break;
    case MQBUS_RESOURCE_NOT_EXIST:
        LogMessage(LOG_LEVEL_ERROR, "Resource queue unavailable");
        break;
    case MQBUS_DESCRIPTOR_NOT_FOUND:
        LogMessage(LOG_LEVEL_ERROR, "Message queue descriptor not found");
        break;
    case MQBUS_MQ_COULD_NOT_BE_DESTROYED:
        LogMessage(LOG_LEVEL_ERROR, "Unable to delete message queue");
        break;
    }

    return -1;
}

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
                monitor_t* ldm  )
{

  char   pcTempBuffer [512];

  double earth_radius = a;

  double lat_origin = 0.0;
  double lon_origin = 0.0;
  double alt_origin = 0.0;
  double lat        = 0.0;
  double lon        = 0.0;
  double alt        = 0.0;

  ldm->timestamp = (uint64_t) (time * 100);
  ldm->speed     = (uint16_t) (vel * 100);
  ldm->heading   = (uint16_t) (hdg * 100);

  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoLatidude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &lat_origin);

  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoLongitude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &lon_origin);

  bzero(pcTempBuffer,512);
  iUtilGetParaConfFile("OrigoAltitude=", pcTempBuffer);
  sscanf(pcTempBuffer, "%lf", &alt_origin);

  lat = ((y * 180)/(PI * earth_radius)) + lat_origin;
  lon = ((x * 180)/(PI * earth_radius)) * (1 / (cos((PI / 180) * (0.5 * (lat_origin + lat))))) + lon_origin;
  alt = z + alt_origin;

  ldm->latitude  = (uint32_t) (lat * 10000000);
  ldm->longitude = (uint32_t) (lon * 10000000);
  ldm->altitude  = (uint32_t) (alt * 100);

}


static void             init_crc16_tab( void );

static bool             crc_tab16_init          = false;
static uint16_t         crc_tab16[256];

#define   CRC_POLY_16   0xA001

static void init_crc16_tab( void ) {

  uint16_t i;
  uint16_t j;
  uint16_t crc;
  uint16_t c;

  for (i=0; i<256; i++) {

    crc = 0;
    c   = i;

    for (j=0; j<8; j++) {

      if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ CRC_POLY_16;
      else                      crc =   crc >> 1;

      c = c >> 1;
    }

    crc_tab16[i] = crc;
  }

  crc_tab16_init = true;

}


#define   CRC_START_16    0x0000

uint16_t crc_16( const unsigned char *input_str, uint16_t num_bytes ) {

  uint16_t crc;
  uint16_t tmp;
  uint16_t short_c;
  const unsigned char *ptr;
  uint16_t i = 0;

  if ( ! crc_tab16_init ) init_crc16_tab();

  crc = CRC_START_16;
  ptr = input_str;

  if ( ptr != NULL ) for (i=0; i<num_bytes; i++) {

    short_c = 0x00ff & (uint16_t) *ptr;
    tmp     =  crc       ^ short_c;
    crc     = (crc >> 8) ^ crc_tab16[ tmp & 0xff ];
    ptr++;
  }

  return crc;

}  /* crc_16 */




U16 SwapU16(U16 val)
{
    return (val << 8) | (val >> 8 );
}

I16 SwapI16(I16 val)
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

U32 SwapU32(U32 val)
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

I32 SwapI32(I32 val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

I64 SwapI64(I64 val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL ) | ((val >> 8) & 0x00FF00FF00FF00FFULL );
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL ) | ((val >> 16) & 0x0000FFFF0000FFFFULL );
    return (val << 32) | ((val >> 32) & 0xFFFFFFFFULL);
}

U64 SwapU64(U64 val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL ) | ((val >> 8) & 0x00FF00FF00FF00FFULL );
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL ) | ((val >> 16) & 0x0000FFFF0000FFFFULL );
    return (val << 32) | (val >> 32);
}


I32 UtilConnectTCPChannel(const C8* Module, I32* Sockfd, const C8* IP, const U32 Port)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    int iResult;

    *Sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (*Sockfd < 0)    {
        LogMessage(LOG_LEVEL_ERROR,"Failed to open control socket");
    }

    server = gethostbyname(IP);
    if (server == NULL)
    {
        LogMessage(LOG_LEVEL_ERROR,"Unknown host");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(Port);


    LogMessage(LOG_LEVEL_INFO,"Attempting to connect to control socket: %s:%i", IP, Port);

    do
    {
        iResult = connect(*Sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

        if ( iResult < 0)
        {
            if(errno == ECONNREFUSED)
            {
                LogMessage(LOG_LEVEL_WARNING,"Unable to connect to %s port %d, retrying in 3 sec...", IP, Port);
                fflush(stdout);
                (void)sleep(3);
            }
            else
            {
                 LogMessage(LOG_LEVEL_ERROR,"Failed to connect to control socket");
            }
        }
    } while(iResult < 0);


    iResult = fcntl(*Sockfd, F_SETFL, fcntl(*Sockfd, F_GETFL, 0) | O_NONBLOCK);

    LogMessage(LOG_LEVEL_INFO,"Maestro connected to %s:%d", IP, Port);
    return iResult;

}


void UtilSendTCPData(const C8* Module, const C8* Data, I32 Length, I32* Sockfd, U8 Debug)
{
    I32 i, n, error = 0;

    socklen_t len = sizeof(error);
    I32 retval;

    // TODO: Change this when bytes thingy has been implemented in logging
    if(Debug == 1){ printf("[%s] Bytes sent: ", Module); i = 0; for(i = 0; i < Length; i++) printf("%x-", (C8)*(Data+i)); printf("\n");}

    n = write(*Sockfd, Data, Length);

    retval = getsockopt(*Sockfd, SOL_SOCKET, SO_ERROR, &error, &len);

    if(retval != 0)
    {
      LogMessage(LOG_LEVEL_ERROR,"Failed to get socket error code = %s", strerror(retval));
    }

    if(error != 0)
    {
      LogMessage(LOG_LEVEL_ERROR,"Socket error: %s", strerror(error));
    }

    if (n < 0)
    {
        LogMessage(LOG_LEVEL_ERROR,"Failed to send on control socket, length = %d", Length);
    }
}


I32 UtilReceiveTCPData(const C8* Module, I32* Sockfd, C8* Data, I32 Length, U8 Debug)
{
    I32 i, Result;

    if(Length <= 0) Result = recv(*Sockfd, Data, TCP_RX_BUFFER,  0);
    else Result = recv(*Sockfd, Data, Length,  0);

    // TODO: Change this when bytes thingy has been implemented in logging
    if(Debug == 1 && Result < 0){ printf("[%s] Received TCP data: ", Module); i = 0; for(i = 0; i < Result; i++) printf("%x-", (C8)*(Data+i)); printf("\n");}

    return Result;
}




void UtilCreateUDPChannel(const C8* Module, I32 *Sockfd, const C8* IP, const U32 Port, struct sockaddr_in* Addr)
{
    int result;
    struct hostent *object;


    *Sockfd= socket(AF_INET, SOCK_DGRAM, 0);
    if (*Sockfd < 0)
    {
        LogMessage(LOG_LEVEL_ERROR,"Failed to connect to CPC socket");
    }

    /* Set address to object */
    object = gethostbyname(IP);

    if (object==0)
    {
        LogMessage(LOG_LEVEL_ERROR,"Unknown host");
    }

    bcopy((char *) object->h_addr, (char *)&Addr->sin_addr.s_addr, object->h_length);
    Addr->sin_family = AF_INET;
    Addr->sin_port = htons(Port);

    /* set socket to non-blocking */
    result = fcntl(*Sockfd, F_SETFL,
                   fcntl(*Sockfd, F_GETFL, 0) | O_NONBLOCK);
    if (result < 0)
    {
        LogMessage(LOG_LEVEL_ERROR,"Error calling fcntl");
    }

    LogMessage(LOG_LEVEL_INFO,"Created UDP channel to address: %s:%d", IP, Port);


}


void UtilSendUDPData(const C8* Module, I32 *Sockfd, struct sockaddr_in* Addr, C8 *Data, I32 Length, U8 Debug)
{
    I32 result, i;


    result = sendto(*Sockfd, Data, Length, 0, (const struct sockaddr *) Addr, sizeof(struct sockaddr_in));

    // TODO: Change this when bytes thingy has been implemented in logging
    if(Debug){ printf("[%s] Bytes sent: ", Module); i = 0; for(i = 0; i < Length; i++) printf("%x-", (unsigned char)*(Data+i)); printf("\n");}

    if (result < 0)
    {
        LogMessage(LOG_LEVEL_ERROR,"Failed to send on process control socket");
    }

}


void UtilReceiveUDPData(const C8* Module, I32* Sockfd, C8* Data, I32 Length, I32* ReceivedNewData, U8 Debug)
{
    I32 Result, i;
    *ReceivedNewData = 0;
    do
    {
        Result = recv(*Sockfd, Data, Length, 0);

        if (Result < 0)
        {
            if(errno != EAGAIN && errno != EWOULDBLOCK)
            {
                LogMessage(LOG_LEVEL_ERROR,"Failed to receive from monitor socket");
            }
            else
            {
                LogMessage(LOG_LEVEL_DEBUG, "No data received");
            }
        }
        else
        {
            *ReceivedNewData = 1;
            LogMessage(LOG_LEVEL_DEBUG,"Received: <%s>", Data);
            // TODO: Change this when bytes thingy has been implemented in logging
            if(Debug == 1){ printf("[%s] Received UDP data: ", Module); i = 0; for(i = 0; i < Result; i++) printf("%x-", (C8)*(Data+i)); printf("\n");}

        }

    } while(Result > 0 );
}


U32 UtilIPStringToInt(C8 *IP)
{
    C8 *p, *ps;
    C8 Buffer[3];
    U32 IpU32 = 0;

    ps = IP;
    p = strchr(IP,'.');
    if(p != NULL)
    {
      bzero(Buffer,3);
      strncpy(Buffer, ps, (U64)p - (U64)ps);
      IpU32 = (IpU32 | (U32)atoi(Buffer)) << 8;

      ps = p + 1;
      p = strchr(ps,'.');
      bzero(Buffer,3);
      strncpy(Buffer, ps, (U64)p - (U64)ps);

      IpU32 = (IpU32 | (U32)atoi(Buffer)) << 8;

      ps = p + 1;
      p = strchr(ps,'.');
      bzero(Buffer,3);
      strncpy(Buffer, ps, (U64)p - (U64)ps);

      IpU32 = (IpU32 | (U32)atoi(Buffer)) << 8;

      ps = p + 1;
      p = strchr(ps, 0);
      bzero(Buffer,3);
      strncpy(Buffer, ps, (U64)p - (U64)ps);

      IpU32 = (IpU32 | (U32)atoi(Buffer));

      //printf("IpU32 = %x\n", IpU32);
    }

    return IpU32;
}


U32 UtilHexTextToBinary(U32 DataLength, C8 *Text, C8 *Binary, U8 Debug)
{
  U32 i, j = 0;
  C8 Bin;
  C8 Hex;

  for(i = 0; i < DataLength; )
  {

    Hex = *(Text + i++);
    if(Hex >= 0x30 && Hex <= 0x39) Hex = Hex - 0x30;
    else if (Hex >= 0x41 && Hex <= 0x46) Hex = Hex - 0x37;
    Bin = Hex << 4;

    Hex = *(Text + i++);
    if(Hex >= 0x30 && Hex <= 0x39) Hex = Hex - 0x30;
    else if (Hex >= 0x41 && Hex <= 0x46) Hex = Hex - 0x37;
    Bin = Bin | Hex;

    *(Binary + j++) = Bin;

  }

 if(Debug)
  {
     // TODO: Change this when bytes thingy has been implemented in logging
    printf("[Util:UtilHexTextToBinary] Length = %d: ", DataLength/2);
    for(i = 0;i < DataLength/2; i ++) printf("%x ", *(Binary + i));
    printf("\n");
  }


  return j;
}


U32 UtilBinaryToHexText(U32 DataLength, C8 *Binary, C8 *Text, U8 Debug)
{
    U32 i, j=0;
    C8 Hex;


    for(i = 0; i < DataLength; i++)
    {
      Hex = *(Binary + i) >> 4;
      //Hex = Hex >> 4;
      //printf("%x\n", Hex);
      if(Hex >= 0 && Hex <= 9) Hex = Hex + 0x30;
      else if (Hex >= 10 && Hex <= 15) Hex = Hex + 0x37;
      //printf("%x\n", Hex);
      *(Text + j++) = Hex;

      Hex = *(Binary + i) & 0x0F;
      //printf("%x\n", Hex);
      //Hex = Hex & 0x0F;
      //printf("%x\n", Hex);
      if(Hex >= 0 && Hex <= 9) Hex = Hex + 0x30;
      else if (Hex >= 10 && Hex <= 15) Hex = Hex + 0x37;
      //printf("%x", Hex);
      *(Text + j++) = Hex;
    }


   if(Debug)
    {
       // TODO: Change this when bytes thingy has been implemented in logging
      printf("[Util:UtilBinaryToHexText] Length = %d: ", j);
      for(i = 0;i < j; i ++) printf("%x ", *(Text + i));
      printf("\n");
    }



    return j;
}


#define NORMAL_COLOR  "\x1B[0m"
#define GREEN  "\x1B[32m"
#define BLUE  "\x1B[34m"

// "F-<filename>\n" and "D-<filename>\n" is 4 bytes longer than only the filename
#define FILE_INFO_LENGTH (MAX_PATH_LENGTH+4)

U32 UtilCreateDirContent(C8* DirPath, C8* TempPath)
{

  FILE *fd;
  C8 Filename[FILE_INFO_LENGTH];
  C8 CompletePath[MAX_PATH_LENGTH];
  bzero(CompletePath, MAX_PATH_LENGTH);
  GetCurrentDir(CompletePath, MAX_PATH_LENGTH);
  strcat(CompletePath, DirPath);//Concatenate dir path

  DIR * d = opendir(CompletePath); // open the path
  if(d==NULL) return 1; // if was not able return
  struct dirent * dir; // for the directory entries

  bzero(CompletePath, MAX_PATH_LENGTH);
  GetCurrentDir(CompletePath, MAX_PATH_LENGTH);
  strcat(CompletePath, TempPath); //Concatenate temp file path

  fd = fopen(CompletePath, "r");
  if(fd != NULL)
  {
      fclose(fd);
      remove(CompletePath); //Remove file if exist
  }

  fd = fopen(CompletePath, "w+"); //Create the file
  if(fd == NULL) //return if failing to create file
  {
      return 2;
  }

  while ((dir = readdir(d)) != NULL) // if we were able to read somehting from the directory
  {
    bzero(Filename, FILE_INFO_LENGTH);

    if(dir-> d_type != DT_DIR)
      //printf("%s%s\n",BLUE, dir->d_name); // if the type is not directory just print it with blue
      sprintf(Filename, "F-%s\n", dir->d_name);
    else
    if(dir -> d_type == DT_DIR && strcmp(dir->d_name,".")!=0 && strcmp(dir->d_name,"..")!=0 ) // if it is a directory
    {
      sprintf(Filename, "D-%s\n", dir->d_name);
      //printf("%s%s\n", GREEN, dir->d_name); // print its name in green
      //printf("D-%s\n", dir->d_name); // print its name in green
      //char d_path[255]; // here I am using sprintf which is safer than strcat
      //sprintf(d_path, "%s/%s", Path, dir->d_name);
      //UtilCreateDirContent(d_path, TempPath); // recall with the new path
    }

    if(strlen(Filename) > 0)
    {
      //printf("%s", Filename);
      fwrite(Filename, 1, strlen(Filename), fd); //write dir content to file
      fflush(fd);
    }
  }
  //printf("\n");
  //printf("%s\n", CompletePath);

  closedir(d); // close the directory

  fclose(fd); //close the file

  return 0;
}



I32 UtilISOBuildINSUPMessage(C8* MessageBuffer, INSUPType *INSUPData, C8 CommandOption, U8 Debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, ISO_INSUP_MESSAGE_LENGTH+ISO_MESSAGE_FOOTER_LENGTH);

    INSUPData->Header.SyncWordU16 = SYNC_WORD;
    INSUPData->Header.TransmitterIdU8 = 0;
    INSUPData->Header.MessageCounterU8 = 0;
    INSUPData->Header.AckReqProtVerU8 = 0;
    INSUPData->Header.MessageIdU16 = ISO_INSUP_CODE;
    INSUPData->Header.MessageLengthU32 = sizeof(INSUPType) - sizeof(HeaderType);
    INSUPData->ModeValueIdU16 = VALUE_ID_INSUP_MODE;
    INSUPData->ModeContentLengthU16 = 1;
    INSUPData->ModeU8 = (U8)CommandOption;

    p=(C8 *)INSUPData;
    for(i=0; i<sizeof(INSUPType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8 *)MessageBuffer, sizeof(OSTMType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    *(MessageBuffer + i++) = (U8)(Crc);
    MessageIndex = i;

    if(Debug)
    {
        // TODO: Change this when bytes thingy has been implemented in logging
        printf("INSUP total length = %d bytes (header+message+footer)\n", (int)(ISO_INSUP_MESSAGE_LENGTH+ISO_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(INSUPType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}

I32 UtilISOBuildHEABMessage(C8* MessageBuffer, HEABType *HEABData, TimeType *GPSTime, U8 CCStatus, U8 Debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, ISO_HEAB_MESSAGE_LENGTH+ISO_MESSAGE_FOOTER_LENGTH);

    HEABData->Header.SyncWordU16 = SYNC_WORD;
    HEABData->Header.TransmitterIdU8 = 0;
    HEABData->Header.MessageCounterU8 = 0;
    HEABData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    HEABData->Header.MessageIdU16 = ISO_HEAB_CODE;
    HEABData->Header.MessageLengthU32 = sizeof(HEABType) - sizeof(HeaderType);
    //HEABData->HeabStructValueIdU16 = 0;
    //HEABData->HeabStructContentLengthU16 = sizeof(HEABType) - sizeof(HeaderType) - 4;
    HEABData->GPSSOWU32 = ((GPSTime->GPSSecondsOfWeekU32*1000 + (U32)UtilGetMillisecond(GPSTime)) << 2) + GPSTime->MicroSecondU16;
    HEABData->CCStatusU8 = CCStatus;

    if(!GPSTime->isGPSenabled){
        UtilgetCurrentGPStime(NULL,&HEABData->GPSSOWU32);
    }

    p=(C8 *)HEABData;
    for(i=0; i<sizeof(HEABType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8*)MessageBuffer, sizeof(HEABType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc);
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    MessageIndex = i;

    if(Debug)
    {
        // TODO: Change this when bytes thingy has been implemented in logging
        printf("HEAB total length = %d bytes (header+message+footer)\n", (int)(ISO_HEAB_MESSAGE_LENGTH + ISO_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(HEABType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}


U16 UtilGetMillisecond(TimeType *GPSTime)
{
  struct timeval now;
  U16 MilliU16 = 0, NowU16 = 0;
  gettimeofday(&now, NULL);
  NowU16 = (U16)(now.tv_usec / 1000);

  if(NowU16 >= GPSTime->LocalMillisecondU16) MilliU16 = NowU16 - GPSTime->LocalMillisecondU16;
  else if(NowU16 < GPSTime->LocalMillisecondU16) MilliU16 = 1000 - GPSTime->LocalMillisecondU16 + NowU16;

  //printf("Result= %d, now= %d, local= %d \n", MilliU16, NowU16, GPSTime->LocalMillisecondU16);
  return MilliU16;
}

I32 UtilISOBuildTRAJInfo(C8* MessageBuffer, TRAJInfoType *TRAJInfoData, U8 debug)
{
    I32 MessageIndex = 0, i;

    U16 Crc = 0, U16Data = 0;
    I16 I16Data = 0;
    U32 U32Data = 0;
    I32 I32Data = 0;
    U64 U64Data = 0;
    C8 *p;

    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+1)) << 8;
    U16Data = U16Data | *MessageBuffer;
    TRAJInfoData->TrajectoryIDValueIdU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+3)) << 8;
    U16Data = U16Data | *(MessageBuffer+2);
    TRAJInfoData->TrajectoryIDContentLengthU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+5)) << 8;
    U16Data = U16Data | *(MessageBuffer+4);
    TRAJInfoData->TrajectoryIDU16 = U16Data;

    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+7)) << 8;
    U16Data = U16Data | *(MessageBuffer+6);
    TRAJInfoData->TrajectoryNameValueIdU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+9)) << 8;
    U16Data = U16Data | *(MessageBuffer+8);
    TRAJInfoData->TrajectoryNameContentLengthU16 = U16Data;
    for(i = 0; i < TRAJInfoData->TrajectoryNameContentLengthU16; i ++) TRAJInfoData->TrajectoryNameC8[i] = *(MessageBuffer + 10 + i);

    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+65)) << 8;
    U16Data = U16Data | *(MessageBuffer+64);
    TRAJInfoData->TrajectoryVersionValueIdU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+67)) << 8;
    U16Data = U16Data | *(MessageBuffer+66);
    TRAJInfoData->TrajectoryVersionContentLengthU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+69)) << 8;
    U16Data = U16Data | *(MessageBuffer+68);
    TRAJInfoData->TrajectoryVersionU16 = U16Data;

    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+71)) << 8;
    U16Data = U16Data | *(MessageBuffer+70);
    TRAJInfoData->IpAddressValueIdU16 = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+73)) << 8;
    U16Data = U16Data | *(MessageBuffer+72);
    TRAJInfoData->IpAddressContentLengthU16 = U16Data;
    U32Data = 0;
    U32Data = (U32Data | *(MessageBuffer+77)) << 8;
    U32Data = (U32Data | *(MessageBuffer+76)) << 8;
    U32Data = (U32Data | *(MessageBuffer+75)) << 8;
    U32Data = U32Data | *(MessageBuffer+74);
    TRAJInfoData->IpAddressU32 = U32Data;

    if(debug)
    {
        // TODO: Change this when bytes thingy has been implemented in logging
        printf("TRAJInfo total length = %d bytes\n", (int)(ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH));
        printf("----TRAJInfo----\n");
        for(i = 0;i < sizeof(TRAJInfoType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
        printf("TrajectoryID = %d\n", TRAJInfoData->TrajectoryIDU16);
        printf("TrajectoryName = %s\n", TRAJInfoData->TrajectoryNameC8);
        printf("TrajectoryVersion = %d\n", TRAJInfoData->TrajectoryVersionU16);
        printf("IpAddress = %d\n", TRAJInfoData->IpAddressU32);
        printf("\n----MESSAGE----\n");
    }

    return 0;
}


I32 UtilISOBuildTRAJMessageHeader(C8* MessageBuffer, I32 RowCount, HeaderType *HeaderData, TRAJInfoType *TRAJInfoData, U8 debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, ISO_MESSAGE_HEADER_LENGTH + ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH);

    HeaderData->SyncWordU16 = SYNC_WORD;
    HeaderData->TransmitterIdU8 = 0;
    HeaderData->MessageCounterU8 = 0;
    HeaderData->AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    HeaderData->MessageIdU16 = ISO_TRAJ_CODE;
    HeaderData->MessageLengthU32 = ISO_DTM_ROW_MESSAGE_LENGTH*RowCount + ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH;

    p=(C8 *)HeaderData;
    for(i=0; i< ISO_MESSAGE_HEADER_LENGTH; i++) *(MessageBuffer + i) = *p++;


    TRAJInfoData->TrajectoryIDValueIdU16 = VALUE_ID_TRAJECTORY_ID;
    TRAJInfoData->TrajectoryIDContentLengthU16 = 2;

    TRAJInfoData->TrajectoryNameValueIdU16 = VALUE_ID_TRAJECTORY_NAME;
    TRAJInfoData->TrajectoryNameContentLengthU16 = 64;

    TRAJInfoData->TrajectoryVersionValueIdU16 = VALUE_ID_TRAJECTORY_VERSION;
    TRAJInfoData->TrajectoryVersionContentLengthU16 = 2;

    TRAJInfoData->IpAddressValueIdU16 = 0xA000;
    TRAJInfoData->IpAddressContentLengthU16 = 4;

    p=(C8 *)TRAJInfoData;
    for(; i< ISO_MESSAGE_HEADER_LENGTH + ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH; i++) *(MessageBuffer + i) = *p++;

    MessageIndex = i;


    if(debug)
    {
        // TODO: Change this when bytes thingy has been implemented in logging
        printf("Header + TRAJInfo total length = %d bytes\n", (int)(ISO_MESSAGE_HEADER_LENGTH + ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH));
        printf("----HEADER + TRAJInfo----\n");
        for(i = 0;i < sizeof(HeaderType) + sizeof(TRAJInfoType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
        printf("DOTM message total length = %d bytes.\n", (int)HeaderData->MessageLengthU32);
        printf("TrajectoryID = %d\n", TRAJInfoData->TrajectoryIDU16);
        printf("TrajectoryName = %s\n", TRAJInfoData->TrajectoryNameC8);
        printf("TrajectoryVersion = %d\n", TRAJInfoData->TrajectoryVersionU16);
        printf("IpAddress = %d\n", TRAJInfoData->IpAddressU32);
        printf("\n----MESSAGE----\n");
    }

    return MessageIndex; //Total number of bytes = ISO_MESSAGE_HEADER_LENGTH
}

I32 UtilISOBuildTRAJMessage(C8 *MessageBuffer, C8 *DTMData, I32 RowCount, DOTMType *DOTMData, U8 debug)
{
    I32 MessageIndex = 0;
    U32 Data;
    C8 *src, *p;
    U16 Crc = 0;

    bzero(MessageBuffer, ISO_DTM_ROW_MESSAGE_LENGTH*RowCount);

    I32 i = 0, j = 0, n = 0;
    for(i = 0; i < RowCount; i++)
    {
        //Time
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 3);
        //if(debug) printf("%x-",Data);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 2);
        //if(debug) printf("%x-",Data);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 1);
        //if(debug) printf("%x-",Data);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 0);
        //if(debug) printf("%x- ",Data);
        DOTMData->RelativeTimeValueIdU16 = VALUE_ID_RELATIVE_TIME;
        DOTMData->RelativeTimeContentLengthU16 = 4;
        DOTMData->RelativeTimeU32 = SwapU32((U32)Data);

        //x
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 7);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 6);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 5);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 4);
        DOTMData->XPositionValueIdU16 = VALUE_ID_X_POSITION;
        DOTMData->XPositionContentLengthU16 = 4;
        DOTMData->XPositionI32 = SwapI32((I32)Data);

        //y
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 11);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 10);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 9);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 8);
        DOTMData->YPositionValueIdU16 = VALUE_ID_Y_POSITION;
        DOTMData->YPositionContentLengthU16 = 4;
        DOTMData->YPositionI32 = SwapI32((I32)Data);

        //z
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 15);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 14);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 13);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 12);
        DOTMData->ZPositionValueIdU16 = VALUE_ID_Z_POSITION;
        DOTMData->ZPositionContentLengthU16 = 4;
        DOTMData->ZPositionI32 = SwapI32((I32)Data);

        //Heading
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 17);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 16);
        //Data = UtilRadToDeg(Data);
        //Data = 4500 - Data; //Turn heading back pi/2
        //while(Data<0) Data+=360.0;
        //while(Data>3600) Data-=360.0;
        DOTMData->HeadingValueIdU16 = VALUE_ID_HEADING;
        DOTMData->HeadingContentLengthU16 = 2;
        DOTMData->HeadingU16 = SwapU16((U16)(Data));

        //Longitudinal speed
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 19);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 18);
        DOTMData->LongitudinalSpeedValueIdU16 = VALUE_ID_LONGITUDINAL_SPEED;
        DOTMData->LongitudinalSpeedContentLengthU16 = 2;
        DOTMData->LongitudinalSpeedI16 = SwapI16((I16)Data);

        //Lateral speed
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 21);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 20);
        DOTMData->LateralSpeedValueIdU16 = VALUE_ID_LATERAL_SPEED;
        DOTMData->LateralSpeedContentLengthU16 = 2;
        DOTMData->LateralSpeedI16 = SwapI16((I16)Data);

        //Longitudinal acceleration
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 23);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 22);
        DOTMData->LongitudinalAccValueIdU16 = VALUE_ID_LONGITUDINAL_ACCELERATION;
        DOTMData->LongitudinalAccContentLengthU16 = 2;
        DOTMData->LongitudinalAccI16 = SwapI16((I16)Data);

        //Lateral acceleration
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 25);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 24);
        DOTMData->LateralAccValueIdU16 = VALUE_ID_LATERAL_ACCELERATION;
        DOTMData->LateralAccContentLengthU16 = 2;
        DOTMData->LateralAccI16 = SwapI16((I16)Data);

        //Curvature
        Data = 0;
        Data = *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 29);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 28);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 27);
        Data = (Data<<8) | *(DTMData + SIM_TRAJ_BYTES_IN_ROW*i + 26);
        DOTMData->CurvatureValueIdU16 = VALUE_ID_CURVATURE;
        DOTMData->CurvatureContentLengthU16 = 4;
        DOTMData->CurvatureI32 = SwapI32((I32)Data);

        p=(C8 *)DOTMData;
        for(j=0; j<sizeof(DOTMType); j++, n++) *(MessageBuffer + n) = *p++;
        MessageIndex = n;

        if (debug)
        {
            LogPrint("%d. Time=%d, X=%d, Y=%d, Z=%d, Heading=%d, LongitudinalSpeedI16=%d, LateralSpeedI16=%d, LongitudinalAccI16=%d, LateralAccI16=%d, CurvatureI32=%d",
                     i, DOTMData->RelativeTimeU32,
                     DOTMData->XPositionI32,
                     DOTMData->YPositionI32,
                     DOTMData->ZPositionI32,
                     DOTMData->HeadingU16,
                     DOTMData->LongitudinalSpeedI16,
                     DOTMData->LateralSpeedI16,
                     DOTMData->LongitudinalAccI16,
                     DOTMData->LateralAccI16,
                     DOTMData->CurvatureI32);
        }
    }


    Crc = crc_16((const C8*)MessageBuffer, sizeof(DOTMType));
    Crc = 0;
    *(MessageBuffer + MessageIndex++) = (U8)(Crc);
    *(MessageBuffer + MessageIndex++) = (U8)(Crc >> 8);


    if(debug == 2)
    {
        // TODO: Replace this when bytes thingy has been implemented
        int i = 0;
        for(i = 0; i < MessageIndex; i ++)
        {
            if((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15) printf("0");
            printf("%x-", (unsigned char)MessageBuffer[i]);
        }
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}

I32 UtilISOBuildHeader(C8 *MessageBuffer, HeaderType *HeaderData, U8 Debug)
{
    I32 MessageIndex = 0, i = 0;
    dbl Data;
    U16 Crc = 0, U16Data = 0;
    I16 I16Data = 0;
    U32 U32Data = 0;
    I32 I32Data = 0;
    U64 U64Data = 0;
    C8 *p;

    U16Data = (U16Data | *(MessageBuffer+1)) << 8;
    U16Data = U16Data | *(MessageBuffer+0);

    HeaderData->SyncWordU16 = U16Data;
    HeaderData->TransmitterIdU8 = *(MessageBuffer+2);
    HeaderData->MessageCounterU8 = *(MessageBuffer+3);
    HeaderData->AckReqProtVerU8 = *(MessageBuffer+4);

    U16Data = 0;
    U16Data = (U16Data | *(MessageBuffer+6)) << 8;
    U16Data = U16Data | *(MessageBuffer+5);
    HeaderData->MessageIdU16 = U16Data;

    U32Data = (U32Data | *(MessageBuffer+10)) << 8;
    U32Data = (U32Data | *(MessageBuffer+9)) << 8;
    U32Data = (U32Data | *(MessageBuffer+8)) << 8;
    U32Data = U32Data | *(MessageBuffer+7);
    HeaderData->MessageLengthU32 = U32Data;

    if(Debug)
    {
      LogPrint("SyncWordU16 = 0x%x", HeaderData->SyncWordU16);
      LogPrint("TransmitterIdU8 = 0x%x", HeaderData->TransmitterIdU8);
      LogPrint("MessageCounterU8 = 0x%x", HeaderData->MessageCounterU8);
      LogPrint("AckReqProtVerU8 = 0x%x", HeaderData->AckReqProtVerU8);
      LogPrint("MessageIdU16 = 0x%x", HeaderData->MessageIdU16);
      LogPrint("MessageLengthU32 = 0x%x", HeaderData->MessageLengthU32);
    }

    return 0;
}
