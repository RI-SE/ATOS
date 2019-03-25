/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 MAESTRO project
  ------------------------------------------------------------------------------
  -- File        : timecontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : MAESTRO
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>  


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>

#include "util.h"
#include "logger.h"


#define TIME_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define TIME_CONTROL_BUFFER_SIZE_20 20
#define TIME_CONTROL_BUFFER_SIZE_54 54
#define TIME_CONTROL_TASK_PERIOD_MS 1
#define LOG_PATH "./timelog/"
#define LOG_FILE "time.log"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

static void TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static int TimeControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);
//static void TimeControlRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData);
static void TimeControlRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData);
U32 TimeControlIPStringToInt(C8 *IP);
U16 TimeControlGetMillisecond(TimeType *GPSTime);



/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int timecontrol_task(TimeType *GPSTime, GSDType *GSD)
{

  C8 TextBufferC8[TIME_CONTROL_BUFFER_SIZE_20];
  C8 ServerIPC8[TIME_CONTROL_BUFFER_SIZE_20];
  U16 ServerPortU16;
  I32 SocketfdI32=-1;
  struct sockaddr_in time_addr;
  
  I32 iExit = 0, iCommand, result;
  C8 TimeBuffer[TIME_CONTROL_BUFFER_SIZE_54];
  I32 ReceivedNewData, i;
  C8 SendData[4] = {0, 0, 3, 0xe8};
  //C8 SendData[4] = {0, 0, 0, 1};
  struct timespec sleep_time, ref_time;
  C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  struct timeval tv, ExecTime;
  struct tm *tm;
  
  U32 IpU32;
  U8 PrevSecondU8;
  U16 CurrentMilliSecondU16, PrevMilliSecondU16;
  U8 CycleCount = 0;

  gettimeofday(&ExecTime, NULL);
  CurrentMilliSecondU16 = (U16) (ExecTime.tv_usec / 1000);
  PrevMilliSecondU16 = CurrentMilliSecondU16;

  bzero(TextBufferC8, TIME_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "TimeServerIP=", "", TextBufferC8);
  bzero(ServerIPC8, TIME_CONTROL_BUFFER_SIZE_20);
  strcat(ServerIPC8, TextBufferC8);
  IpU32 = TimeControlIPStringToInt(ServerIPC8);
  

  if(IpU32 == 0)
  {
    gettimeofday(&tv, NULL);

    GPSTime->MicroSecondU16 = 0;
    GPSTime->GPSMillisecondsU64 = tv.tv_sec*1000 + tv.tv_usec/1000 - MS_TIME_DIFF_UTC_GPS + MS_LEAP_SEC_DIFF_UTC_GPS;
    GPSTime->GPSWeekU16 = (U16)(GPSTime->GPSMillisecondsU64 / WEEK_TIME_MS);
    GPSTime->GPSSecondsOfWeekU32 = (U32)((GPSTime->GPSMillisecondsU64 - (U64)(GPSTime->GPSWeekU16) * WEEK_TIME_MS) / 1000);
    GPSTime->GPSSecondsOfDayU32 = (GPSTime->GPSMillisecondsU64 % DAY_TIME_MS) / 1000;
    GPSTime->GPSMinutesU32 = (GPSTime->GPSSecondsOfDayU32 / 60) % 60;
    GPSTime->isGPSenabled = 0;
    GPSTime->TimeInitiatedU8 = 1;
  }

  bzero(TextBufferC8, TIME_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "TimeServerPort=", "", TextBufferC8);
  ServerPortU16 = (U16)atoi(TextBufferC8);
  if(IpU32 != 0)
  {
    TimeControlCreateTimeChannel(ServerIPC8, ServerPortU16, &SocketfdI32, &time_addr);
    TimeControlSendUDPData(&SocketfdI32, &time_addr, SendData, 4, 0);
    GPSTime->isGPSenabled = 1;
    printf("[TimeControl] Get time from GPS.\n");
  } else printf("[TimeControl] Count fake time.\n");


  while(!iExit)
  {

    gettimeofday(&ExecTime, NULL);
    CurrentMilliSecondU16 = (U16) (ExecTime.tv_usec / 1000);
    if(CurrentMilliSecondU16 < PrevMilliSecondU16)
    { 
      GSD->TimeControlExecTimeU16 = CurrentMilliSecondU16 + (1000 - PrevMilliSecondU16);
      //printf("%d\n", GSD->TimeControlExecTimeU16);
    }
    else 
    {
      GSD->TimeControlExecTimeU16 = abs(PrevMilliSecondU16 - CurrentMilliSecondU16);
      //printf("%d\n", GSD->TimeControlExecTimeU16);
    }
    PrevMilliSecondU16 = CurrentMilliSecondU16;

    if(IpU32 != 0)
    {
      bzero(TimeBuffer,TIME_CONTROL_BUFFER_SIZE_54);
      TimeControlRecvTime(&SocketfdI32, TimeBuffer, TIME_CONTROL_BUFFER_SIZE_54, &ReceivedNewData);
    }
    
    if(ReceivedNewData && IpU32 != 0)
    {
      //for(i=0; i < TIME_CONTROL_BUFFER_SIZE_54; i++) printf("%x-", TimeBuffer[i]);
      //printf("\n");
      GPSTime->LockedU8 = 1;
      GPSTime->TimeInitiatedU8 = 1;
      GPSTime->ProtocolVersionU8 = TimeBuffer[0];
      GPSTime->YearU16 = ((U16)TimeBuffer[1]) << 8 | TimeBuffer[2];
      GPSTime->MonthU8 = TimeBuffer[3];
      GPSTime->DayU8 = TimeBuffer[4];
      GPSTime->HourU8 = TimeBuffer[5];
      GPSTime->MinuteU8 = TimeBuffer[6];
      GPSTime->SecondU8 = TimeBuffer[7];
      GPSTime->MillisecondU16 = ((U16)TimeBuffer[8]) << 8 | TimeBuffer[9];
      GPSTime->MicroSecondU16 = 0;
      GPSTime->SecondCounterU32 = ((U32)TimeBuffer[10]) << 24 | ((U32)TimeBuffer[11]) << 16 | ((U32)TimeBuffer[12]) << 8 | TimeBuffer[13];
      GPSTime->GPSMillisecondsU64 = ((U64)TimeBuffer[14]) << 56 | ((U64)TimeBuffer[15]) << 48 | ((U64)TimeBuffer[16]) << 40 | ((U64)TimeBuffer[17]) << 32 |
                                ((U64)TimeBuffer[18]) << 24 | ((U64)TimeBuffer[19]) << 16 | ((U64)TimeBuffer[20]) << 8 | TimeBuffer[21];
      GPSTime->GPSMinutesU32 = ((U32)TimeBuffer[22]) << 24 | ((U32)TimeBuffer[23]) << 16 | ((U32)TimeBuffer[24]) << 8 | TimeBuffer[25];
      GPSTime->GPSWeekU16 = ((U16)TimeBuffer[26]) << 8 | TimeBuffer[27];
      GPSTime->GPSSecondsOfWeekU32 = (((U32)TimeBuffer[28]) << 24 | ((U32)TimeBuffer[29]) << 16 | ((U32)TimeBuffer[30]) << 8 | TimeBuffer[31]) + 18;
      GPSTime->GPSSecondsOfDayU32 = ((U32)TimeBuffer[32]) << 24 | ((U32)TimeBuffer[33]) << 16 | ((U32)TimeBuffer[34]) << 8 | TimeBuffer[35];
      GPSTime->ETSIMillisecondsU64 = ((U64)TimeBuffer[36]) << 56 | ((U64)TimeBuffer[37]) << 48 | ((U64)TimeBuffer[38]) << 40 | ((U64)TimeBuffer[39]) << 32 |
                                ((U64)TimeBuffer[40]) << 24 | ((U64)TimeBuffer[41]) << 16 | ((U64)TimeBuffer[42]) << 8 | TimeBuffer[43];
      GPSTime->LatitudeU32 = ((U32)TimeBuffer[44]) << 24 | ((U32)TimeBuffer[45]) << 16 | ((U32)TimeBuffer[46]) << 8 | TimeBuffer[47];
      GPSTime->LongitudeU32 = ((U32)TimeBuffer[48]) << 24 | ((U32)TimeBuffer[49]) << 16 | ((U32)TimeBuffer[50]) << 8 | TimeBuffer[51];
      GPSTime->FixQualityU8 = TimeBuffer[52];
      GPSTime->NSatellitesU8 = TimeBuffer[53];

      gettimeofday(&tv, NULL);

      GPSTime->LocalMillisecondU16 = (U16) (tv.tv_usec / 1000);
      
      GPSTime->LockedU8 = 0;
      //TimeControlGetMillisecond(GPSTime);
      //printf("ProtocolVersionU8: %d\n", GPSTime->ProtocolVersionU8);
      //printf("YearU16: %d\n", GPSTime->YearU16);
      //printf("MonthU8: %d\n", GPSTime->MonthU8);
      //printf("DayU8: %d\n", GPSTime->DayU8);
      //printf("HourU8: %d\n", GPSTime->HourU8);
      //printf("MinuteU8: %d\n", GPSTime->MinuteU8);
      //printf("SecondU8: %d\n", GPSTime->SecondU8);
      //printf("MillisecondU16: %d\n", GPSTime->MillisecondU16);
      //printf("SecondCounterU32: %d\n", GPSTime->SecondCounterU32);
      //printf("GPSMillisecondsU64: %ld\n", GPSTime->GPSMillisecondsU64);
      //printf("GPSMinutesU32: %d\n", GPSTime->GPSMinutesU32);
      //printf("GPSWeekU16: %d\n", GPSTime->GPSWeekU16);
      //printf("GPSSecondsOfWeekU32: %d\n", GPSTime->GPSSecondsOfWeekU32);
      //printf("GPSSecondsOfDayU32: %d\n", GPSTime->GPSSecondsOfDayU32);
      //printf("ETSIMillisecondsU64: %ld\n", GPSTime->ETSIMillisecondsU64);
      //printf("LatitudeU32: %d\n", GPSTime->LatitudeU32);
      //printf("LongitudeU32: %d\n", GPSTime->LongitudeU32);
      //printf("LocalMillisecondU16: %d\n", GPSTime->LocalMillisecondU16);
      //printf("FixQualityU8: %d\n", GPSTime->FixQualityU8);
      //printf("NSatellitesU8: %d\n", GPSTime->NSatellitesU8);
    }
    else if(IpU32 == 0)
    {
      gettimeofday(&tv, NULL);

      tm = localtime(&tv.tv_sec);

      // Add 1900 to get the right year value
      GPSTime->YearU16 =  (U16)tm->tm_year + 1900;
      // Months are 0 based in struct tm
      GPSTime->MonthU8 =  (U8)tm->tm_mon + 1;
      GPSTime->DayU8 = (U8)tm->tm_mday;
      GPSTime->HourU8 = (U8)tm->tm_hour;
      GPSTime->MinuteU8 = (U8)tm->tm_min;
      GPSTime->SecondU8 = (U8)tm->tm_sec;
      GPSTime->MillisecondU16 = (U16) (tv.tv_usec / 1000);
      
      GPSTime->LocalMillisecondU16 = (U16) (tv.tv_usec / 1000);

      GPSTime->GPSMillisecondsU64 = GPSTime->GPSMillisecondsU64 + 1000;
      
      if(GPSTime->SecondU8 != PrevSecondU8)
      {
        PrevSecondU8 = GPSTime->SecondU8;
        GPSTime->SecondCounterU32 ++;
        if(GPSTime->GPSSecondsOfDayU32 >= 86400) GPSTime->GPSSecondsOfDayU32 = 0;
        else GPSTime->GPSSecondsOfDayU32 ++;
        
        if(GPSTime->GPSSecondsOfWeekU32 >= 604800)
        {
          GPSTime->GPSSecondsOfWeekU32 = 0;
          GPSTime->GPSWeekU16 ++;
        } else GPSTime->GPSSecondsOfWeekU32 ++;

        if(GPSTime->SecondCounterU32 % 60 == 0) GPSTime->GPSMinutesU32 ++;
      }
    }

    if(GSD->ExitU8 == 1)
    {
      if(IpU32)
      {
        SendData[0] = 0, SendData[1] = 0, SendData[2] = 0, SendData[3] = 0;
        TimeControlSendUDPData(&SocketfdI32, &time_addr, SendData, 4, 0);
      }
      iExit = 1;
      printf("[TimeControl] Timecontrol exiting.\n");
      (void)iCommClose();
    }

    if(ReceivedNewData == 1 && IpU32 != 0)
    {
       /* Make call periodic */
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = 500000000;
      nanosleep(&sleep_time,&ref_time);
    }
    else if (IpU32 == 0)
    {
      sleep_time.tv_sec = 1;
      sleep_time.tv_nsec = 0;//100000000;
      nanosleep(&sleep_time,&ref_time); 
    }
  }
}

U16 TimeControlGetMillisecond(TimeType *GPSTime)
{
  struct timeval now;
  U16 MilliU16 = 0, NowU16 = 0;
  while(GPSTime->LockedU8 == 1);
  gettimeofday(&now, NULL);
  NowU16 = (U16)(now.tv_usec / 1000);
  //if(NowU16 >= GPSTime->LocalMillisecondU16) MilliU16 = NowU16 - GPSTime->LocalMillisecondU16;
  //else if(NowU16 < GPSTime->LocalMillisecondU16) MilliU16 = 1000 + ((I16)NowU16 - (I16)GPSTime->LocalMillisecondU16);

  if(NowU16 >= GPSTime->LocalMillisecondU16) MilliU16 = NowU16 - GPSTime->LocalMillisecondU16;
  else if(NowU16 < GPSTime->LocalMillisecondU16) MilliU16 = 1000 - GPSTime->LocalMillisecondU16 + NowU16;

  //printf("Result= %d, now= %d, local= %d \n", MilliU16, NowU16, GPSTime->LocalMillisecondU16);
  return MilliU16;
}

static void TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
  int result;
  struct hostent *object;

  DEBUG_LPRINT(DEBUG_LEVEL_MEDIUM,"[TimeControl] Time source IP: %s, port: %d\n", name, port);
  /* Connect to object safety socket */

  *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
  {
    util_error("[TimeControl] ERR: Failed to connect to time socket");
  }

  /* Set address to object */
  object = gethostbyname(name);
  
  if (object==0)
  {
    util_error("[TimeControl] ERR: Unknown host");
  }

  bcopy((char *) object->h_addr, 
    (char *)&addr->sin_addr.s_addr, object->h_length);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);


  struct timeval timeout;      
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  if (setsockopt (*sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) util_error("[TimeControl] Setsockopt failed\n");


  
   /* set socket to non-blocking */
   result = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  if (result < 0)
  {
    util_error("[TimeControl] ERR: calling fcntl");
  }
  DEBUG_LPRINT(DEBUG_LEVEL_MEDIUM,"[TimeControl] Created socket and time address: %s %d\n",name,port);
}


U32 TimeControlIPStringToInt(C8 *IP)
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


static int TimeControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug)
{
    int result, i;
 
    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));

  
    if(debug)
    {
      for(i = 0;i < Length; i ++) printf("[%d]=%x ", i, (C8)*(SendData+i));
      printf("\n");
    }

    if (result < 0)
    {
      util_error("[TimeControl] ERR: Failed to send on time socket");
    }

    return 0;
}


static void TimeControlRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData)
{
  int result;
  *recievedNewData = 0;
    do
    {
      
      result = recv(*sockfd, buffer, length, 0);
      
      if (result < 0)
      {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
        {
          util_error("[TimeControl] ERR: Failed to receive from time socket");
        }
        else
        {

          DEBUG_LPRINT(DEBUG_LEVEL_LOW,"[TimeControl]  No data receive, result=%d\n", result);

        }
      }
      else
      {
        *recievedNewData = 1;
        DEBUG_LPRINT(DEBUG_LEVEL_LOW,"[TimeControl] Received data: <%s>, %d\n",buffer, result);

      }
    } while(result > 0 );
}
