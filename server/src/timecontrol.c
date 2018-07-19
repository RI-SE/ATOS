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
#define TIME_CONTROL_BUFFER_SIZE_52 52
#define TIME_CONTROL_TASK_PERIOD_MS 1
#define LOG_PATH "./timelog/"
#define LOG_FILE "time.log"

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

static void TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static int TimeControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);
static void TimeControlRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData);
U32 TimeControlIPStringToInt(C8 *IP);


/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int timecontrol_task(TimeType *GPSTime)
{

  C8 TextBufferC8[TIME_CONTROL_BUFFER_SIZE_20];
  C8 ServerIPC8[TIME_CONTROL_BUFFER_SIZE_20];
  U16 ServerPortU16;
  I32 SocketfdI32=-1;
  struct sockaddr_in time_addr;
  I32 iExit = 0, iCommand;
  C8 TimeBuffer[TIME_CONTROL_BUFFER_SIZE_52];
  I32 ReceivedNewData, i;
  C8 SendData[4] = {0, 0, 0, 250};
  struct timespec sleep_time, ref_time;
  C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  struct timeval tv;
  struct tm *tm;
  
  U32 IpU32;
  U8 PrevSecondU8;
  U16 PrevMilliSecondU16;
  
  (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);



  bzero(TextBufferC8, TIME_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "TimeServerIP=", "", TextBufferC8);
  bzero(ServerIPC8, TIME_CONTROL_BUFFER_SIZE_20);
  strcat(ServerIPC8, TextBufferC8);
  IpU32 = TimeControlIPStringToInt(ServerIPC8);

  if(IpU32 == 0)
  {
    GPSTime->MicroSecondU16 = 0;
    GPSTime->GPSMillisecondsU64 = 0;
    GPSTime->GPSSecondsOfWeekU32 = 0;
    GPSTime->GPSMinutesU32 = 0;
    GPSTime->GPSWeekU16 = 2006;
  }

  bzero(TextBufferC8, TIME_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "TimeServerPort=", "", TextBufferC8);
  ServerPortU16 = (U16)atoi(TextBufferC8);
  if(IpU32 != 0)
  {
    TimeControlCreateTimeChannel(ServerIPC8, ServerPortU16, &SocketfdI32,  &time_addr);
    TimeControlSendUDPData(&SocketfdI32, &time_addr, SendData, 4, 0);
  }
  printf("Checking time...\n");
  while(!iExit)
  {

    if(IpU32 != 0)
    {
      bzero(TimeBuffer,TIME_CONTROL_BUFFER_SIZE_52);
      TimeControlRecvTime(&SocketfdI32, TimeBuffer, TIME_CONTROL_BUFFER_SIZE_52, &ReceivedNewData);
    }
    
    if(ReceivedNewData && IpU32 != 0)
    {
      //for(i=0; i < TIME_CONTROL_BUFFER_SIZE_52; i++) printf("%x-", TimeBuffer[i]);
      //printf("\n");
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
      GPSTime->GPSSecondsOfWeekU32 = ((U32)TimeBuffer[28]) << 24 | ((U32)TimeBuffer[29]) << 16 | ((U32)TimeBuffer[30]) << 8 | TimeBuffer[31];
      GPSTime->GPSSecondsOfDayU32 = ((U32)TimeBuffer[32]) << 24 | ((U32)TimeBuffer[33]) << 16 | ((U32)TimeBuffer[34]) << 8 | TimeBuffer[35];
      GPSTime->ETSIMillisecondsU64 = ((U64)TimeBuffer[36]) << 56 | ((U64)TimeBuffer[37]) << 48 | ((U64)TimeBuffer[38]) << 40 | ((U64)TimeBuffer[39]) << 32 |
                                ((U64)TimeBuffer[40]) << 24 | ((U64)TimeBuffer[41]) << 16 | ((U64)TimeBuffer[42]) << 8 | TimeBuffer[43];
      GPSTime->LatitudeU32 = ((U32)TimeBuffer[44]) << 24 | ((U32)TimeBuffer[45]) << 16 | ((U32)TimeBuffer[46]) << 8 | TimeBuffer[47];
      GPSTime->LongitudeU32 = ((U32)TimeBuffer[48]) << 24 | ((U32)TimeBuffer[49]) << 16 | ((U32)TimeBuffer[50]) << 8 | TimeBuffer[51];

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

    }
    else if( GPSTime->MicroSecondU16 == 0)
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
      
      if(GPSTime->MillisecondU16 != PrevMilliSecondU16)
      {

        if(GPSTime->MillisecondU16 < PrevMilliSecondU16)
        {
          GPSTime->GPSMillisecondsU64 = GPSTime->GPSMillisecondsU64 + GPSTime->MillisecondU16 + (1000 - PrevMilliSecondU16);
           //printf("< %ld, %d, %d, %d\n", GPSTime->GPSMillisecondsU64, (1000 - PrevMilliSecondU16), GPSTime->MillisecondU16, PrevMilliSecondU16);
        } 
        else
        {
          GPSTime->GPSMillisecondsU64 = GPSTime->GPSMillisecondsU64 + abs(PrevMilliSecondU16 - GPSTime->MillisecondU16);
          //printf("> %ld, %d, %d, %d\n", GPSTime->GPSMillisecondsU64, abs(GPSTime->MillisecondU16 - PrevMilliSecondU16), GPSTime->MillisecondU16, PrevMilliSecondU16);
        }
        PrevMilliSecondU16 = GPSTime->MillisecondU16;
      }

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


    bzero(MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
    (void)iCommRecv(&iCommand,MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);

    if(iCommand == COMM_EXIT)
    {
      iExit = 1;
      printf("timecontrol exiting.\n");
      (void)iCommClose();
    }
    
  }


}


static void TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
  int result;
  struct hostent *object;

  printf("Time source IP: %s, port: %d\n", name, port);
  /* Connect to object safety socket */
  #ifdef DEBUG
    printf("INF: Creating time socket\n");
    fflush(stdout);
  #endif

  *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
  {
    util_error("ERR: Failed to connect to time socket");
  }

  /* Set address to object */
  object = gethostbyname(name);
  
  if (object==0)
  {
    util_error("ERR: Unknown host");
  }

  bcopy((char *) object->h_addr, 
    (char *)&addr->sin_addr.s_addr, object->h_length);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);

   /* set socket to non-blocking */
  result = fcntl(*sockfd, F_SETFL, 
    fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  if (result < 0)
  {
    util_error("ERR: calling fcntl");
  }

  #ifdef DEBUG
    printf("INF: Created socket and time address: %s %d\n",name,port);
    fflush(stdout);
  #endif

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

      printf("IpU32 = %x\n", IpU32);
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
      util_error("ERR: Failed to send on time socket");
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
          util_error("ERR: Failed to receive from time socket");
        }
        else
        {
          #ifdef DEBUG
            printf("INF: No data receive, result=%d\n", result);
            fflush(stdout);
          #endif
        }
      }
      else
      {
        *recievedNewData = 1;
        #ifdef DEBUG
          printf("INF: Received: <%s>, %d\n",buffer, result);
          fflush(stdout);
        #endif
      }
    } while(result > 0 );
}
