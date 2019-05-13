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
#include <signal.h>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>

#include "timecontrol.h"
#include "logger.h"
#include "logging.h"


#define TIME_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define TIME_CONTROL_HOSTNAME_BUFFER_SIZE 20
#define TIME_CONTROL_RECEIVE_BUFFER_SIZE 54
#define TIME_CONTROL_TASK_PERIOD_MS 1
#define TIME_INTERVAL_NUMBER_BYTES 4
#define REPLY_TIMEOUT_S 3

#define SLEEP_TIME_GPS_CONNECTED_S 0
#define SLEEP_TIME_GPS_CONNECTED_NS 500000000
#define SLEEP_TIME_NO_GPS_CONNECTED_S 1
#define SLEEP_TIME_NO_GPS_CONNECTED_NS 0

#define LOG_PATH "./timelog/"
#define LOG_FILE "time.log"
#define LOG_BUFFER_LENGTH 128

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

static int TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static int TimeControlSendUDPData(int* sockfd, struct sockaddr_in* addr, C8* SendData, int Length, char debug);
//static void TimeControlRecvTime(int* sockfd, char* buffer, int length, int* recievedNewData);
static void TimeControlRecvTime(int* sockfd, C8* buffer, int length, int* recievedNewData);
U32 TimeControlIPStringToInt(C8 *IP);
U16 TimeControlGetMillisecond(TimeType *GPSTime);
static void TimeControlDecodeTimeBuffer(TimeType* GPSTime, C8* TimeBuffer, C8 debug);


/*------------------------------------------------------------
  -- Private variables.
  ------------------------------------------------------------*/
#define MODULE_NAME "TimeControl"
static const LOG_LEVEL logLevel = LOG_LEVEL_INFO;


/*------------------------------------------------------------
-- SigInt handler function.
------------------------------------------------------------*/
void sig_handlerTimeControl(int signo)
  {
    if (signo == SIGINT)
          printf("received SIGINT in timecontrol\n");
          printf("Shutting down timecontrol\n");
          exit(1);

    if (signo == SIGUSR1)
          printf("received SIGUSR1\n");
    if (signo == SIGKILL)
          printf("received SIGKILL\n");
    if (signo == SIGSTOP)
          printf("received SIGSTOP\n");
  }

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int timecontrol_task(TimeType *GPSTime, GSDType *GSD)
{

    C8 TextBufferC8[TIME_CONTROL_HOSTNAME_BUFFER_SIZE];
    C8 ServerIPC8[TIME_CONTROL_HOSTNAME_BUFFER_SIZE];
    U16 ServerPortU16;
    I32 SocketfdI32=-1;
    struct sockaddr_in time_addr;

    I32 iExit = 0, iCommand, result;
    C8 TimeBuffer[TIME_CONTROL_RECEIVE_BUFFER_SIZE];
    C8 LogBuffer[LOG_BUFFER_LENGTH];
    I32 ReceivedNewData, i;
    C8 SendData[TIME_INTERVAL_NUMBER_BYTES] = {0, 0, 3, 0xe8};
    //C8 SendData[4] = {0, 0, 0, 1};
    struct timespec sleep_time, ref_time;
    C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
    struct timeval tv, ExecTime;
    struct tm *tm;

    U32 IpU32;
    U8 PrevSecondU8;
    U16 CurrentMilliSecondU16, PrevMilliSecondU16;
    U8 CycleCount = 0;

    LogInit(MODULE_NAME,logLevel);
    LogMessage(LOG_LEVEL_INFO,"Time control task running with PID: %i",getpid());

    GPSTime->isGPSenabled = 0;

    gettimeofday(&ExecTime, NULL);
    CurrentMilliSecondU16 = (U16) (ExecTime.tv_usec / 1000);
    PrevMilliSecondU16 = CurrentMilliSecondU16;

    // Search .conf file for time server IP
    bzero(TextBufferC8, TIME_CONTROL_HOSTNAME_BUFFER_SIZE);
    UtilSearchTextFile(TEST_CONF_FILE, "TimeServerIP=", "", TextBufferC8);
    bzero(ServerIPC8, TIME_CONTROL_HOSTNAME_BUFFER_SIZE);
    strcat(ServerIPC8, TextBufferC8);
    IpU32 = TimeControlIPStringToInt(ServerIPC8);

    // Search .conf file for time server port
    bzero(TextBufferC8, TIME_CONTROL_HOSTNAME_BUFFER_SIZE);
    UtilSearchTextFile(TEST_CONF_FILE, "TimeServerPort=", "", TextBufferC8);
    ServerPortU16 = (U16)atoi(TextBufferC8);

    // If time server is specified, connect to it
    if(IpU32 != 0)
    {
        LogMessage(LOG_LEVEL_INFO,"Connecting to time server...");

        if (TimeControlCreateTimeChannel(ServerIPC8, ServerPortU16, &SocketfdI32, &time_addr))
        {
            LogMessage(LOG_LEVEL_INFO, "Established connection to time server");
            TimeControlSendUDPData(&SocketfdI32, &time_addr, SendData, TIME_INTERVAL_NUMBER_BYTES, 0);
            GPSTime->isGPSenabled = 1;
        }
        else
        {
            LogMessage(LOG_LEVEL_WARNING, "Unable to connect to time server: defaulting to system time");

            // Send warning over MQ
            LOG_SEND(LogBuffer, "Unable to connect to time server");
        }

    }

    if (!GPSTime->isGPSenabled) {
        LogMessage(LOG_LEVEL_INFO,"Initializing with system time");

        gettimeofday(&tv, NULL);

        GPSTime->MicroSecondU16 = 0;
        GPSTime->GPSMillisecondsU64 = tv.tv_sec*1000 + tv.tv_usec/1000 - MS_TIME_DIFF_UTC_GPS + MS_LEAP_SEC_DIFF_UTC_GPS;
        GPSTime->GPSWeekU16 = (U16)(GPSTime->GPSMillisecondsU64 / WEEK_TIME_MS);
        GPSTime->GPSSecondsOfWeekU32 = (U32)((GPSTime->GPSMillisecondsU64 - (U64)(GPSTime->GPSWeekU16) * WEEK_TIME_MS) / 1000);
        GPSTime->GPSSecondsOfDayU32 = (GPSTime->GPSMillisecondsU64 % DAY_TIME_MS) / 1000;
        GPSTime->GPSMinutesU32 = (GPSTime->GPSSecondsOfDayU32 / 60) % 60;
        GPSTime->isTimeInitializedU8 = 1;
    }

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

        if(GPSTime->isGPSenabled)
        {
            bzero(TimeBuffer,TIME_CONTROL_RECEIVE_BUFFER_SIZE);
            TimeControlRecvTime(&SocketfdI32, TimeBuffer, TIME_CONTROL_RECEIVE_BUFFER_SIZE, &ReceivedNewData);

            if(ReceivedNewData) TimeControlDecodeTimeBuffer(GPSTime, TimeBuffer, 0);
        }
        else if(!GPSTime->isGPSenabled)
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
            if(GPSTime->isGPSenabled)
            {
                SendData[0] = 0;
                SendData[1] = 0;
                SendData[2] = 0;
                SendData[3] = 0;
                TimeControlSendUDPData(&SocketfdI32, &time_addr, SendData, TIME_INTERVAL_NUMBER_BYTES, 0);
            }
            iExit = 1;
            LogMessage(LOG_LEVEL_INFO,"Time control exiting");
            (void)iCommClose();
        }

        if(ReceivedNewData && GPSTime->isGPSenabled)
        {
            /* Make call periodic */
            sleep_time.tv_sec = SLEEP_TIME_GPS_CONNECTED_S;
            sleep_time.tv_nsec = SLEEP_TIME_GPS_CONNECTED_NS;
            nanosleep(&sleep_time,&ref_time);
        }
        else if (!GPSTime->isGPSenabled)
        {
            sleep_time.tv_sec = SLEEP_TIME_NO_GPS_CONNECTED_S;
            sleep_time.tv_nsec = SLEEP_TIME_NO_GPS_CONNECTED_NS;
            nanosleep(&sleep_time,&ref_time);
        }
    }
}

U16 TimeControlGetMillisecond(TimeType *GPSTime)
{
    struct timeval now;
    U16 MilliU16 = 0, NowU16 = 0;
    gettimeofday(&now, NULL);
    NowU16 = (U16)(now.tv_usec / 1000);
    //if(NowU16 >= GPSTime->LocalMillisecondU16) MilliU16 = NowU16 - GPSTime->LocalMillisecondU16;
    //else if(NowU16 < GPSTime->LocalMillisecondU16) MilliU16 = 1000 + ((I16)NowU16 - (I16)GPSTime->LocalMillisecondU16);

    if(NowU16 >= GPSTime->LocalMillisecondU16) MilliU16 = NowU16 - GPSTime->LocalMillisecondU16;
    else if(NowU16 < GPSTime->LocalMillisecondU16) MilliU16 = 1000 - GPSTime->LocalMillisecondU16 + NowU16;

    //printf("Result= %d, now= %d, local= %d \n", MilliU16, NowU16, GPSTime->LocalMillisecondU16);
    return MilliU16;
}

static int TimeControlCreateTimeChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
    int result;
    struct hostent *object;
    C8 packetIntervalMs[TIME_INTERVAL_NUMBER_BYTES] = {0,0,0,100}; // Make server send with this interval while waiting for first reply
    C8 timeBuffer[TIME_CONTROL_RECEIVE_BUFFER_SIZE];
    int receivedNewData = 0;
    struct timeval timeout = {REPLY_TIMEOUT_S, 0};
    struct timeval tEnd,tCurr;

    LogMessage(LOG_LEVEL_INFO,"Time source address: %s:%d",name,port);
    /* Connect to object safety socket */

    *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
    if (*sockfd < 0)
    {
        util_error("Failed to connect to time socket");
    }

    /* Set address to object */
    object = gethostbyname(name);

    if (object==NULL)
    {
        util_error("Unknown host");
    }

    bcopy((char *) object->h_addr,
          (char *)&addr->sin_addr.s_addr, object->h_length);
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);


    if (setsockopt (*sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0) util_error("Setsockopt failed");



    /* set socket to non-blocking */
    result = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
    if (result < 0)
    {
        util_error("Error calling fcntl");
    }
    LogMessage(LOG_LEVEL_INFO,"Created socket and time address: %s:%d",name,port);

    // Check for existence of remote server
    LogMessage(LOG_LEVEL_INFO,"Awaiting reply from time server...");
    // Set send interval to be as short as possible to minimise wait for reply
    TimeControlSendUDPData(sockfd, addr, packetIntervalMs, TIME_INTERVAL_NUMBER_BYTES, 0);

    // Set time to stop waiting for reply
    gettimeofday(&tEnd, NULL);
    timeradd(&tEnd,&timeout,&tEnd);

    do
    {
        gettimeofday(&tCurr, NULL);
        TimeControlRecvTime(sockfd, timeBuffer, TIME_CONTROL_RECEIVE_BUFFER_SIZE, &receivedNewData);
    } while (!receivedNewData && timercmp(&tCurr,&tEnd,<));

    return receivedNewData;
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


static int TimeControlSendUDPData(int* sockfd, struct sockaddr_in* addr, C8* SendData, int Length, char debug)
{
    int result, i;

    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));


    if(debug)
    {
        // TODO: Change to log write when bytes thingy has been implemented
        for(i = 0;i < Length; i ++) printf("[%d]=%x ", i, (C8)*(SendData+i));
        printf("\n");
    }

    if (result < 0)
    {
        util_error("Failed to send on time socket");
    }

    return 0;
}


static void TimeControlRecvTime(int* sockfd, C8* buffer, int length, int* recievedNewData)
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
                util_error("Failed to receive from time socket");
            }
            else
            {
                LogMessage(LOG_LEVEL_DEBUG,"No data received, result=%d", result);
            }
        }
        else
        {
            *recievedNewData = 1;
            LogMessage(LOG_LEVEL_DEBUG,"Received data: <%s>, result=%d",buffer, result);

        }
    } while(result > 0 );
}

static void TimeControlDecodeTimeBuffer(TimeType* GPSTime, C8* TimeBuffer, C8 debug)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

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
    GPSTime->FixQualityU8 = TimeBuffer[52];
    GPSTime->NSatellitesU8 = TimeBuffer[53];

    gettimeofday(&tv, NULL);

    GPSTime->LocalMillisecondU16 = (U16) (tv.tv_usec / 1000);

    GPSTime->isTimeInitializedU8 = 1;

    if (debug)
    {
        //TimeControlGetMillisecond(GPSTime);
        //LogPrintBytes(TimeBuffer,0,TIME_CONTROL_RECEIVE_BUFFER_SIZE);
        //LogPrint("ProtocolVersionU8: %d", GPSTime->ProtocolVersionU8);
        //LogPrint("YearU16: %d", GPSTime->YearU16);
        //LogPrint("MonthU8: %d", GPSTime->MonthU8);
        //LogPrint("DayU8: %d", GPSTime->DayU8);
        LogPrint("Time: %d:%d:%d", GPSTime->HourU8, GPSTime->MinuteU8, GPSTime->SecondU8);
        //LogPrint("MinuteU8: %d", GPSTime->MinuteU8);
        //LogPrint("SecondU8: %d", GPSTime->SecondU8);
        //LogPrint("MillisecondU16: %d", GPSTime->MillisecondU16);
        //LogPrint("SecondCounterU32: %d", GPSTime->SecondCounterU32);
        //LogPrint("GPSMillisecondsU64: %ld", GPSTime->GPSMillisecondsU64);
        //LogPrint("GPSMinutesU32: %d", GPSTime->GPSMinutesU32);
        //LogPrint("GPSWeekU16: %d", GPSTime->GPSWeekU16);
        //LogPrint("GPSSecondsOfWeekU32: %d", GPSTime->GPSSecondsOfWeekU32);
        //LogPrint("GPSSecondsOfDayU32: %d", GPSTime->GPSSecondsOfDayU32);
        //LogPrint("ETSIMillisecondsU64: %ld", GPSTime->ETSIMillisecondsU64);
        //LogPrint("LatitudeU32: %d", GPSTime->LatitudeU32);
        //LogPrint("LongitudeU32: %d", GPSTime->LongitudeU32);
        //LogPrint("LocalMillisecondU16: %d", GPSTime->LocalMillisecondU16);
        //LogPrint("FixQualityU8: %d", GPSTime->FixQualityU8);
        //LogPrint("NSatellitesU8: %d", GPSTime->NSatellitesU8);
    }
}
