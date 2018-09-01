/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2018 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : simulatorcontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS II
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
#include "timecontrol.h"


#define SIM_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define SIM_CONTROL_BUFFER_SIZE_20 20
#define SIM_CONTROL_BUFFER_SIZE_128 128
#define SIM_CONTROL_TASK_PERIOD_MS 1
#define SIM_CONTROL_HEARTBEAT_TIME_MS 10
#define SIM_CONTROL_LOG_BUFFER_LENGTH 128

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void SimulatorControlSendHeartbeat( I32 *Sockfd, struct sockaddr_in *Addr, TimeType *GPSTime, U8 Debug);
void SimulatorControlInitiateSimulator( I32 *Sockfd, U8 SimulatorMode, U8 Debug);
void SimulatorControlStartScenario( I32 *Sockfd, C8 *StartTime, U8 Debug);
void SimulatorControlSendMONR( I32 *Sockfd, struct sockaddr_in *Addr, C8 *MonrData, U8 Debug);
/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int simulatorcontrol_task(TimeType *GPSTime, GSDType *GSD)
{

  I32 iExit = 0, iCommand;
  C8 MqRecvBuffer[MQ_MAX_MESSAGE_LENGTH];
  (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);

  printf("Starting simulator control...\n");
  
  C8 TextBufferC8[SIM_CONTROL_BUFFER_SIZE_20];
  C8 SimulatorServerIPC8[SIM_CONTROL_BUFFER_SIZE_20];
  U32 SimulatorIpU32 = 0;
  U16 SimulatorTCPPortU16;
  I32 SimulatorTCPSocketfdI32=-1;
  U16 SimulatorUDPPortU16;
  I32 SimulatorUDPSocketfdI32=-1;

  struct sockaddr_in simulator_addr;
  
  I32 ClientResultI32;
  C8 ReceiveBuffer[SIM_CONTROL_BUFFER_SIZE_128];
  I32 ReceivedNewData, i;
  C8 SendData[4] = {0, 0, 3, 0xe8};
  struct timespec sleep_time, ref_time;
  struct timeval tv, ExecTime;
  struct tm *tm;
  

  U8 PrevSecondU8;
  U16 CurrentMilliSecondU16, PrevMilliSecondU16;
  U16 CycleU16;
  U8 ServerStatusU8 = 0;
  U8 SimulatorInitiatedU8 = 0;
  C8 *MiscPtr;
  U64 StartTimeU64;
  C8 LogBuffer[SIM_CONTROL_LOG_BUFFER_LENGTH];
  C8 Timestamp[SIM_CONTROL_BUFFER_SIZE_20];

  gettimeofday(&ExecTime, NULL);
  CurrentMilliSecondU16 = (U16) (ExecTime.tv_usec / 1000);
  PrevMilliSecondU16 = CurrentMilliSecondU16;

  bzero(TextBufferC8, SIM_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "SimulatorIP=", "", TextBufferC8);
  bzero(SimulatorServerIPC8, SIM_CONTROL_BUFFER_SIZE_20);
  strcat(SimulatorServerIPC8, TextBufferC8);
  printf("IP: %s\n", TextBufferC8);
  SimulatorIpU32 = UtilIPStringToInt(SimulatorServerIPC8);


  if(SimulatorIpU32 != 0)
  {
    bzero(TextBufferC8, SIM_CONTROL_BUFFER_SIZE_20);
    UtilSearchTextFile(TEST_CONF_FILE, "SimulatorTCPPort=", "", TextBufferC8);
    SimulatorTCPPortU16 = (U16)atoi(TextBufferC8);
    bzero(TextBufferC8, SIM_CONTROL_BUFFER_SIZE_20);
    UtilSearchTextFile(TEST_CONF_FILE, "SimulatorUDPPort=", "", TextBufferC8);
    SimulatorUDPPortU16 = (U16)atoi(TextBufferC8);
        
    printf("SimulatorTCPPort = %d\n", SimulatorTCPPortU16);
    printf("SimulatorUDPPort = %d\n", SimulatorUDPPortU16);

    while(!iExit)
    {


      if(SimulatorTCPSocketfdI32 <= 0)
      {
        ClientResultI32 =  UtilConnectTCPChannel("SimulatorControl", &SimulatorTCPSocketfdI32, (const C8*)SimulatorServerIPC8, SimulatorTCPPortU16);
        UtilCreateUDPChannel("SimulatorControl", &SimulatorUDPSocketfdI32, (const C8*)SimulatorServerIPC8, SimulatorUDPPortU16, &simulator_addr);
      }
      else
      {
        bzero(ReceiveBuffer, SIM_CONTROL_BUFFER_SIZE_128);
        ClientResultI32 = UtilReceiveTCPData("SimulatorControl", &SimulatorTCPSocketfdI32, ReceiveBuffer, 1);

        if(ClientResultI32 == 0)
        {
            DEBUG_LPRINT(DEBUG_LEVEL_HIGH, "[SimulatorControl] Client closed connection.\n");
            close(SimulatorTCPSocketfdI32);
            SimulatorTCPSocketfdI32 = -1;
            SimulatorInitiatedU8 = 0;
        }
        
        if(SimulatorInitiatedU8 == 0 && SimulatorTCPSocketfdI32 > 0)
        {
          
          SimulatorControlInitiateSimulator(&SimulatorTCPSocketfdI32, 1, 0);
          SimulatorInitiatedU8 = 1;
        }
        
        if(ClientResultI32 > 0)
        {
          //Do something with the data
          printf("[SimulatorControl] %s\n", ReceiveBuffer);
        }

        bzero(ReceiveBuffer, SIM_CONTROL_BUFFER_SIZE_128);
        UtilReceiveUDPData("SimulatorControl", &SimulatorUDPSocketfdI32, ReceiveBuffer, 100, &ReceivedNewData, 0);

        if(ReceivedNewData)
        {
          //printf("VOIP recevied by simulatorcontrol!\n");

          for(i = 0; i < ReceiveBuffer[3] + 4; i++) GSD->VOILData[i] = (U8)ReceiveBuffer[i];
          
          (void)iCommSend(COMM_VIOP,ReceiveBuffer); 
          ReceivedNewData = 0;         

        }


        if(CycleU16 == 0)
        {
          SimulatorControlSendHeartbeat( &SimulatorUDPSocketfdI32, &simulator_addr, GPSTime, 0);
        }

      }

      if(GSD->ScenarioStartTimeU32 != 0)
      {
        
        bzero(Timestamp, SIM_CONTROL_BUFFER_SIZE_20);
        sprintf(Timestamp, "%" PRIu32, GSD->ScenarioStartTimeU32);
        LOG_SEND(LogBuffer, "[SimulatorControl] Sending StartScenario(%s)\n", Timestamp);
        SimulatorControlStartScenario( &SimulatorTCPSocketfdI32, Timestamp, 1);
        GSD->ScenarioStartTimeU32 = 0;
      }


      bzero(MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
      (void)iCommRecv(&iCommand,MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);

      
      if(iCommand == COMM_EXIT)
      {
        iExit = 1;  
        printf("simulatorcontrol exiting.\n");
        (void)iCommClose();
      } 
      else if(iCommand == COMM_MONI)
      {
        //printf("Monr sim %s\n", MqRecvBuffer);
        SimulatorControlSendMONR( &SimulatorUDPSocketfdI32, &simulator_addr, MqRecvBuffer, 0);
      }

       ++CycleU16;
      if(CycleU16 >= SIM_CONTROL_HEARTBEAT_TIME_MS/SIM_CONTROL_TASK_PERIOD_MS) CycleU16 = 0;
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = SIM_CONTROL_TASK_PERIOD_MS*1000000;
      (void)nanosleep(&sleep_time,&ref_time);


    }
  }
}




void SimulatorControlStartScenario( I32 *Sockfd, C8 *StartTime, U8 Debug)
{

  C8 SendData[SIM_CONTROL_BUFFER_SIZE_128];
  C8 SendLength[4] = {0,0,0,0};
  I32 Length = 0;
  bzero(SendData, SIM_CONTROL_BUFFER_SIZE_128);
  strcat(SendData,"StartScenario(");
  strcat(SendData, StartTime);
  strcat(SendData, ")");
  
  Length = (I32)(strlen(SendData));
  SendLength[3] = (U8)Length;
  UtilSendTCPData("SimulatorControl", (const C8*)SendLength, 4, Sockfd, Debug);
  UtilSendTCPData("SimulatorControl", (const C8*)SendData, Length, Sockfd, Debug);

}

void SimulatorControlInitiateSimulator( I32 *Sockfd, U8 SimulatorMode, U8 Debug)
{

 
  C8 SendData[SIM_CONTROL_BUFFER_SIZE_128];
  C8 SendLength[4] = {0,0,0,0};
  C8 Mode[1] ={0};
  I32 Length = 0;
  bzero(SendData, SIM_CONTROL_BUFFER_SIZE_128);
  strcat(SendData,"InitSimulator(");
  
  sprintf(Mode, "%" PRIu8, SimulatorMode);
  strcat(SendData, Mode);
  strcat(SendData, ")");
  
  Length = (I32)(strlen(SendData));
  SendLength[3] = (U8)Length;
    
  UtilSendTCPData("SimulatorControl", (const C8*)SendLength, 4, Sockfd, Debug);
  UtilSendTCPData("SimulatorControl", (const C8*)SendData, Length, Sockfd, Debug);
}



void SimulatorControlSendHeartbeat( I32 *Sockfd, struct sockaddr_in *Addr, TimeType *GPSTime, U8 Debug)
{

  C8 Data[10];

  bzero(Data,10);
  U32 GPSSOWms = GPSTime->GPSSecondsOfWeekU32*1000 + (U32)TimeControlGetMillisecond(GPSTime);
  Data[3] = 6;
  Data[5] = 1;
  Data[6] = (U8)(GPSSOWms >> 24);
  Data[7] = (U8)(GPSSOWms >> 16);
  Data[8] = (U8)(GPSSOWms >> 8);
  Data[9] = (U8)GPSSOWms;
  //printf("%d\n", GPSSOWms);
 
  UtilSendUDPData("SimulatorControl", Sockfd, Addr, Data, 10, Debug);

}


void SimulatorControlSendMONR( I32 *Sockfd, struct sockaddr_in *Addr, C8 *MonrData, U8 Debug)
{

  C8 Data[128];

  bzero(Data,128);
  Data[3] = strlen(MonrData);
  Data[5] = 2;
  strcat((Data+6), MonrData);
  
 
  UtilSendUDPData("SimulatorControl", Sockfd, Addr, Data, strlen(MonrData) + 6, Debug);

}
