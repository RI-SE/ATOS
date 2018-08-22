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


#define SIM_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define SIM_CONTROL_BUFFER_SIZE_20 20
#define SIM_CONTROL_BUFFER_SIZE_52 52
#define SIM_CONTROL_TASK_PERIOD_MS 1
#define SIM_CONTROL_HEARTBEAT_TIME_MS 10

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void SimulatorControlSendHeartbeat( I32 *Sockfd, struct sockaddr_in *Addr, U8 ServerStatus, U8 Debug);
void SimulatorControlInitiateSimulator( I32 *Sockfd, U8 SimulatorMode, U8 Debug);

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
  C8 ReceiveBuffer[SIM_CONTROL_BUFFER_SIZE_52];
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
        
    //printf("SimulatorTCPPort = %d\n", SimulatorTCPPortU16);
    //printf("SimulatorUDPPort = %d\n", SimulatorUDPPortU16);

    while(!iExit)
    {


      if(SimulatorTCPSocketfdI32 <= 0)
      {
        ClientResultI32 =  UtilConnectTCPChannel("SimulatorControl", &SimulatorTCPSocketfdI32, (const C8*)SimulatorServerIPC8, SimulatorTCPPortU16);
        UtilCreateUDPChannel("SimulatorControl", &SimulatorUDPSocketfdI32, (const C8*)SimulatorServerIPC8, SimulatorUDPPortU16, &simulator_addr);
      }
      else
      {
        bzero(ReceiveBuffer, SIM_CONTROL_BUFFER_SIZE_52);
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
          
          SimulatorControlInitiateSimulator(&SimulatorTCPSocketfdI32, 1, 1);
          SimulatorInitiatedU8 = 1;
        }
        
        if(ClientResultI32 > 0)
        {
          //Do something with the data
          printf("[SimulatorControl] %s\n", ReceiveBuffer);
        }

        bzero(ReceiveBuffer, SIM_CONTROL_BUFFER_SIZE_52);
        UtilReceiveUDPData("SimulatorControl", &SimulatorUDPSocketfdI32, ReceiveBuffer, 100, &ReceivedNewData, 1);

        if(ReceivedNewData)
        {
          

        }


        if(CycleU16 == 0)
        {
          SimulatorControlSendHeartbeat( &SimulatorUDPSocketfdI32, &simulator_addr, ServerStatusU8++, 0);
        }

      }

      bzero(MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);
      (void)iCommRecv(&iCommand,MqRecvBuffer,MQ_MAX_MESSAGE_LENGTH);

      if(iCommand == COMM_EXIT)
      {
        iExit = 1;  
        printf("simulatorcontrol exiting.\n");
        (void)iCommClose();
      }



       ++CycleU16;
      if(CycleU16 >= SIM_CONTROL_HEARTBEAT_TIME_MS/SIM_CONTROL_TASK_PERIOD_MS) CycleU16 = 0;
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = SIM_CONTROL_TASK_PERIOD_MS*1000000;
      (void)nanosleep(&sleep_time,&ref_time);


    }
  }
}




void SimulatorControlInitiateSimulator( I32 *Sockfd, U8 SimulatorMode, U8 Debug)
{

  C8 Data[] = "InitSimulator(1)";


  UtilSendTCPData("SimulatorControl", (const C8*)Data, sizeof(Data), Sockfd, Debug);

}



void SimulatorControlSendHeartbeat( I32 *Sockfd, struct sockaddr_in *Addr, U8 ServerStatus, U8 Debug)
{

  C8 Data[5];

  bzero(Data,5);
  Data[3] = 1;
  Data[4] = ServerStatus;

  UtilSendUDPData("SimulatorControl", Sockfd, Addr, Data, 5, Debug);

}
