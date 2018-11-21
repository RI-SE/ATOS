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
#define SIM_CONTROL_BUFFER_SIZE_5 5
#define SIM_CONTROL_BUFFER_SIZE_20 20
#define SIM_CONTROL_BUFFER_SIZE_128 128
#define SIM_CONTROL_BUFFER_SIZE_400 400
#define SIM_CONTROL_TASK_PERIOD_MS 1
#define SIM_CONTROL_HEARTBEAT_TIME_MS 10
#define SIM_CONTROL_LOG_BUFFER_LENGTH 128
#define SIM_CONTROL_VOP_OBJECT_SIZE 23

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/

void SimulatorControlSendHeartbeat( I32 *Sockfd, struct sockaddr_in *Addr, TimeType *GPSTime, U8 Debug);
void SimulatorControlInitiateSimulator( I32 *Sockfd, U8 SimulatorMode, U8 Debug);
void SimulatorControlStartScenario( I32 *Sockfd, C8 *StartTime, U8 Debug);
void SimulatorControlSendMONR( I32 *Sockfd, struct sockaddr_in *Addr, C8 *MonrData, U8 Debug);
U32 SimulatorControlBuildObjectMonitorMessage(C8* MessageBuffer, C8 *MONRData, ObjectMonitorType *ObjectMonitorData, U8 debug);
U32 SimulatorControlVOILToASCII(C8 *VOILData, C8 *VOILString);
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
  C8 ReceiveBuffer[SIM_CONTROL_BUFFER_SIZE_400];
  C8 SendBuffer[SIM_CONTROL_BUFFER_SIZE_128];
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
  C8 VOILString[SIM_CONTROL_LOG_BUFFER_LENGTH];
  C8 Timestamp[SIM_CONTROL_BUFFER_SIZE_20];
  ObjectMonitorType ObjectMonitorData;
  U32 LengthU32;

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
    //ReceivedNewData = 1;

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
        ClientResultI32 = UtilReceiveTCPData("SimulatorControl", &SimulatorTCPSocketfdI32, ReceiveBuffer, 0);

        if(ClientResultI32 == 0)
        {
            DEBUG_LPRINT(DEBUG_LEVEL_HIGH, "[SimulatorControl] Client closed connection.\n");
            close(SimulatorTCPSocketfdI32);
            SimulatorTCPSocketfdI32 = -1;
            SimulatorInitiatedU8 = 0;
        }

        if(SimulatorInitiatedU8 == 0 && SimulatorTCPSocketfdI32 > 0)
        {

          /*Initiate the simulator if not initialized and a there is a valid TCP connection */
          SimulatorControlInitiateSimulator(&SimulatorTCPSocketfdI32, 1, 0);
          SimulatorInitiatedU8 = 1;
        }

        if(ClientResultI32 > 0)
        {
          //Ok, received data on TCP, do something with the data
          printf("[SimulatorControl] %s\n", ReceiveBuffer);
        }


        /*Check if we received data from the simulator*/
        bzero(ReceiveBuffer, SIM_CONTROL_BUFFER_SIZE_400);
        UtilReceiveUDPData("SimulatorControl", &SimulatorUDPSocketfdI32, ReceiveBuffer, 100, &ReceivedNewData, 0);


        if(ReceivedNewData)
        {
          /*
          ReceiveBuffer[0] = 0; ReceiveBuffer[1] = 0; ReceiveBuffer[2] = 0; ReceiveBuffer[3] = 0;
          ReceiveBuffer[4] = 0xA1; ReceiveBuffer[5] = 0x00;
          ReceiveBuffer[6] = 12; ReceiveBuffer[7] = 34; ReceiveBuffer[8] = 56; ReceiveBuffer[9] = 78;
          ReceiveBuffer[10] = 3; ReceiveBuffer[11] = 2;
          ReceiveBuffer[12] = 5; ReceiveBuffer[13] = 6;
          ReceiveBuffer[14] = 0; ReceiveBuffer[15] = 0; ReceiveBuffer[16] = 56; ReceiveBuffer[17] = 45;
          ReceiveBuffer[18] = 19; ReceiveBuffer[19] = 0; ReceiveBuffer[20] = 23; ReceiveBuffer[21] = 69;
          ReceiveBuffer[22] = 254; ReceiveBuffer[23] = 0; ReceiveBuffer[24] = 2; ReceiveBuffer[25] = 80;
          ReceiveBuffer[26] = 24; ReceiveBuffer[27] = 6;
          ReceiveBuffer[28] = 73; ReceiveBuffer[29] = 8;
          ReceiveBuffer[30] = 28; ReceiveBuffer[31] = 81;
          ReceiveBuffer[32] = 21; ReceiveBuffer[33] = 33;

          ReceiveBuffer[34] = 10; ReceiveBuffer[35] = 11;
          ReceiveBuffer[36] = 0; ReceiveBuffer[37] = 0; ReceiveBuffer[38] = 56; ReceiveBuffer[39] = 45;
          ReceiveBuffer[40] = 19; ReceiveBuffer[41] = 0; ReceiveBuffer[42] = 23; ReceiveBuffer[43] = 69;
          ReceiveBuffer[44] = 254; ReceiveBuffer[45] = 0; ReceiveBuffer[46] = 2; ReceiveBuffer[47] = 80;
          ReceiveBuffer[48] = 24; ReceiveBuffer[49] = 6;
          ReceiveBuffer[50] = 73; ReceiveBuffer[51] = 8;
          ReceiveBuffer[52] = 28; ReceiveBuffer[53] = 81;
          ReceiveBuffer[54] = 21; ReceiveBuffer[55] = 33;
          */

          /*We received data...*/
          /*Add binary data to global data*/

          //printf("FROM HIL: ");
          for(i = 0; i < ReceiveBuffer[3] + 4; i++)
          {
            GSD->VOILData[i] = (U8)ReceiveBuffer[i];
            //printf("%x-", GSD->VOILData[i]);
          }
           // printf("\n");

          /*Make ASCII data from binary data*/
          bzero(VOILString, SIM_CONTROL_LOG_BUFFER_LENGTH);
          SimulatorControlVOILToASCII(ReceiveBuffer, VOILString);
          /*Send data to message queue so it is written to the log file*/
          (void)iCommSend(COMM_VIOP, VOILString);
        }


        if(CycleU16 == 0)
        {
          /*Send hearbeat to the simulator*/
          SimulatorControlSendHeartbeat( &SimulatorUDPSocketfdI32, &simulator_addr, GPSTime, 0);
        }

      }

      if(GSD->ScenarioStartTimeU32 != 0)
      {
        /*There is a scenario start time in global data, send it to the simulator*/
        bzero(Timestamp, SIM_CONTROL_BUFFER_SIZE_20);
        sprintf(Timestamp, "%" PRIu32, GSD->ScenarioStartTimeU32);
        printf("[SimulatorControl] Sending StartScenario(%s)\n", Timestamp);
        //LOG_SEND(LogBuffer, "[SimulatorControl] Sending StartScenario(%s)\n", Timestamp);
        SimulatorControlStartScenario( &SimulatorTCPSocketfdI32, Timestamp, 0);
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
        printf("Monr sim %s\n", MqRecvBuffer);
        LengthU32 = SimulatorControlBuildObjectMonitorMessage(SendBuffer, MqRecvBuffer, &ObjectMonitorData, 0);
        UtilSendUDPData("SimulatorControl", &SimulatorUDPSocketfdI32, &simulator_addr, SendBuffer, LengthU32, 0);
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


U32 SimulatorControlBuildObjectMonitorMessage(C8* MessageBuffer, C8 *MONRData, ObjectMonitorType *ObjectMonitorData, U8 debug)
{

  C8 *ptr;
  C8 TextBuffer[SIM_CONTROL_BUFFER_SIZE_20];
  U32 i,j;

  /*10.130.22.8;0;0;-6;14;229;256;800;0;0;0;0;4;0; */
  //printf("%s\n", MONRData);

  ptr = MONRData;
  /*Get IP*/
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr, (uint64_t)strchr(ptr, ';') - (uint64_t)ptr);
  ObjectMonitorData->ObjectIPU32 = SwapU32(UtilIPStringToInt(TextBuffer));

  /*Get GPSSOW*/
  ptr = strchr(ptr+1, ';');
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->GPSSOWU32 = SwapU32((U32) atoi(TextBuffer));

  //Get XPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->XPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get YPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->YPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get ZPosition
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->ZPositionI32 = SwapI32((I32) atoi(TextBuffer));

  //Get Heading
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->HeadingU16 = SwapU16((U16) atoi(TextBuffer));

  //Get Speed
  ptr = strchr(ptr+1, ';');
  bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_20);
  strncpy(TextBuffer, ptr+1, (uint64_t)strchr(ptr+1, ';') - (uint64_t)ptr);
  ObjectMonitorData->SpeedI16 = SwapI16((I16) atoi(TextBuffer));

  //Set MessageId
  ObjectMonitorData->MessageIdU16 = SwapU16(2);

  ptr=(char *)ObjectMonitorData;
  for(i=0; i<sizeof(ObjectMonitorType); i++) *(MessageBuffer + i + 4) = *ptr++;

  *(MessageBuffer + 0) = (U8)(i >> 24);
  *(MessageBuffer + 1) = (U8)(i >> 16);
  *(MessageBuffer + 2) = (U8)(i >> 8);
  *(MessageBuffer + 3) = (U8)(i);

  if(debug)
  {
    printf("----MONR TO SIMULATOR----\n");
    for(j = 0;j < sizeof(ObjectMonitorType) + 4; j ++) printf("%x ", (unsigned char)MessageBuffer[j]);
    printf("\n");
  }


  return i + 4;
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



U32 SimulatorControlVOILToASCII(C8 *VOILData, C8 *VOILString)
{
    C8 Buffer[SIM_CONTROL_BUFFER_SIZE_5];
    C8 TextBuffer[SIM_CONTROL_BUFFER_SIZE_400];
    U32 i;
    U16 WordU16;
    I16 WordI16;
    U32 WordU32;
    I32 WordI32;
    U8 VoilSize = SIM_CONTROL_VOP_OBJECT_SIZE - 1;

    WordU16 = *(VOILData + 4);
    WordU16 = (WordU16 << 8) | *(VOILData+5);


    if(WordU16 == 0xA100)
    {

        bzero(TextBuffer, SIM_CONTROL_BUFFER_SIZE_400);
        //GPSSOW
        bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
        WordU32 = 0;
        WordU32 = *(VOILData + 6);
        WordU32 = (WordU32 << 8) | *(VOILData+7);
        WordU32 = (WordU32 << 8) | *(VOILData+8);
        WordU32 = (WordU32 << 8) | *(VOILData+9);
        sprintf(Buffer, "%" PRIu32, WordU32);
        strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

        //Dynamic World state
        bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
        sprintf(Buffer, "%" PRIu8, *(VOILData+10));
        strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

        //Object Count
        bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
        sprintf(Buffer, "%" PRIu8, *(VOILData+11));
        strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

        C8 *StartPtr = (VOILData+12);
        for(i=0; i < *(VOILData+11); i++)
        {
          //Object Id
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          sprintf(Buffer, "%" PRIu8, *(StartPtr+VoilSize*i+0));
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //Object State
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          sprintf(Buffer, "%" PRIu8, *(StartPtr+VoilSize*i+1));
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //XPosition
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordI32 = 0;
          WordI32 = *(StartPtr+VoilSize*i+2);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+3);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+4);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+5);
          sprintf(Buffer, "%" PRIi32, WordI32);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //YPosition
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordI32 = 0;
          WordI32 = *(StartPtr+VoilSize*i+6);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+7);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+8);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+9);
          sprintf(Buffer, "%" PRIi32, WordI32);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //ZPosition
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordI32 = 0;
          WordI32 = *(StartPtr+VoilSize*i+10);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+11);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+12);
          WordI32 = (WordI32 << 8) | *(StartPtr+VoilSize*i+13);
          sprintf(Buffer, "%" PRIi32, WordI32);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //Heading
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordU16 = 0;
          WordU16 = *(StartPtr+VoilSize*i+14);
          WordU16 = (WordU16 << 8) | *(StartPtr+VoilSize*i+15);
          sprintf(Buffer, "%" PRIu16, WordU16);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //Roll
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordU16 = 0;
          WordU16 = *(StartPtr+VoilSize*i+16);
          WordU16 = (WordU16 << 8) | *(StartPtr+VoilSize*i+17);
          sprintf(Buffer, "%" PRIu16, WordU16);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //Pitch
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordU16 = 0;
          WordU16 = *(StartPtr+VoilSize*i+18);
          WordU16 = (WordU16 << 8) | *(StartPtr+VoilSize*i+19);
          sprintf(Buffer, "%" PRIu16, WordU16);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");

          //Speed
          bzero(Buffer, SIM_CONTROL_BUFFER_SIZE_5);
          WordI16 = 0;
          WordI16 = *(StartPtr+VoilSize*i+20);
          WordI16 = (WordI16 << 8) | *(StartPtr+VoilSize*i+21);
          sprintf(Buffer, "%" PRIi16, WordI16);
          strcat(TextBuffer, Buffer); strcat(TextBuffer,";");
        }
    }

    strncpy(VOILString, (const C8*)TextBuffer, strlen(TextBuffer) );




    return 0;
}
