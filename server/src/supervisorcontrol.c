/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2019 CHRONOS II project
  ------------------------------------------------------------------------------
  -- File        : supervisorcontrol.c
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


/*------------------------------------------------------------
  -- Definition declarations.
  ------------------------------------------------------------*/

#define SUP_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define SUP_CONTROL_BUFFER_SIZE_20  20
#define SUP_CONTROL_BUFFER_SIZE_2048 2048
#define SUP_CONTROL_BUFFER_SIZE_3100 3100
#define SUP_MQ_MAX_SIZE 6200
#define SUP_MESSAGE_BUFFER 1024
#define SUP_DEBUG_TCP_RX_DATA 0

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/



/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int supervisorcontrol_task(TimeType *GPSTime, GSDType *GSD)
{

  C8 TextBufferC8[SUP_CONTROL_BUFFER_SIZE_20];
  C8 SupervisorServerIpC8[SUP_CONTROL_BUFFER_SIZE_20];
  U32 SupervisorIpU32 = 0;
  U16 SupervisorTCPPortU16;
  I32 SupervisorTCPSocketfdI32=-1;
  
  struct sockaddr_in supervisor_addr;
  
  I32 ClientResultI32;
  C8 RxBuffer[SUP_CONTROL_BUFFER_SIZE_2048];
  U8 SupervisorInitiatedU8 = 0;

  U32 RxTotalDataU32 = 0;
  U32 ReqRxLengthU32 = 0;
  U8 WaitAllDataU8 = 0;
  U8 DataChunkedU8 = 0;
 
  C8 ReceiveBuffer[SUP_CONTROL_BUFFER_SIZE_3100];

  I32 i,j;
 
  HeaderType HeaderData;
  INSUPType INSUPData;
  HEABType HEABData;
  DOTMType DOTMData;
  C8 MessageBuffer[SUP_MESSAGE_BUFFER];

  U32 MessageLength;
  U32 DTMIpU32;
  U32 DTMLengthU32;
  TRAJInfoType TRAJInfoData;
  U16 MiscU16;
  C8 DTMTrajBuffer[ISO_DTM_ROWS_IN_TRANSMISSION*ISO_DTM_ROW_MESSAGE_LENGTH + ISO_MESSAGE_HEADER_LENGTH + ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH];



 /* C8 UDPReceiveBuffer[SIM_CONTROL_BUFFER_SIZE_400];
  C8 SendBuffer[SIM_CONTROL_BUFFER_SIZE_128];
  I32 ReceivedNewData, i, j;
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
  C8 SimFuncReqResponse[SIM_CONTROL_BUFFER_SIZE_64];
  C8 SimFuncRx[SIM_CONTROL_BUFFER_SIZE_64];
  U16 SimRxCodeU16 = 0;
  U16 ResponseDataIndexU16 = 0;
  C8 MsgQueBuffer[SIM_CONTROL_BUFFER_SIZE_6200];

  SMGDType SMGD;

  SMGD.SimulatorModeU8 = 0;
  
  OBCState_t OBCStateStatus = OBC_STATE_IDLE;
  U8 ObjectAddressListSentU8 = 0;*/

  I32 iExit = 0, iCommand;
  C8 MqBuffer[SUP_MQ_MAX_SIZE];
  (void)iCommInit(IPC_RECV_SEND,MQ_LG,0);


  printf("Starting supervisor control...\n");
 


  /*gettimeofday(&ExecTime, NULL);
  CurrentMilliSecondU16 = (U16) (ExecTime.tv_usec / 1000);
  PrevMilliSecondU16 = CurrentMilliSecondU16;*/

  bzero(TextBufferC8, SUP_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "SupervisorIP=", "", TextBufferC8);
  bzero(SupervisorServerIpC8, SUP_CONTROL_BUFFER_SIZE_20);
  strcat(SupervisorServerIpC8, TextBufferC8);
  

  printf("[SupervisorControl] Supervisor IP: %s\n", TextBufferC8);
  SupervisorIpU32 = UtilIPStringToInt(SupervisorServerIpC8);


  if(SupervisorIpU32 != 0)
  {
    bzero(TextBufferC8, SUP_CONTROL_BUFFER_SIZE_20);
    UtilSearchTextFile(TEST_CONF_FILE, "SupervisorTCPPort=", "", TextBufferC8);
    SupervisorTCPPortU16 = (U16)atoi(TextBufferC8);
        
    printf("SupervisorTCPPort = %d\n", SupervisorTCPPortU16);

    while(!iExit)
    {

      if(SupervisorTCPSocketfdI32 <= 0)
      {
        ClientResultI32 =  UtilConnectTCPChannel("SupervisorControl", &SupervisorTCPSocketfdI32, (const C8*)SupervisorServerIpC8, SupervisorTCPPortU16);
      }
      else
      {
        bzero(RxBuffer, SUP_CONTROL_BUFFER_SIZE_2048);
        
        ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer, 0); //Data length resides in ClientResultI32

        if(ClientResultI32 == 0)
        {
            DEBUG_LPRINT(DEBUG_LEVEL_HIGH, "[SupervisorControl] Client closed connection.\n");
            close(SupervisorTCPSocketfdI32);
            SupervisorTCPSocketfdI32 = -1;
            SupervisorInitiatedU8 = 0;
        }

        /*Initiate the simulator if not initialized and a there is a valid TCP connection */
        if(SupervisorInitiatedU8 == 0 && SupervisorTCPSocketfdI32 > 0)
        {
          UtilISOBuildINSUPMessage(RxBuffer, &INSUPData, 0, 0);
          UtilISOBuildHEABMessage(RxBuffer, &HEABData, GPSTime, 0, 0);
          SupervisorInitiatedU8 = 1;
        }
        

        //Send ObjectAddressList if in correct mode and ObjectControl is in armed state
        //if(OBCStateStatus == OBC_STATE_ARMED && (SMGD.SimulatorModeU8 == SIM_CONTROL_DTM_MODE || SMGD.SimulatorModeU8 == SIM_CONTROL_VIM_DTM_MODE) && ObjectAddressListSentU8 == 0)
        //{
        //  SimulatorControlObjectAddressList(&SimulatorTCPSocketfdI32, "192.168.0.4", SimFuncReqResponse, 0);
        //  ObjectAddressListSentU8 = 1;
        //}


        if(ClientResultI32 > 0 || WaitAllDataU8 == 1)
        {

          RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
          if(WaitAllDataU8 == 0)
          {
            UtilISOBuildHeader(RxBuffer, &HeaderData, 0);
            ReqRxLengthU32 = HeaderData.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH + ISO_MESSAGE_FOOTER_LENGTH;
            bzero(ReceiveBuffer, SUP_CONTROL_BUFFER_SIZE_3100);
            j = 0;
          }
          
          if(ClientResultI32 > 0 && SUP_DEBUG_TCP_RX_DATA)
          {
            printf("[SupervisorControl] TCP Rx length = %d data: ", ClientResultI32);
            for(int i = 0;i < ClientResultI32; i ++) printf("%x ", (C8)RxBuffer[i]);
            printf("\n");
            printf("[SupervisorControl] ReqRxLengthU32= %d\n", ReqRxLengthU32);
          }

          if (RxTotalDataU32 != ReqRxLengthU32)
          {
            WaitAllDataU8 = 1;
            DataChunkedU8 = 1;
            for(i = 0; i < ClientResultI32; i++, j++) ReceiveBuffer[j] = RxBuffer[i]; 
          }
          else
          {
            if(DataChunkedU8 == 1) for(i = 0; i < ClientResultI32; i++, j++) ReceiveBuffer[j] = RxBuffer[i]; 
            else if(DataChunkedU8 == 0) for(j = 0; j < RxTotalDataU32; j++) ReceiveBuffer[j] = RxBuffer[j];
            DataChunkedU8 = 0;
            ReqRxLengthU32 = 0;
            WaitAllDataU8 = 0; 
          }

          if(ClientResultI32 > 0 && SUP_DEBUG_TCP_RX_DATA)
          {
            printf("[SupervisorControl] WaitAllDataU8= %d\n", WaitAllDataU8);
            printf("[SupervisorControl] RxTotalDataU32= %d\n", RxTotalDataU32);
            printf("[SupervisorControl] ClientResultI32= %d\n", ClientResultI32);
            printf("[SupervisorControl] DataChunkedU8= %d\n", DataChunkedU8);
          }
        }
        
        if(WaitAllDataU8 == 0 && ClientResultI32 > 0)
        {

          if(HeaderData.MessageIdU16 == ISO_TRAJ_CODE)
          {

            bzero(MqBuffer,SUP_MQ_MAX_SIZE);

            UtilBinaryToHexText(RxTotalDataU32, ReceiveBuffer, MqBuffer, 0);
            (void)iCommSend(COMM_TRAJ, MqBuffer);

          }

          ClientResultI32 = 0;
        }
      }  

      bzero(MqBuffer,SUP_MQ_MAX_SIZE);
      (void)iCommRecv(&iCommand,MqBuffer,SUP_MQ_MAX_SIZE);
      if(iCommand == COMM_TRAJ_SUP)
      {

        DTMLengthU32 = UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        DTMIpU32 = (C8)DTMTrajBuffer[0];
        DTMIpU32 = (C8)DTMTrajBuffer[1] | (DTMIpU32 << 8);
        DTMIpU32 = (C8)DTMTrajBuffer[2] | (DTMIpU32 << 8);
        DTMIpU32 = (C8)DTMTrajBuffer[3] | (DTMIpU32 << 8);
        TRAJInfoData.IpAddressU32 = DTMIpU32;

        MiscU16 = (C8)DTMTrajBuffer[4];
        MiscU16 = (C8)DTMTrajBuffer[5] | (MiscU16 << 8);
        TRAJInfoData.TrajectoryIDU16 = MiscU16;
        i = 0;
        do
        {
            TRAJInfoData.TrajectoryNameC8[i] = (C8)DTMTrajBuffer[i + 6];
            i ++;
        } while(TRAJInfoData.TrajectoryNameC8[i-1] != 0);

        MiscU16 = (C8)DTMTrajBuffer[70];
        MiscU16 = (C8)DTMTrajBuffer[71] | (MiscU16 << 8);

        TRAJInfoData.TrajectoryVersionU16 = MiscU16;

        MiscU16 = 72; //Number of bytes in [Ip, Id, Name, Version]  

        /*TRAJ Header*/
        MessageLength = UtilISOBuildTRAJMessageHeader(MessageBuffer, (DTMLengthU32-MiscU16)/SIM_TRAJ_BYTES_IN_ROW , &HeaderData, &TRAJInfoData, 0);
        /*Send TRAJ header*/
        UtilSendTCPData("SupervisorControl", MessageBuffer, MessageLength, &SupervisorTCPSocketfdI32, 0);
        /*TRAJ Data*/
        MessageLength = UtilISOBuildTRAJMessage(MessageBuffer, DTMTrajBuffer+MiscU16, (DTMLengthU32-MiscU16)/SIM_TRAJ_BYTES_IN_ROW, &DOTMData, 0);
        /*Send DTM data*/
        UtilSendTCPData("SupervisorControl", MessageBuffer, MessageLength, &SupervisorTCPSocketfdI32, 0);

      }
      else if(iCommand == COMM_EXIT)
      {
        iExit = 1;
        printf("supervisor control exiting.\n");
        (void)iCommClose();
      }
    }
  }
}



