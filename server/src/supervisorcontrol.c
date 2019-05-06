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
#include <netinet/tcp.h>
#include <poll.h>
#include <netdb.h>

#include "util.h"
#include "logger.h"
#include "logging.h"


/*------------------------------------------------------------
  -- Definition declarations.
  ------------------------------------------------------------*/

#define SUP_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define SUP_CONTROL_BUFFER_SIZE_20  20
#define SUP_CONTROL_BUFFER_SIZE_2048 2048
#define SUP_CONTROL_BUFFER_SIZE_3100 3100
#define SUP_CONTROL_BUFFER_SIZE_6200 6200
#define SUP_MQ_MAX_SIZE 6200
#define SUP_MESSAGE_BUFFER 1024
#define SUP_DEBUG_TCP_RX_DATA 0
#define SUP_TEXT_ROW_LENGTH 256

#define SUP_MODE_NORMAL 1
#define SUP_MODE_DEBUG 2
#define SUP_ISO_MESSAGE_RX_TIMEOUT 1000
#define SUP_TEXT_BUFFER_20 20

#define SUP_OBJ_PROP_FILE "../conf/objprop.conf"
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/


/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

#define MODULE_NAME "SupervisorControl"
static const LOG_LEVEL logLevel = LOG_LEVEL_DEBUG;

/*-------
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
  C8 TxBuffer[SUP_CONTROL_BUFFER_SIZE_2048];
  U8 SupervisorInitiatedU8 = 0;

  U32 RxTotalDataU32 = 0;
  U32 ReqRxLengthU32 = 0;
  U8 WaitAllDataU8 = 0;
  U8 DataChunkedU8 = 0;
 
  C8 ReceiveBuffer[SUP_CONTROL_BUFFER_SIZE_6200];
  C8 ASCIIBuffer[SUP_CONTROL_BUFFER_SIZE_6200];

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
  U8 ISOMessageStartedU8 = 0;
  U8 ISOMessageReadRestU8 = 0;
  U8 ISOMessageReceivedU8 = 0;

  OPROType OPROData; 
  C8 ObjectTrajPath[MAX_OBJECTS][MAX_FILE_PATH];
  C8 ObjectIPAddress[MAX_OBJECTS][MAX_FILE_PATH];
  I32 ObjectCountI32;
  FILE *fd;
  C8 TextRow[SUP_TEXT_ROW_LENGTH];
  C8 ValueBuffer[SUP_TEXT_BUFFER_20];
  C8 *C8Ptr;
  U32 IpU32 = 0;
  U8 ObjectTypeU8 = 0;
  U8 OperationModeU8 = 0;
  U32 MassU32 = 0;
  U32 DimensionXU32 = 0;
  U32 DimensionYU32 = 0;
  U32 DimensionZU32 = 0;

  I32 iExit = 0, iCommand;
  C8 MqBuffer[SUP_MQ_MAX_SIZE];
  (void)iCommInit(IPC_RECV_SEND,MQ_SU,0);
  U16 IterationCounter = 0;
  U32 TimestampU32 = 0;

  LogInit(MODULE_NAME,logLevel);
  LogMessage( LOG_LEVEL_INFO, "Supervisor control task running with PID: %i", getpid());
 
  bzero(TextBufferC8, SUP_CONTROL_BUFFER_SIZE_20);
  UtilSearchTextFile(TEST_CONF_FILE, "SupervisorIP=", "", TextBufferC8);
  bzero(SupervisorServerIpC8, SUP_CONTROL_BUFFER_SIZE_20);
  strcat(SupervisorServerIpC8, TextBufferC8);

  LogMessage(LOG_LEVEL_INFO,"Supervisor IP: %s", TextBufferC8);
  SupervisorIpU32 = UtilIPStringToInt(SupervisorServerIpC8);

  //UtilGetObjectsInfo(ObjectTrajPath, ObjectIPAddress, &ObjectCountI32);

  if(SupervisorIpU32 != 0)
  {
    bzero(TextBufferC8, SUP_CONTROL_BUFFER_SIZE_20);
    UtilSearchTextFile(TEST_CONF_FILE, "SupervisorTCPPort=", "", TextBufferC8);
    SupervisorTCPPortU16 = (U16)atoi(TextBufferC8);
        

    printf("[SupervisorControl] SupervisorTCPPort = %d\n", SupervisorTCPPortU16);


    while(!iExit)
    {

      if(SupervisorTCPSocketfdI32 <= 0)
      {
        ClientResultI32 =  UtilConnectTCPChannel("SupervisorControl", &SupervisorTCPSocketfdI32, (const C8*)SupervisorServerIpC8, SupervisorTCPPortU16);
        int yes = 1;
        int result = setsockopt(SupervisorTCPSocketfdI32, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));
        if(result < 0 )
        {
          LogMessage(LOG_LEVEL_WARNING,"Failed to set socket option code = %d", result);
        }
      }
      else
      {
       
        if(ISOMessageStartedU8 == 0 && ISOMessageReadRestU8 == 0) 
        {
          RxTotalDataU32 = 0;
          ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer, 2, 0); //Data length resides in ClientResultI32
          if(ClientResultI32 > 0) RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
        }
        else if(ClientResultI32 == 0)
        {
            LogMessage(LOG_LEVEL_INFO,"Client closed connection");
            close(SupervisorTCPSocketfdI32);
            SupervisorTCPSocketfdI32 = -1;
            SupervisorInitiatedU8 = 0;
        }

        //Initiate the simulator if not initialized and a there is a valid TCP connection
        if(SupervisorInitiatedU8 == 0 && SupervisorTCPSocketfdI32 > 0)
        {
          UtilISOBuildINSUPMessage(TxBuffer, &INSUPData, SUP_MODE_NORMAL, 1);
          UtilSendTCPData("SupervisorControl", TxBuffer, INSUPData.Header.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH + ISO_MESSAGE_FOOTER_LENGTH, &SupervisorTCPSocketfdI32, 0);
 
        }
        
        //Check if Sync word
        if(RxBuffer[0] == 0x7E && RxBuffer[1] == 0x7E && ISOMessageStartedU8 == 0 && ISOMessageReadRestU8 == 0) 
        {
          ISOMessageStartedU8 = 1;
          TimestampU32 = (U32)UtilgetCurrentUTCtimeMS();
          if(SUP_DEBUG_TCP_RX_DATA) LogMessage(LOG_LEVEL_DEBUG,"Rx SYNC word");
        } 

        //Get start of message
        if(ISOMessageStartedU8 == 1)
        {
          bzero(RxBuffer + 2, ISO_MESSAGE_HEADER_LENGTH - 2);
          ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer+2, ISO_MESSAGE_HEADER_LENGTH-2, 0); //Data length resides in ClientResultI32
          if(ClientResultI32 > 0)
          {
            
            RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
            if(SUP_DEBUG_TCP_RX_DATA) LogMessage(LOG_LEVEL_DEBUG,"Read Header. Total received bytes %d", RxTotalDataU32);
            if(RxTotalDataU32 == 11)
            {
              UtilISOBuildHeader(RxBuffer, &HeaderData, SUP_DEBUG_TCP_RX_DATA);
              ReqRxLengthU32 = HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH;
              bzero(RxBuffer+ISO_MESSAGE_HEADER_LENGTH, ReqRxLengthU32);
              ISOMessageStartedU8 = 0;
              ISOMessageReadRestU8 = 1;
              ClientResultI32 = 0;
              RxTotalDataU32 = 0;
              TimestampU32 = (U32)UtilgetCurrentUTCtimeMS();  
            }
          }
          else if (ClientResultI32 == 0)
          {
            ISOMessageStartedU8 = 0;
            SupervisorTCPSocketfdI32 = -1;
            SupervisorInitiatedU8 = 0;
          }
          else if((U32)UtilgetCurrentUTCtimeMS() - TimestampU32 > SUP_ISO_MESSAGE_RX_TIMEOUT)
          {
            ISOMessageStartedU8 = 0;
          }
         } 

        //Get the rest of the message
        if(ISOMessageReadRestU8 == 1)
        {
          ReqRxLengthU32 = ReqRxLengthU32 - ClientResultI32;
          ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer+ClientResultI32+ISO_MESSAGE_HEADER_LENGTH, ReqRxLengthU32, 0); //Data length resides in ClientResultI32
          if(ClientResultI32 > 0)
          {
            RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
            if(SUP_DEBUG_TCP_RX_DATA) LogMessage(LOG_LEVEL_DEBUG,"Read %d requested bytes, so far %d bytes read", (U32)(HeaderData.MessageLengthU32+ISO_MESSAGE_FOOTER_LENGTH), RxTotalDataU32);
            if(RxTotalDataU32 == HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH)
            {
              ISOMessageStartedU8 = 0;
              ISOMessageReadRestU8 = 0;
              ISOMessageReceivedU8 = 1;
              if(SUP_DEBUG_TCP_RX_DATA)
              { 
                LogMessage(LOG_LEVEL_DEBUG,"ISO Message received!");
                //for(i = 0; i < (ISO_MESSAGE_HEADER_LENGTH + HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH); i ++) printf("%x ", RxBuffer[i]);
                //printf("\n");
              }
            }
          }
          else if (ClientResultI32 == 0)
          {
            ISOMessageReadRestU8 = 0;
            SupervisorTCPSocketfdI32 = -1;
            SupervisorInitiatedU8 = 0;
          }
          else if((U32)UtilgetCurrentUTCtimeMS() - TimestampU32 > SUP_ISO_MESSAGE_RX_TIMEOUT)
          {
            ISOMessageReadRestU8 = 0;
          }
        } 

        if(ISOMessageReceivedU8 == 1)
        {
          if(SUP_DEBUG_TCP_RX_DATA) LogMessage(LOG_LEVEL_DEBUG,"MessageId %d handled", HeaderData.MessageIdU16);
          
          if(HeaderData.MessageIdU16 == ISO_TRAJ_CODE)
          {

            bzero(ASCIIBuffer, (HeaderData.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH + ISO_MESSAGE_FOOTER_LENGTH)*2 + 1);
            UtilBinaryToHexText((HeaderData.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH + ISO_MESSAGE_FOOTER_LENGTH), RxBuffer, ASCIIBuffer, 0);
            iCommSend(COMM_TRAJ_FROMSUP, ASCIIBuffer);
            
            printf("[SupervisorControl] %d. Sending chunk to ObjectControl, size is %d bytes.\n", ++IterationCounter, HeaderData.MessageLengthU32);

          } 
          else if(HeaderData.MessageIdU16 == ISO_HEAB_CODE)
          {
            //printf("[SupervisorControl] HEAB.\n");
          }
          else if(HeaderData.MessageIdU16 == ISO_INSUP_CODE)
          {
            printf("[SupervisorControl] INSUP Received");
            SupervisorInitiatedU8 = 1; //Check the INSUP response data before setting this variable...

            if(SupervisorInitiatedU8 == 1)
            {
              //Build and send OPRO 
              fd = fopen(SUP_OBJ_PROP_FILE, "r");
              if(fd)
              {
                ObjectCountI32 = UtilCountFileRows(fd) - 1;
                fclose(fd);
                fd = fopen(SUP_OBJ_PROP_FILE, "r");
                UtilReadLine(fd, TextRow); //Read first line
                for(i = 0; i < ObjectCountI32; i ++)
                {
                  bzero(TextRow, SUP_TEXT_ROW_LENGTH);
                  UtilReadLine(fd, TextRow);
                  C8Ptr = TextRow;
                  for(j = 0; j < 7; j ++)
                  {
                    bzero(ValueBuffer, SUP_TEXT_BUFFER_20);
                    strncpy(ValueBuffer, C8Ptr, (U64)strstr(C8Ptr, ";") - (U64)C8Ptr);
                    C8Ptr = C8Ptr + (U64)strlen(ValueBuffer) + 1;
                    switch(j)
                    {
                      case 0:
                        IpU32 = UtilIPStringToInt(ValueBuffer);
                      break;
                      case 1:
                        ObjectTypeU8 = atoi(ValueBuffer);
                      break;
                      case 2:
                        OperationModeU8 = atoi(ValueBuffer);
                      break;
                      case 3:
                        MassU32 = atoi(ValueBuffer);
                      break;
                      case 4:
                        DimensionXU32 = atoi(ValueBuffer);
                      break;
                      case 5:
                        DimensionYU32 = atoi(ValueBuffer);
                      break;
                      case 6:
                        DimensionZU32 = atoi(ValueBuffer);
                      break;

                      default:
                      break;
                    }
                  }
                }

                UtilISOBuildOPROMessage(TxBuffer, &OPROData, UtilIPStringToInt(ObjectIPAddress[i]), ObjectTypeU8, OperationModeU8, MassU32, DimensionXU32, DimensionYU32, DimensionZU32, 0);
                UtilSendTCPData("SupervisorControl", TxBuffer, OPROData.Header.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH + ISO_MESSAGE_FOOTER_LENGTH, &SupervisorTCPSocketfdI32, 0);

                fclose(fd);
              }

            }
          }
          
          bzero(RxBuffer, ISO_MESSAGE_HEADER_LENGTH + HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH);
          ISOMessageReceivedU8 = 0;
          RxTotalDataU32 = 0;
          ClientResultI32 = 0;
        }
      }  

      bzero(MqBuffer,SUP_MQ_MAX_SIZE);
      (void)iCommRecv(&iCommand,MqBuffer,SUP_MQ_MAX_SIZE,NULL);

      if(iCommand == COMM_TRAJ_TOSUP)
      {
        DTMLengthU32 = UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        printf("[SupervisorControl] Sending chunk to Supervisor, size is %d bytes\n", (I32)strlen(MqBuffer));
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
      else if(iCommand == COMM_MONI_BIN /*GSD->MONRSizeU8 > 0*/)
      {
        //Send MONR data
        if (strlen(MqBuffer) > 0)
        {
          UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
          UtilSendTCPData("SupervisorControl", DTMTrajBuffer, sizeof(MONRType)+2, &SupervisorTCPSocketfdI32, 0);
        }

      }
      else if(iCommand == COMM_OSTM)
      {
        //Send OSTM data
        UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        UtilSendTCPData("SupervisorControl", DTMTrajBuffer, sizeof(OSTMType)+2, &SupervisorTCPSocketfdI32, 0);
      }
      else if(iCommand == COMM_OSEM)
      {
        //Send OSEM data
        UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        UtilSendTCPData("SupervisorControl", DTMTrajBuffer, sizeof(OSEMType)-2, &SupervisorTCPSocketfdI32, 0);
      }
      else if(iCommand == COMM_OBJ_STRT)
      {
        //Send STRT data
        UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        UtilSendTCPData("SupervisorControl", DTMTrajBuffer, sizeof(STRTType)+2, &SupervisorTCPSocketfdI32, 0);
      }
      else if(iCommand == COMM_HEAB)
      {
        //Send HEAB data
        UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);
        UtilSendTCPData("SupervisorControl", DTMTrajBuffer, sizeof(HEABType)+2, &SupervisorTCPSocketfdI32, 0);
      }
      else if(iCommand == COMM_EXIT)
      {
        iExit = 1;
        LogMessage(LOG_LEVEL_INFO,"Supervisor control exiting");
        (void)iCommClose();
      }
    }
  }
}



