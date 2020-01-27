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
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <netdb.h>

#include "logger.h"
#include "datadictionary.h"


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

#define SUP_MODE_NORMAL 1
#define SUP_MODE_DEBUG 2
#define SUP_ISO_MESSAGE_RX_TIMEOUT 1000
/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
static void signalHandler(int signo);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

#define MODULE_NAME "SupervisorControl"
static volatile int iExit = 0;

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
void supervisorcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {

	C8 TextBufferC8[SUP_CONTROL_BUFFER_SIZE_20];
	C8 SupervisorServerIpC8[SUP_CONTROL_BUFFER_SIZE_20];
	U32 SupervisorIpU32 = 0;
	U16 SupervisorTCPPortU16;
	I32 SupervisorTCPSocketfdI32 = -1;

	struct sockaddr_in supervisor_addr;

	I32 ClientResultI32;
	C8 RxBuffer[SUP_CONTROL_BUFFER_SIZE_2048];
	C8 TxBuffer[SUP_CONTROL_BUFFER_SIZE_2048];
	U8 SupervisorInitiatedU8 = 0;

	U32 RxTotalDataU32 = 0;
	U32 ReqRxLengthU32 = 0;
	U8 WaitAllDataU8 = 0;
	U8 DataChunkedU8 = 0;

	C8 ReceiveBuffer[SUP_CONTROL_BUFFER_SIZE_3100];

	I32 i, j;

	HeaderType HeaderData;
	DOTMType DOTMData;
	C8 MessageBuffer[SUP_MESSAGE_BUFFER];

	U32 MessageLength;
	U32 DTMIpU32;
	U32 DTMLengthU32;
	TRAJInfoType TRAJInfoData;
	U16 MiscU16;
	C8 DTMTrajBuffer[ISO_DTM_ROWS_IN_TRANSMISSION * ISO_DTM_ROW_MESSAGE_LENGTH + ISO_MESSAGE_HEADER_LENGTH +
					 ISO_TRAJ_INFO_ROW_MESSAGE_LENGTH];
	U8 ISOMessageStartedU8 = 0;
	U8 ISOMessageReadRestU8 = 0;
	U8 ISOMessageReceivedU8 = 0;

	enum COMMAND iCommand;
	C8 MqBuffer[SUP_MQ_MAX_SIZE];

	GSD->SupChunkSize = 0;
	U16 IterationCounter = 0;
	U32 TimestampU32 = 0;

	// Create log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Supervisor control task running with PID: %i", getpid());

	DataDictionaryGetExternalSupervisorIPU32(GSD, &SupervisorIpU32);

	DataDictionaryGetExternalSupervisorIPC8(GSD, SupervisorServerIpC8, SUP_CONTROL_BUFFER_SIZE_20);

	LogMessage(LOG_LEVEL_INFO, "Supervisor IP: %s", SupervisorServerIpC8);

	LogMessage(LOG_LEVEL_INFO, "Supervisor control task running with PID: %i", getpid());

	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Set up message bus connection
	if (iCommInit())
		util_error("Unable to connect to message bus");


	if (SupervisorIpU32 != 0) {
		DataDictionaryGetSupervisorTCPPortU16(GSD, &SupervisorTCPPortU16);

		LogMessage(LOG_LEVEL_INFO, "SupervisorTCPPort = %d", SupervisorTCPPortU16);

		while (!iExit) {

			if (SupervisorTCPSocketfdI32 <= 0) {
				ClientResultI32 =
					UtilConnectTCPChannel("SupervisorControl", &SupervisorTCPSocketfdI32,
										  (const C8 *)SupervisorServerIpC8, SupervisorTCPPortU16);
				int yes = 1;
				int result = setsockopt(SupervisorTCPSocketfdI32, IPPROTO_TCP, TCP_NODELAY, (char *)&yes,
										sizeof (int));

				if (result < 0) {
					LogMessage(LOG_LEVEL_WARNING, "Failed to set socket option code = %d", result);
				}
			}
			else {

				if (ISOMessageStartedU8 == 0 && ISOMessageReadRestU8 == 0) {
					RxTotalDataU32 = 0;
					ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer, 2, 0);	//Data length resides in ClientResultI32
					if (ClientResultI32 > 0)
						RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
				}
				else if (ClientResultI32 == 0) {
					LogMessage(LOG_LEVEL_INFO, "Client closed connection");
					close(SupervisorTCPSocketfdI32);
					SupervisorTCPSocketfdI32 = -1;
					SupervisorInitiatedU8 = 0;
				}

				//Initiate the simulator if not initialized and a there is a valid TCP connection
				if (SupervisorInitiatedU8 == 0 && SupervisorTCPSocketfdI32 > 0) {
					MessageLength =
						encodeINSUPMessage(SUPERVISOR_COMMAND_NORMAL, TxBuffer, sizeof (TxBuffer), 0);
					UtilSendTCPData("SupervisorControl", TxBuffer, MessageLength, &SupervisorTCPSocketfdI32,
									0);
					SupervisorInitiatedU8 = 1;
				}

				//Check if Sync word
				if (RxBuffer[0] == 0x7E && RxBuffer[1] == 0x7E && ISOMessageStartedU8 == 0
					&& ISOMessageReadRestU8 == 0) {
					ISOMessageStartedU8 = 1;
					TimestampU32 = (U32) UtilgetCurrentUTCtimeMS();
					if (SUP_DEBUG_TCP_RX_DATA)
						LogMessage(LOG_LEVEL_DEBUG, "Rx SYNC word");
				}

				//Get start of message
				if (ISOMessageStartedU8 == 1) {
					bzero(RxBuffer + 2, ISO_MESSAGE_HEADER_LENGTH - 2);
					ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer + 2, ISO_MESSAGE_HEADER_LENGTH - 2, 0);	//Data length resides in ClientResultI32
					if (ClientResultI32 > 0) {

						RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
						if (SUP_DEBUG_TCP_RX_DATA)
							LogMessage(LOG_LEVEL_DEBUG, "Read Header. Total received bytes %d",
									   RxTotalDataU32);
						if (RxTotalDataU32 == 11) {
							UtilISOBuildHeader(RxBuffer, &HeaderData, SUP_DEBUG_TCP_RX_DATA);
							ReqRxLengthU32 = HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH;
							bzero(RxBuffer + ISO_MESSAGE_HEADER_LENGTH, ReqRxLengthU32);
							ISOMessageStartedU8 = 0;
							ISOMessageReadRestU8 = 1;
							ClientResultI32 = 0;
							RxTotalDataU32 = 0;
							TimestampU32 = (U32) UtilgetCurrentUTCtimeMS();
						}
					}
					else if (ClientResultI32 == 0) {
						ISOMessageStartedU8 = 0;
						SupervisorTCPSocketfdI32 = -1;
						SupervisorInitiatedU8 = 0;
					}
					else if ((U32) UtilgetCurrentUTCtimeMS() - TimestampU32 > SUP_ISO_MESSAGE_RX_TIMEOUT) {
						ISOMessageStartedU8 = 0;
					}
				}

				//Get the rest of the message
				if (ISOMessageReadRestU8 == 1) {
					ReqRxLengthU32 = ReqRxLengthU32 - ClientResultI32;
					ClientResultI32 = UtilReceiveTCPData("SupervisorControl", &SupervisorTCPSocketfdI32, RxBuffer + ClientResultI32 + ISO_MESSAGE_HEADER_LENGTH, ReqRxLengthU32, 0);	//Data length resides in ClientResultI32
					if (ClientResultI32 > 0) {
						RxTotalDataU32 = RxTotalDataU32 + ClientResultI32;
						if (SUP_DEBUG_TCP_RX_DATA)
							LogMessage(LOG_LEVEL_DEBUG, "Read %d requested bytes, so far %d bytes read",
									   (U32) (HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH),
									   RxTotalDataU32);
						if (RxTotalDataU32 == HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH) {
							ISOMessageStartedU8 = 0;
							ISOMessageReadRestU8 = 0;
							ISOMessageReceivedU8 = 1;
							if (SUP_DEBUG_TCP_RX_DATA) {
								LogMessage(LOG_LEVEL_DEBUG, "ISO Message received!");
								//for(i = 0; i < (ISO_MESSAGE_HEADER_LENGTH + HeaderData.MessageLengthU32 + ISO_MESSAGE_FOOTER_LENGTH); i ++) printf("%x ", RxBuffer[i]);
								//printf("\n");
							}
						}
					}
					else if (ClientResultI32 == 0) {
						ISOMessageReadRestU8 = 0;
						SupervisorTCPSocketfdI32 = -1;
						SupervisorInitiatedU8 = 0;
					}
					else if ((U32) UtilgetCurrentUTCtimeMS() - TimestampU32 > SUP_ISO_MESSAGE_RX_TIMEOUT) {
						ISOMessageReadRestU8 = 0;
					}
				}

				if (ISOMessageReceivedU8 == 1) {
					if (SUP_DEBUG_TCP_RX_DATA)
						LogMessage(LOG_LEVEL_DEBUG, "MessageId %d handled", HeaderData.MessageIdU16);

					if (HeaderData.MessageIdU16 == ISO_TRAJ_CODE) {
						//bzero(MqBuffer,SUP_MQ_MAX_SIZE);
						//UtilBinaryToHexText(RxTotalDataU32, ReceiveBuffer, MqBuffer, 0);
						//printf("%d. Send COMM_TRAJ_FROMSUP\n", ++IterationCounter);
						//(void)iCommSend(COMM_TRAJ_FROMSUP, MqBuffer);
						for (i = 0;
							 i <
							 HeaderData.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH +
							 ISO_MESSAGE_FOOTER_LENGTH; i++) {
							GSD->SupChunk[i] = RxBuffer[i];
						}
						GSD->SupChunkSize =
							HeaderData.MessageLengthU32 + ISO_MESSAGE_HEADER_LENGTH +
							ISO_MESSAGE_FOOTER_LENGTH;
						LogMessage(LOG_LEVEL_INFO, "%d. Sending chunk to ObjectControl, size is %d bytes",
								   ++IterationCounter, GSD->SupChunkSize);
					}
					else if (HeaderData.MessageIdU16 == ISO_HEAB_CODE) {
						//printf("[SupervisorControl] HEAB.\n");
					}


					bzero(RxBuffer,
						  ISO_MESSAGE_HEADER_LENGTH + HeaderData.MessageLengthU32 +
						  ISO_MESSAGE_FOOTER_LENGTH);
					ISOMessageReceivedU8 = 0;
					RxTotalDataU32 = 0;
					ClientResultI32 = 0;
				}
			}

			bzero(MqBuffer, SUP_MQ_MAX_SIZE);
			(void)iCommRecv(&iCommand, MqBuffer, SUP_MQ_MAX_SIZE, NULL);
			if ( /*iCommand == COMM_TRAJ_TOSUP */ GSD->ChunkSize > 0) {


				bzero(MqBuffer, SUP_MQ_MAX_SIZE);
				for (i = 0; i < GSD->ChunkSize; i++)
					MqBuffer[i] = GSD->Chunk[i];
				DTMLengthU32 = UtilHexTextToBinary(strlen(MqBuffer), MqBuffer, DTMTrajBuffer, 0);

				LogMessage(LOG_LEVEL_INFO, "Sending chunk to Supervisor, size is %d == %d bytes",
						   GSD->ChunkSize, (I32) strlen(MqBuffer));

				//for(i = 0; i < GSD->ChunkSize; i ++) DTMTrajBuffer[i] = GSD->Chunk[i];
				//DTMLengthU32 = GSD->ChunkSize;
				DTMIpU32 = (C8) DTMTrajBuffer[0];
				DTMIpU32 = (C8) DTMTrajBuffer[1] | (DTMIpU32 << 8);
				DTMIpU32 = (C8) DTMTrajBuffer[2] | (DTMIpU32 << 8);
				DTMIpU32 = (C8) DTMTrajBuffer[3] | (DTMIpU32 << 8);
				TRAJInfoData.IpAddressU32 = DTMIpU32;

				MiscU16 = (C8) DTMTrajBuffer[4];
				MiscU16 = (C8) DTMTrajBuffer[5] | (MiscU16 << 8);
				TRAJInfoData.TrajectoryIDU16 = MiscU16;
				i = 0;
				do {
					TRAJInfoData.TrajectoryNameC8[i] = (C8) DTMTrajBuffer[i + 6];
					i++;
				} while (TRAJInfoData.TrajectoryNameC8[i - 1] != 0);

				MiscU16 = (C8) DTMTrajBuffer[70];
				MiscU16 = (C8) DTMTrajBuffer[71] | (MiscU16 << 8);

				TRAJInfoData.TrajectoryVersionU16 = MiscU16;

				MiscU16 = 72;	//Number of bytes in [Ip, Id, Name, Version]  

				/*TRAJ Header */
				MessageLength =
					UtilISOBuildTRAJMessageHeader(MessageBuffer,
												  (DTMLengthU32 - MiscU16) / SIM_TRAJ_BYTES_IN_ROW,
												  &HeaderData, &TRAJInfoData, 0);
				/*Send TRAJ header */
				UtilSendTCPData("SupervisorControl", MessageBuffer, MessageLength, &SupervisorTCPSocketfdI32,
								0);
				/*TRAJ Data */
				MessageLength =
					UtilISOBuildTRAJMessage(MessageBuffer, DTMTrajBuffer + MiscU16,
											(DTMLengthU32 - MiscU16) / SIM_TRAJ_BYTES_IN_ROW, &DOTMData, 0);
				/*Send DTM data */
				UtilSendTCPData("SupervisorControl", MessageBuffer, MessageLength, &SupervisorTCPSocketfdI32,
								0);
				GSD->ChunkSize = 0;
			}

			else if (GSD->MONRSizeU8 > 0) {
				//Send MONR data
				UtilSendTCPData("SupervisorControl", GSD->MONRData, GSD->MONRSizeU8,
								&SupervisorTCPSocketfdI32, 0);
				GSD->MONRSizeU8 = 0;
			}
/*
      if(GSD->HEABSizeU8 > 0)
      {
        //Send HEAB data
        UtilSendTCPData("SupervisorControl-HEAB", GSD->HEABData, GSD->HEABSizeU8, &SupervisorTCPSocketfdI32, 0);  
        GSD->HEABSizeU8 = 0;
      }      

      if(GSD->OSTMSizeU8 > 0)
      {
        //Send OSTM data
        UtilSendTCPData("SupervisorControl-OSTM", GSD->OSTMData, GSD->OSTMSizeU8, &SupervisorTCPSocketfdI32, 1);  
        GSD->OSTMSizeU8 = 0;
      }
      
      if(GSD->OSEMSizeU8 > 0)
      {
        //Send OSEM data
        UtilSendTCPData("SupervisorControl-OSEM", GSD->OSEMData, GSD->OSEMSizeU8, &SupervisorTCPSocketfdI32, 1);  
        GSD->OSEMSizeU8 = 0;
      }

      if(GSD->STRTSizeU8 > 0)
      {
        //Send STRT data
        UtilSendTCPData("SupervisorControl-STRT", GSD->STRTData, GSD->STRTSizeU8, &SupervisorTCPSocketfdI32, 1);  
        GSD->STRTSizeU8 = 0;
      }
*/


			if (iCommand == COMM_EXIT) {
				iExit = 1;
				LogMessage(LOG_LEVEL_INFO, "Supervisor control exiting");
				(void)iCommClose();
			}
		}
	}
}

void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		iExit = 1;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}
