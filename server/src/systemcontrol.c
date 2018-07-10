/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : systemcontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include <mqueue.h>
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

#include "remotecontrol.h"
#include "util.h"
#include "systemcontrol.h"



/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
typedef enum {
  SERVER_STATE_UNDEFINED,
  SERVER_STATE_INITIALIZED,
  SERVER_STATE_IDLE,
  SERVER_STATE_READY,
  SERVER_STATE_RUNNING,
  SERVER_STATE_INWORK,
  SERVER_STATE_ERROR,
} ServerState_t;


#define SYSTEM_CONTROL_SERVICE_POLL_TIME_MS 5000

#define SYSTEM_CONTROL_CONTROL_PORT   54241       // Default port, control channel
#define SYSTEM_CONTROL_PROCESS_PORT   54242       // Default port, process channel
#define IPC_BUFFER_SIZE   1024
#define SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE 64

#define SYSTEM_CONTROL_ARG_CHAR_COUNT 		2
#define SYSTEM_CONTROL_COMMAND_MAX_LENGTH 	32
#define SYSTEM_CONTROL_ARG_MAX_COUNT	 	6
#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH	80

#define OSTM_OPT_SET_ARMED_STATE 2
#define OSTM_OPT_SET_DISARMED_STATE 3 
#define SC_RECV_MESSAGE_BUFFER 512

#define SMALL_BUFFER_SIZE_16 16
#define SMALL_BUFFER_SIZE_20 20
#define SMALL_BUFFER_SIZE_6 6
#define SMALL_BUFFER_SIZE_3 3
#define SMALL_BUFFER_SIZE_2 2
#define SYSTEM_CONTROL_SEND_BUFFER_SIZE 1024

#define SYSTEM_CONTROL_CONF_FILE_PATH  "conf/test.conf"

#define SYSTEM_CONTROL_RESPONSE_CODE_OK 						0x0001
#define SYSTEM_CONTROL_RESPONSE_CODE_ERROR 						0x0F10
#define SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE 	0x0F20  
#define SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE  		 	0x0F25  

#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_LENGTH				0x0F30
#define SYSTEM_CONTROL_RESPONSE_CODE_BUSY						0x0F40
#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_SCRIPT				0x0F50
#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_ENCRYPTION_CODE	0x0F60
#define SYSTEM_CONTROL_RESPONSE_CODE_DECRYPTION_ERROR			0x0F61


typedef enum {
	Idle_0, GetServerStatus_0, ArmScenario_0, DisarmScenario_0, StartScenario_1, stop_0, AbortScenario_0, InitializeScenario_0,
	ConnectObject_0, DisconnectObject_0, 
	replay_1, control_0, Exit_0, start_ext_trigg_1, nocommand
} SystemControlCommand_t;
const char* SystemControlCommandsArr[] = 
{ 	
	"Idle_0", "GetServerStatus_0", "ArmScenario_0", "DisarmScenario_0", "StartScenario_1", "stop_0", "AbortScenario_0", "InitializeScenario_0",
	"ConnectObject_0", "DisconnectObject_0", 
	"replay_1", "control_0", "Exit_0", "start_ext_trigg_1"
};

const char* SystemControlStatesArr[] = { "UNDEFINED", "INITIALIZED", "IDLE", "READY", "RUNNING", "INWORK", "ERROR"};
const char* SystemControlOBCStatesArr[] = { "UNDEFINED", "IDLE", "INITIALIZED", "CONNECTED", "ARMED", "RUNNING", "ERROR"};


char SystemControlCommandArgCnt[SYSTEM_CONTROL_ARG_CHAR_COUNT];
char SystemControlStrippedCommand[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
char SystemControlArgument[SYSTEM_CONTROL_ARG_MAX_COUNT][SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH];


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
//SystemControlCommand_t SystemControlFindCommandOld(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *ArgCount);
SystemControlCommand_t SystemControlFindCommand(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *ArgCount);
static I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle);
static I32 SystemControlConnectServer(int* sockfd, const char* name, const uint32_t port);
static void SystemControlSendBytes(const char* data, int length, int* sockfd, int debug);
void SystemControlSendControlResponse(U16 ResponseStatus, C8* ResponseString, C8* ResponseData, I32 ResponseDataLength, I32* Sockfd, U8 Debug);
void SystemControlSendLog(C8* LogString, I32* Sockfd, U8 Debug);
/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void systemcontrol_task(TimeType *GPSTime)
{

	int ServerHandle;
	int ClientSocket;
	int ClientResult = 0;


	ServerState_t server_state = SERVER_STATE_UNDEFINED;
	SystemControlCommand_t SystemControlCommand = Idle_0;
	SystemControlCommand_t PreviousSystemControlCommand = Idle_0;

	int CommandArgCount=0, /*CurrentCommandArgCounter=0,*/ CurrentInputArgCount=0;
	char pcBuffer[IPC_BUFFER_SIZE];
	char inchr;
	struct timeval tvTime;
	int iExit = 0;
	
	ObjectPosition OP;
	int i,i1;
	char *StartPtr, *StopPtr, *CmdPtr;
	struct timespec tTime;
	int iCommand;
  	char pcRecvBuffer[SC_RECV_MESSAGE_BUFFER];
	(void)iCommInit(IPC_RECV_SEND,MQ_SC,1);
	char ObjectIP[SMALL_BUFFER_SIZE_16];
	char ObjectPort[SMALL_BUFFER_SIZE_6];
	char TriggId[SMALL_BUFFER_SIZE_6];
	char TriggAction[SMALL_BUFFER_SIZE_6];
	char TriggDelay[SMALL_BUFFER_SIZE_20];
	uint64_t uiTime;
	U8 ModeU8 = 0;
	C8 TextBufferC8[SMALL_BUFFER_SIZE_20];
	C8 ServerIPC8[SMALL_BUFFER_SIZE_20];
	C8 UsernameC8[SMALL_BUFFER_SIZE_20];
	C8 PasswordC8[SMALL_BUFFER_SIZE_20];
	U16 ServerPortU16;
	I32 ServerSocketI32=0;
	ServiceSessionType SessionData;
	C8 RemoteServerRxData[1024];
	struct timespec sleep_time1, ref_time1;
	struct timeval CurrentTimeStruct;
 	U64 CurrentTimeU64 = 0;
 	U64 TimeDiffU64 = 0;
 	U64 OldTimeU64 = 0;
 	U64 PollRateU64 = 0;
 	C8 ControlResponseBuffer[SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE];
 	U8 OBCStateU8;

	bzero(TextBufferC8, SMALL_BUFFER_SIZE_20);
	UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, "RemoteServerMode=", "", TextBufferC8);
	ModeU8 = (U8)atoi(TextBufferC8);
	
	bzero(TextBufferC8, SMALL_BUFFER_SIZE_20);
	UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, "RemoteServerIP=", "", TextBufferC8);
	bzero(ServerIPC8, SMALL_BUFFER_SIZE_20);
	strcat(ServerIPC8, TextBufferC8);
	
	bzero(TextBufferC8, SMALL_BUFFER_SIZE_20);
	UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, "RemoteServerPort=", "", TextBufferC8);
	ServerPortU16 = (U16)atoi(TextBufferC8);
	
	bzero(TextBufferC8, SMALL_BUFFER_SIZE_20);
	UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, "RemoteServerUsername=", "", TextBufferC8);
	bzero(UsernameC8, SMALL_BUFFER_SIZE_20);
	strcat(UsernameC8, TextBufferC8);
	
	bzero(TextBufferC8, SMALL_BUFFER_SIZE_20);
	UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, "RemoteServerPassword=", "", TextBufferC8);
	bzero(PasswordC8, SMALL_BUFFER_SIZE_20);
	strcat(PasswordC8, TextBufferC8);


	printf("Mode: %d\n", ModeU8);
	printf("ServerIP: %s\n", ServerIPC8);
	printf("ServerPort: %d\n", ServerPortU16);
	printf("UsernameC8: %s\n", UsernameC8);
	printf("PasswordC8: %s\n", PasswordC8);
	if(ModeU8 == 0)
	{

	}
	else if(ModeU8 == 1) 
	{
		SessionData.SessionIdU32 = 0;
		SessionData.UserIdU32 = 0;
		SessionData.UserTypeU8 = 0;

		/* */
		PollRateU64 = SYSTEM_CONTROL_SERVICE_POLL_TIME_MS;
		CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;
		OldTimeU64 = CurrentTimeU64;

	}


	while(!iExit)
	{

		if(ModeU8 == 0)
		{
			
			if(ClientSocket <= 0)
			{	

				if(server_state == SERVER_STATE_UNDEFINED)
				{
					//Do some initialization
					server_state = SERVER_STATE_INITIALIZED;
				}

				if(USE_LOCAL_USER_CONTROL == 0) ClientResult = SystemControlInitServer(&ClientSocket, &ServerHandle);
				if(USE_LOCAL_USER_CONTROL == 1) ClientResult = SystemControlConnectServer(&ClientSocket, LOCAL_USER_CONTROL_IP, LOCAL_USER_CONTROL_PORT);

				server_state = SERVER_STATE_IDLE;
			}

			PreviousSystemControlCommand = SystemControlCommand;
			bzero(pcBuffer,IPC_BUFFER_SIZE);
			ClientResult = recv(ClientSocket, pcBuffer, IPC_BUFFER_SIZE,  0);

		  	if (ClientResult <= -1)
		  	{
		    	if(errno != EAGAIN && errno != EWOULDBLOCK)
		    	{
			  		printf("[SystemControl] Wait 5 sec before exiting.\n");
			  		usleep(5000000); //Wait 5 sec before sending exit, just so ObjectControl can send abort in HEAB before exit 
			  		(void)iCommSend(COMM_EXIT,NULL);
			  		perror("[SystemControl] ERR: Failed to receive from command socket.");
			  		exit(1);
				} 
		    }
		    else if(ClientResult == 0) 
		    {
		    	printf("[SystemControl] Client closed connection.\n");
		    	close(ClientSocket); 
		    	ClientSocket = -1;
		    	if(USE_LOCAL_USER_CONTROL == 0) { close(ServerHandle); ServerHandle = -1;}
		    	
		    	SystemControlCommand = AbortScenario_0; //Oops no client is connected, go to AbortScenario_0
		    	server_state == SERVER_STATE_UNDEFINED;
		    }
		    else
		    {
				for(i = 0; i < SYSTEM_CONTROL_ARG_MAX_COUNT; i ++ ) bzero(SystemControlArgument[i],SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH);
				CurrentInputArgCount = 0;
				StartPtr = pcBuffer;
				StopPtr = pcBuffer;
				CmdPtr = strchr(strchr(pcBuffer,'\n')+1,'\n')+1;
				StartPtr = strchr(pcBuffer, '(')+1;
				//printf("pcBuffer: %s\n", pcBuffer);
				while (StopPtr != NULL)
				{
					StopPtr = (char *)strchr(StartPtr, ',');
					if(StopPtr == NULL) strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr, (uint64_t)strchr(StartPtr, ')') - (uint64_t)StartPtr);
					else strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr, (uint64_t)StopPtr - (uint64_t)StartPtr);
					StartPtr = StopPtr+1;
					CurrentInputArgCount ++;
					//printf("CurrentInputArgCount=%d, value=%s\n", CurrentInputArgCount, SystemControlArgument[CurrentInputArgCount-1]);
				}
				
				SystemControlFindCommand(CmdPtr, &SystemControlCommand, &CommandArgCount);
			}
		}
		else if(ModeU8 == 1)
		{
			gettimeofday(&CurrentTimeStruct, NULL);
    		CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;
    		TimeDiffU64 = CurrentTimeU64 - OldTimeU64;
			
			if(ServerSocketI32 <= 0) RemoteControlConnectServer(&ServerSocketI32, ServerIPC8, ServerPortU16);

			if(ServerSocketI32 > 0 && SessionData.SessionIdU32 <= 0)
			{
				RemoteControlSignIn(ServerSocketI32, "users", UsernameC8, PasswordC8, &SessionData, 0);
				printf("SessionId: %u\n", SessionData.SessionIdU32);
				printf("UserId: %u\n", SessionData.UserIdU32);
				printf("Usertype: %d\n", SessionData.UserTypeU8);
				if(SessionData.SessionIdU32 > 0) { printf("Sign in success!\n");} else { printf("Sign in failed!\n");}
			}

			if(ServerSocketI32 > 0 && SessionData.SessionIdU32 > 0 && TimeDiffU64 > PollRateU64)
			{
				OldTimeU64 = CurrentTimeU64;
				bzero(RemoteServerRxData, 1024);
				RemoteControlSendServerStatus(ServerSocketI32, &SessionData, ++i1, 0);

			}
			//usleep(1000000);
		}

		
		if(server_state == SERVER_STATE_INWORK)
		{
			if(SystemControlCommand == AbortScenario_0)
			{
				SystemControlCommand = SystemControlCommand;
			}
			else if(SystemControlCommand == GetServerStatus_0)
			{
				printf("[SystemControl1] State: %s, OBCState: %s, PreviousCommand: %s\n", SystemControlStatesArr[server_state], SystemControlOBCStatesArr[OBCStateU8], SystemControlCommandsArr[PreviousSystemControlCommand]);
			 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			 	ControlResponseBuffer[0] = server_state;
			 	ControlResponseBuffer[1] = OBCStateU8;
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetServerStatus:", ControlResponseBuffer, 2, &ClientSocket, 0);
				SystemControlCommand = PreviousSystemControlCommand;
			} 
			else if(SystemControlCommand != PreviousSystemControlCommand)
			{				
				printf("[SystemControl] Command not allowed, SystemControl is busy in state %s, PreviousCommand: %s\n", SystemControlStatesArr[server_state], SystemControlCommandsArr[PreviousSystemControlCommand]);
				SystemControlSendLog("[SystemControl] Command not allowed, SystemControl is busy in state INWORK.\n", &ClientSocket, 0);
				bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "", ControlResponseBuffer, 0, &ClientSocket, 0);
				SystemControlCommand = PreviousSystemControlCommand;
			}
		}
		
		bzero(pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
		iCommRecv(&iCommand,pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
		if (iCommand == COMM_OBC_STATE)
		{
			OBCStateU8 = (U8)*pcRecvBuffer;
		} 
		else if(iCommand == COMM_LOG)
		{
			SystemControlSendLog(pcRecvBuffer, &ClientSocket, 0);
		}

		switch(SystemControlCommand)
		{

			case Idle_0:
	 			/*bzero(pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
				iCommRecv(&iCommand,pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
				
			 	if(iCommand == COMM_TOM)
				{
					bzero(ObjectIP, SMALL_BUFFER_SIZE_16);
					bzero(ObjectPort, SMALL_BUFFER_SIZE_6);
					bzero(TriggId, SMALL_BUFFER_SIZE_6);
					bzero(TriggAction, SMALL_BUFFER_SIZE_6);
					bzero(TriggDelay, SMALL_BUFFER_SIZE_20);

					StartPtr = pcRecvBuffer;
					StopPtr = (char *)strchr(StartPtr, ';');
					strncpy(ObjectIP, StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
					StartPtr = StopPtr + 1; StopPtr = (char *)strchr(StartPtr, ';');
					strncpy(ObjectPort, StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
					StartPtr = StopPtr + 1; StopPtr = (char *)strchr(StartPtr, ';');
					strncpy(TriggId, StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
					StartPtr = StopPtr + 1; StopPtr = (char *)strchr(StartPtr, ';');
					strncpy(TriggAction, StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
					StartPtr = StopPtr + 1; StopPtr = (char *)strchr(StartPtr, ';');
					strncpy(TriggDelay, StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
					CurrentCommandArgCounter = 1;
					strncpy(SystemControlArgument[CurrentCommandArgCounter], TriggDelay, strlen(TriggDelay));
					printf("[SystemControl] TOM recieved from %s, port=%s, TriggId=%s, TriggAction=%s, TriggDelay=%s\n", ObjectIP, ObjectPort, TriggId ,TriggAction, TriggDelay);
					fflush(stdout);
					
					if((uint8_t)atoi(TriggAction) == TAA_ACTION_EXT_START)
					{
						SystemControlCommand = start_ext_trigg_1;
						CommandArgCount = 1;
					}
					else if ((uint8_t)atoi(TriggAction) == TAA_ACTION_TEST_SIGNAL)
					{
						printf("[SystemControl] Trigg action TEST_SIGNAL not supported by server.\n");
						SystemControlCommand = Idle_0;
						CurrentCommandArgCounter = 0;
						CommandArgCount = 0;
					}
					else
					{
						printf("[SystemControl] Unknown trigg action %s.\n", TriggAction);
						SystemControlCommand = Idle_0;
						CurrentCommandArgCounter = 0;
						CommandArgCount = 0;
					}
				} 
				else
				{
					SystemControlCommand = Idle_0;
					CurrentCommandArgCounter = 0;
				}
				
				if (iCommand == COMM_OBC_STATE)
				{
					OBCStateU8 = (U8)*pcRecvBuffer;
				} 
				else if(iCommand == COMM_LOG)
				{
					SystemControlSendLog(pcRecvBuffer, &ClientSocket, 0);
				}
				*/

			break;
			case GetServerStatus_0:
				printf("[SystemControl] State: %s, OBCState: %s\n", SystemControlStatesArr[server_state], SystemControlOBCStatesArr[OBCStateU8]);
				SystemControlCommand = Idle_0;
			 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			 	ControlResponseBuffer[0] = server_state;
			 	ControlResponseBuffer[1] = OBCStateU8;
			 	printf("GPSMillisecondsU64: %ld\n", GPSTime->GPSMillisecondsU64);
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetServerStatus:", ControlResponseBuffer, 2, &ClientSocket, 0);
			break;
			case InitializeScenario_0:
				if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "IDLE") != NULL)
				{
					(void)iCommSend(COMM_INIT,pcBuffer);
					server_state = SERVER_STATE_INWORK;
				 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "InitializeScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] Sending INIT.\n", &ClientSocket, 0);
				}
				else if(server_state == SERVER_STATE_INWORK && strstr(SystemControlOBCStatesArr[OBCStateU8], "INITIALIZED") != NULL)
				{
					SystemControlSendLog("[SystemControl] Simulate that all objects becomes successfully configured.\n", &ClientSocket, 0);
					
					SystemControlCommand = Idle_0;
					server_state = SERVER_STATE_IDLE;
				}
				else if(server_state == SERVER_STATE_IDLE)
				{
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "InitializeScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] INIT received, state errors!\n", &ClientSocket, 0);
					SystemControlCommand = PreviousSystemControlCommand;
				}
			break;
			case ConnectObject_0:
				if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "INITIALIZED") != NULL)
				{
					(void)iCommSend(COMM_CONNECT,pcBuffer);
					SystemControlCommand = Idle_0;
				 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ConnectObject:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] Sending CONNECT.\n", &ClientSocket, 0);
				}
				else if(server_state == SERVER_STATE_INWORK && strstr(SystemControlOBCStatesArr[OBCStateU8], "CONNECTED") != NULL)
				{
					SystemControlSendLog("[SystemControl] Simulate that all objects are connected.\n", &ClientSocket, 0);
					
					SystemControlCommand = Idle_0;
					server_state = SERVER_STATE_IDLE;
				}
				else if(server_state == SERVER_STATE_IDLE)
				{
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "ConnectObject:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] CONNECT received, state errors!\n", &ClientSocket, 0);
					SystemControlCommand = PreviousSystemControlCommand;
				}
			break;
			case DisconnectObject_0:
				if(server_state == SERVER_STATE_IDLE)
				{
					(void)iCommSend(COMM_DISCONNECT,pcBuffer);
					SystemControlCommand = Idle_0;
				 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DisconnectObject:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] Sending DISCONNECT.\n", &ClientSocket, 0);
				}
				else
				{
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "ConnectObject:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] DISCONNECT received, state errors!\n", &ClientSocket, 0);
					SystemControlCommand = PreviousSystemControlCommand;
				}
			break;
			case ArmScenario_0:
				if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "CONNECTED") != NULL)
				{
					bzero(pcBuffer, IPC_BUFFER_SIZE);
					server_state = SERVER_STATE_INWORK;
					pcBuffer[0] = OSTM_OPT_SET_ARMED_STATE;
					(void)iCommSend(COMM_ARMD,pcBuffer);
				 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "ArmScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] Sending ARM.\n", &ClientSocket, 0);
				} 
				else if(server_state == SERVER_STATE_INWORK && strstr(SystemControlOBCStatesArr[OBCStateU8], "ARMED") != NULL)
				{
					SystemControlSendLog("[SystemControl] Simulate that all objects becomes armed.\n", &ClientSocket, 0);
					
					SystemControlCommand = Idle_0;
					server_state = SERVER_STATE_IDLE;
				}
				else if(server_state == SERVER_STATE_IDLE)
				{
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "StartScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] ARM received, state errors!\n", &ClientSocket, 0);
					SystemControlCommand = PreviousSystemControlCommand;
				}
			break;
			case DisarmScenario_0:
				if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "ARMED") != NULL)
				{
					bzero(pcBuffer, IPC_BUFFER_SIZE);
					server_state = SERVER_STATE_IDLE;
					pcBuffer[0] = OSTM_OPT_SET_DISARMED_STATE;
					(void)iCommSend(COMM_ARMD,pcBuffer);
					bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DisarmScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] Sending DISARM.\n", &ClientSocket, 0);
				}
				else if(server_state == SERVER_STATE_INWORK && strstr(SystemControlOBCStatesArr[OBCStateU8], "CONNECTED") != NULL)
				{
					SystemControlCommand = Idle_0;
					server_state = SERVER_STATE_IDLE;
				}
				else if(server_state == SERVER_STATE_IDLE)
				{
					SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "StartScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					SystemControlSendLog("[SystemControl] DISARM received, state errors!\n", &ClientSocket, 0);
					SystemControlCommand = PreviousSystemControlCommand;
				}
			break;
			case StartScenario_1:
				if(CurrentInputArgCount == CommandArgCount)
				{
					if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "ARMED") != NULL)
					{
						bzero(pcBuffer, IPC_BUFFER_SIZE);
						gettimeofday(&tvTime, NULL);	
						uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
						if(TIME_COMPENSATE_LAGING_VM) uiTime = uiTime - TIME_COMPENSATE_LAGING_VM_VAL; 

						printf("[SystemControl] Current timestamp (gtd): %lu\n",uiTime );

						//clock_gettime(CLOCK_MONOTONIC_COARSE, &tTime);
						//clock_gettime(CLOCK_REALTIME, &tTime);
						//uiTime = (uint64_t)tTime.tv_sec*1000 + (uint64_t)tTime.tv_nsec/1000000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
						//printf("[SystemControl] Current timestamp (cgt): %lu\n",uiTime );

						//printf("[SystemControl] Current timestamp: %lu\n",uiTime );
						uiTime += atoi(SystemControlArgument[0]);
						sprintf ( pcBuffer,"%" PRIu8 ";%" PRIu64 ";",0,uiTime);
						printf("[SystemControl] Sending START <%s> (delayed +%s ms)\n",pcBuffer, SystemControlArgument[0]);
						fflush(stdout);
						
						(void)iCommSend(COMM_STRT,pcBuffer);
						bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
						SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "StartScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
						server_state = SERVER_STATE_INWORK;
					} 
					else if(server_state == SERVER_STATE_INWORK && strstr(SystemControlOBCStatesArr[OBCStateU8], "RUNNING") != NULL)
					{

						SystemControlCommand = Idle_0;
						server_state = SERVER_STATE_IDLE;
					}
					else if(server_state == SERVER_STATE_IDLE)
					{
						SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "StartScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
						SystemControlSendLog("[SystemControl] START received, state errors!\n", &ClientSocket, 0);
						SystemControlCommand = PreviousSystemControlCommand;
					}

				} else printf("START command parameter count error.\n");
			break;
			/*
			case start_ext_trigg_1:
				if(CurrentCommandArgCounter == CommandArgCount)
				{
					bzero(pcBuffer, IPC_BUFFER_SIZE);
					uiTime = (uint64_t)atol(SystemControlArgument[CurrentCommandArgCounter]);
					if(uiTime == 0)
					{
						gettimeofday(&tvTime, NULL);	
						uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
					}
					
					if(TIME_COMPENSATE_LAGING_VM) uiTime = uiTime - TIME_COMPENSATE_LAGING_VM_VAL;

					sprintf (pcBuffer,"%" PRIu8 ";%" PRIu64 ";",0,uiTime);
					printf("[SystemControl] Sending START <%s> (externally trigged)\n", SystemControlArgument[CurrentCommandArgCounter]);
					fflush(stdout);
					
					(void)iCommSend(COMM_STRT,pcBuffer);
					server_state = SERVER_STATE_RUNNINGSERVER_STATE_RUNNING;
					SystemControlCommand = Idle_0;
					CurrentCommandArgCounter = 0;
				} else CurrentCommandArgCounter ++;
			break;*/
			case stop_0:
				(void)iCommSend(COMM_STOP,NULL);
				server_state = SERVER_STATE_IDLE;
				SystemControlCommand = Idle_0;
				bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "stop:", ControlResponseBuffer, 0, &ClientSocket, 0);
			break;
		    case AbortScenario_0:
		        if(strstr(SystemControlOBCStatesArr[OBCStateU8], "CONNECTED") != NULL || strstr(SystemControlOBCStatesArr[OBCStateU8], "ARMED") != NULL || strstr(SystemControlOBCStatesArr[OBCStateU8], "RUNNING") != NULL)
		        {
					(void)iCommSend(COMM_ABORT,NULL);
					server_state = SERVER_STATE_IDLE;
					SystemControlCommand = Idle_0;
					if(ClientSocket >= 0)
					{
						bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
						SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "AbortScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
					}
		        }
				else
				{
					if(ClientSocket >= 0)
					{
						bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
						SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE, "AbortScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
						SystemControlSendLog("[SystemControl] ABORT received, state errors!\n", &ClientSocket, 0);
					}
					server_state = SERVER_STATE_IDLE;
					SystemControlCommand = Idle_0;
				}
		    break;
			/*
			case replay_1:
				if(CurrentCommandArgCounter == CommandArgCount)
				{
					if(!strcmp(SystemControlArgument[CurrentCommandArgCounter],"-help"))
					{
						printf("[SystemControl] -----REPLAY-----\n");
						printf("[SystemControl] Syntax: replay [arg]\n");
						printf("[SystemControl] Ex: replay log/33/event.log\n");
						fflush(stdout);
					}
					else
					{
						(void)iCommSend(COMM_REPLAY, SystemControlArgument[CurrentCommandArgCounter]);
						printf("[SystemControl] System control sending REPLAY on IPC <%s>\n", SystemControlArgument[CurrentCommandArgCounter]);
						fflush(stdout);
					}
					SystemControlCommand = idle_0;
					CurrentCommandArgCounter = 0;
				} else CurrentCommandArgCounter ++;
			break;
			case control_0:
				(void)iCommSend(COMM_CONTROL, NULL);
				//printf("INF: System control sending CONTROL on IPC <%s>\n", pcBuffer);
				fflush(stdout);
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;*/
			case Exit_0:
				(void)iCommSend(COMM_EXIT,NULL);
				iExit = 1;
				usleep(1000000);
				SystemControlCommand = Idle_0;
				bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "Exit:", ControlResponseBuffer, 0, &ClientSocket, 0);
				close(ClientSocket); ClientSocket = -1;
		    	if(USE_LOCAL_USER_CONTROL == 0) { close(ServerHandle); ServerHandle = -1;}
		    	printf("[SystemControl] Server closing.\n");
				
			break;
			
			default:

	
			break;
		}

		usleep(100);
  	}

  (void)iCommClose();
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

SystemControlCommand_t SystemControlFindCommand(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *CommandArgCount)
{

	SystemControlCommand_t command; 	
	char StrippedCommandBuffer[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
	bzero(StrippedCommandBuffer, SYSTEM_CONTROL_COMMAND_MAX_LENGTH);
	//printf("CommandBuffer: %s\n", CommandBuffer);
	strncpy(StrippedCommandBuffer, CommandBuffer, (uint64_t)strchr(CommandBuffer, '(') - (uint64_t)CommandBuffer );
	//printf("StrippedCommandBuffer: %s\n", StrippedCommandBuffer);

	for (command = Idle_0; command != nocommand; command++)
    {
		bzero(SystemControlCommandArgCnt, SYSTEM_CONTROL_ARG_CHAR_COUNT);    
        bzero(SystemControlStrippedCommand, SYSTEM_CONTROL_COMMAND_MAX_LENGTH);
		strncpy(SystemControlStrippedCommand, SystemControlCommandsArr[(int)command], (uint64_t)strchr(SystemControlCommandsArr[(int)command],'_') - (uint64_t)SystemControlCommandsArr[(int)command] );
		strncpy(SystemControlCommandArgCnt, strchr(SystemControlCommandsArr[(int)command],'_')+1, strlen(SystemControlCommandsArr[(int)command]) - ((uint64_t)strchr(SystemControlCommandsArr[(int)command],'_') - (uint64_t)SystemControlCommandsArr[(int)command] + 1));
		
		if (!strcmp(SystemControlStrippedCommand, StrippedCommandBuffer))
        {
			{
				*CommandArgCount = atoi(SystemControlCommandArgCnt);				
				*CurrentCommand = command;            
				return command;
			} 
        }
    }
    return nocommand;
}


void SystemControlSendLog(C8* LogString, I32* Sockfd, U8 Debug)
{
	int i, n, j, t;
	C8 Length[4];
	C8 Header[2] = {0,2};
	C8 Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = 2 + strlen(LogString);
	Length[0] = (C8)(n >> 24); Length[1] = (C8)(n >> 16); Length[2] = (C8)(n >> 8); Length[3] = (C8)n;
	
	//SystemControlSendBytes(Length, 4, Sockfd, 0);
	//SystemControlSendBytes(Header, 5, Sockfd, 0);
	//SystemControlSendBytes(LogString, strlen(LogString), Sockfd, 0);

	
	if(n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE)
	{
		for(i = 0, j = 0; i < 4; i++, j++) Data[j] = Length[i];
		for(i = 0; i < 2; i++, j++) Data[j] = Header[i];
		t = strlen(LogString);
		for(i = 0; i < t; i++, j++) Data[j] = *(LogString+i);
		SystemControlSendBytes(Data, n + 4, Sockfd, 0);
	} else printf("[SystemControl] Log string longer then %d bytes!\n", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	
}



void SystemControlSendControlResponse(U16 ResponseStatus, C8* ResponseString, C8* ResponseData, I32 ResponseDataLength, I32* Sockfd, U8 Debug)
{
	int i, n, j, t;
	C8 Length[4];
	C8 Status[2];
	C8 Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];
	
	bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
	n = 2 + strlen(ResponseString) + ResponseDataLength;
	Length[0] = (C8)(n >> 24); Length[1] = (C8)(n >> 16); Length[2] = (C8)(n >> 8); Length[3] = (C8)n;
	Status[0] = (C8)(ResponseStatus >> 8); Status[1] = (C8)ResponseStatus; 

	if(n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE)
	{
		for(i = 0, j = 0; i < 4; i++, j++) Data[j] = Length[i];
		for(i = 0; i < 2; i++, j++) Data[j] = Status[i];
		t = strlen(ResponseString);
		for(i = 0; i < t; i++, j++) Data[j] = *(ResponseString+i);
		for(i = 0; i < ResponseDataLength; i++, j++) Data[j] = ResponseData[i];
		SystemControlSendBytes(Data, n + 4, Sockfd, 0);
	} else printf("[SystemControl] Response data more then %d bytes!\n", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
}


static void SystemControlSendBytes(const char* data, int length, int* sockfd, int debug)
{
  int i, n;

  if(debug == 1){ printf("Bytes sent: "); int i = 0; for(i = 0; i < length; i++) printf("%d-", (unsigned char)*(data+i)); printf("\n");}

  n = write(*sockfd, data, length);
  if (n < 0)
  {
    util_error("[SystemControl] ERR: Failed to send on control socket");
  }
}



static I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle)
{

	struct sockaddr_in command_server_addr;
	struct sockaddr_in cli_addr;
	socklen_t cli_length;
	unsigned int control_port = SYSTEM_CONTROL_CONTROL_PORT;
	int optval = 1;
	int result = 0;

	/* Init user control socket */
	printf("[SystemControl] Init control socket\n");
	fflush(stdout);
	
	*ServerHandle = socket(AF_INET, SOCK_STREAM, 0);
	if (*ServerHandle < 0)
	{
	  perror("[SystemControl] ERR: Failed to create control socket");
	  exit(1);
	}
	bzero((char *) &command_server_addr, sizeof(command_server_addr));

	command_server_addr.sin_family = AF_INET;
	command_server_addr.sin_addr.s_addr = INADDR_ANY; 
	command_server_addr.sin_port = htons(control_port);

	optval = 1;
	result = setsockopt(*ServerHandle, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

	if (result < 0)
	{
	  perror("[SystemControl] ERR: Failed to call setsockopt");
	  exit(1);
	}

	if (bind(*ServerHandle, (struct sockaddr *) &command_server_addr, sizeof(command_server_addr)) < 0) 
	{
	  perror("[SystemControl] ERR: Failed to bind to control socket");
	  exit(1);
	}

	/* Monitor and control sockets up. Wait for central to connect to control socket to get server address*/
	printf("[SystemControl] Listening for connection from client ...\n");
	fflush(stdout);
	

	listen(*ServerHandle, 1);
	cli_length = sizeof(cli_addr);

	
	fflush(stdout);


	while( *ClientSocket = accept(*ServerHandle, (struct sockaddr *) &cli_addr, &cli_length))
	{

		printf("[SystemControl] Connection accepted!\n");
		break;

	}
	
	printf("[SystemControl] Connection received: %i\n", htons(command_server_addr.sin_port));

	if (*ClientSocket < 0) 
	{
	  perror("[SystemControl] ERR: Failed to accept from central");
	  exit(1);
	}

	/* set socket to non-blocking */
	result = fcntl(*ClientSocket, F_SETFL, fcntl(*ClientSocket, F_GETFL, 0) | O_NONBLOCK);

	return result;
}



static I32 SystemControlConnectServer(int* sockfd, const char* name, const uint32_t port)
{
  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  char buffer[256];
  int iResult;

  *sockfd = socket(AF_INET, SOCK_STREAM, 0);
   
  if (*sockfd < 0) 
  {
    util_error("[SystemControl] ERR: Failed to open control socket");
  }

  server = gethostbyname(name);
  if (server == NULL) 
  {
    util_error("[SystemControl] ERR: Unknown host ");
  }
  
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  
  bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(port);
  
  #ifdef DEBUG
    printf("[SystemControl] Try to connect to control socket: %s %i\n",name,port);
    fflush(stdout);
  #endif
  
  do
  {
    iResult = connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

    if ( iResult < 0) 
    {
      if(errno == ECONNREFUSED)
      {
        printf("[SystemControl] Was not able to connect to UserControl, retry in 3 sec...\n");
        fflush(stdout);
        (void)sleep(3);
      }
      else
      {
        util_error("[SystemControl] ERR: Failed to connect to control socket");
      }
    }
  } while(iResult < 0);

  
	iResult = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  //#ifdef DEBUG
    printf("[SystemControl] Maestro connected to UserControl: %s %i\n",name,port);
    fflush(stdout);
  //#endif

  return iResult;

}