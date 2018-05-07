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
#include "systemcontrol.h"
#include "util.h"



/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
typedef enum {
  SERVER_STATUS_UNDEFINED,
  SERVER_STATUS_INIT,
  SERVER_STATUS_IDLE,
  SERVER_STATUS_READY,
  SERVER_STATUS_RUNNING,
  SERVER_STATUS_INWORK,
  SERVER_STATUS_FAIL,
} state_t;


#define SYSTEM_CONTROL_SERVICE_POLL_TIME_MS 5000

#define SYSTEM_CONTROL_CONTROL_PORT   54241       // Default port, control channel
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

#define SYSTEM_CONTROL_CONF_FILE_PATH  "conf/test.conf"

typedef enum {
	idle_0, getserverstatus_0, arm_0, disarm_0, start_1, stop_0, abort_0, replay_1, 
	control_0, exit_0, cx_0, cc_0, listen_0, start_ext_trigg_1, nocommand
} SystemControlCommand_t;
const char* SystemControlCommandsArr[] = 
{ 	
	"idle_0", "getserverstatus_0", "arm_0", "disarm_0", "start_1", "stop_0", "abort_0", "replay_1", 
	"control_0", "exit_0", "cx_0", "cc_0", "listen_0", "start_ext_trigg_1"
};

const char* SystemControlSeverStatesArr[] = { "UNDEFINED", "INIT", "IDLE", "READY", "RUNNING", "INWORK", "FAIL"};

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
void SystemControlSendControlResponse(C8* ResponseString, C8* ResponseData, I32 ResponseDataLength, I32* Sockfd, U8 Debug);
/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void systemcontrol_task()
{

	int ServerHandle;
	int ClientSocket;
	int ClientResult = 0;


	state_t server_state = SERVER_STATUS_IDLE;
	SystemControlCommand_t SystemControlCommand = idle_0;
	SystemControlCommand_t PreviousSystemControlCommand = idle_0;

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
				if(USE_LOCAL_USER_CONTROL == 0) ClientResult = SystemControlInitServer(&ClientSocket, &ServerHandle);
				if(USE_LOCAL_USER_CONTROL == 1) ClientResult = SystemControlConnectServer(&ClientSocket, LOCAL_USER_CONTROL_IP, LOCAL_USER_CONTROL_PORT);
			}

			PreviousSystemControlCommand = SystemControlCommand;
			bzero(pcBuffer,IPC_BUFFER_SIZE);
			ClientResult = recv(ClientSocket, pcBuffer, IPC_BUFFER_SIZE,  0);

		  	if (ClientResult <= -1)
		  	{
		    	if(errno != EAGAIN && errno != EWOULDBLOCK)
		    	{
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
		    	if(USE_LOCAL_USER_CONTROL == 0) close(ServerHandle);
		    	//SystemControlCommand = listen_0;
		    	//Oops no client is connected, send abort to ObjectControl
		    	SystemControlCommand = abort_0;
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

		
		if(server_state != SERVER_STATUS_IDLE)
		{
			if(SystemControlCommand == getserverstatus_0)
			{
				printf("[SystemControl] Server status: %s, %s\n", SystemControlSeverStatesArr[server_state], SystemControlCommandsArr[PreviousSystemControlCommand]);
			 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			 	ControlResponseBuffer[0] = server_state;
				SystemControlSendControlResponse("getserverstatus:", ControlResponseBuffer, 1, &ClientSocket, 0);				
			}

			if(server_state == SERVER_STATUS_READY && (SystemControlCommand == start_1 || SystemControlCommand == abort_0)) SystemControlCommand = SystemControlCommand;
			else if (server_state == SERVER_STATUS_RUNNING && SystemControlCommand == abort_0) SystemControlCommand = SystemControlCommand;
			else SystemControlCommand = PreviousSystemControlCommand;
		}
		
		switch(SystemControlCommand)
		{

			case idle_0:
	 			bzero(pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
				iCommRecv(&iCommand,pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
				/*
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
						SystemControlCommand = idle_0;
						CurrentCommandArgCounter = 0;
						CommandArgCount = 0;
					}
					else
					{
						printf("[SystemControl] Unknown trigg action %s.\n", TriggAction);
						SystemControlCommand = idle_0;
						CurrentCommandArgCounter = 0;
						CommandArgCount = 0;
					}
				} 
				else
				{
					SystemControlCommand = idle_0;
					CurrentCommandArgCounter = 0;
				}
				*/
			break;
			/*case listen_0:
				printf("[SystemControl] Listening for new client connection...\n");
				ClientResult = SystemControlInitServer(&ClientSocket, &ServerHandle);
				SystemControlCommand = idle_0;
			break;*/
			case getserverstatus_0:
				printf("[SystemControl] Server status: %s\n", SystemControlSeverStatesArr[server_state]);
				SystemControlCommand = idle_0;
			 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
			 	ControlResponseBuffer[0] = server_state;
				SystemControlSendControlResponse("getserverstatus:", ControlResponseBuffer, 1, &ClientSocket, 0);
			break;
			case arm_0:
				if( server_state == SERVER_STATUS_IDLE)
				{
					bzero(pcBuffer, IPC_BUFFER_SIZE);
					server_state = SERVER_STATUS_INWORK;
					pcBuffer[0] = OSTM_OPT_SET_ARMED_STATE;
					(void)iCommSend(COMM_ARMD,pcBuffer);
					printf("[SystemControl] Sending ARM.\n");
					//SystemControlCommand = idle_0;
				 	bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse("arm:", ControlResponseBuffer, 0, &ClientSocket, 0);
				} 
				else if(server_state == SERVER_STATUS_INWORK)
				{
					printf("IN_WORK: Simulate that all objects becomes armed. Server goes to READY state.\n");
					
					SystemControlCommand = idle_0;
					server_state = SERVER_STATUS_READY;
				}

			break;
			case disarm_0:
				bzero(pcBuffer, IPC_BUFFER_SIZE);
				server_state = SERVER_STATUS_IDLE;
				pcBuffer[0] = OSTM_OPT_SET_DISARMED_STATE;
				(void)iCommSend(COMM_ARMD,pcBuffer);
				printf("[SystemControl] Sending DISARM.\n");
				//SystemControlCommand = idle_0;
				bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse("disarm:", ControlResponseBuffer, 0, &ClientSocket, 0);
			break;
			case start_1:
				if(CurrentInputArgCount == CommandArgCount)
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
					server_state = SERVER_STATUS_RUNNING;
					SystemControlCommand = idle_0;
					bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
					SystemControlSendControlResponse("start:", ControlResponseBuffer, 0, &ClientSocket, 0);
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
					server_state = SERVER_STATUS_RUNNING;
					SystemControlCommand = idle_0;
					CurrentCommandArgCounter = 0;
				} else CurrentCommandArgCounter ++;
			break;*/
			case stop_0:
				(void)iCommSend(COMM_STOP,NULL);
				server_state = SERVER_STATUS_IDLE;
				SystemControlCommand = idle_0;
				bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse("stop:", ControlResponseBuffer, 0, &ClientSocket, 0);
			break;
		    case abort_0:
		        (void)iCommSend(COMM_ABORT,NULL);
		        server_state = SERVER_STATUS_IDLE;
		        SystemControlCommand = idle_0;
		        bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
				SystemControlSendControlResponse("abort:", ControlResponseBuffer, 0, &ClientSocket, 0);
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
			break;
			case exit_0:
				(void)iCommSend(COMM_EXIT,NULL);
				iExit = 1;
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;
			case cx_0:
				//printf("Argument count: %d\n", CommandArgCount);
				//printf("CurrentCommandArgCounter: %d\n", CurrentCommandArgCounter);
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
				CommandArgCount = 0;
				//printf("Current active command: %s\n", SystemControlCommandsArr[SystemControlCommand]);
				fflush(stdout);
			break;
			case cc_0:
				printf("[SystemControl] Command waiting for input: %s\n", SystemControlCommandsArr[PreviousSystemControlCommand]);				
				printf("[SystemControl] CurrentCommandArgCounter: %d\n", CurrentCommandArgCounter-1);
				fflush(stdout);
				SystemControlCommand = PreviousSystemControlCommand;
			break;*/
			
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

	for (command = idle_0; command != nocommand; command++)
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



void SystemControlSendControlResponse(C8* ResponseString, C8* ResponseData, I32 ResponseDataLength, I32* Sockfd, U8 Debug)
{
	int i, n;
	C8 Length[4];

	n = strlen(ResponseString) + ResponseDataLength;
	Length[0] = (C8)(n >> 24); Length[1] = (C8)(n >> 16); Length[2] = (C8)(n >> 8); Length[3] = (C8)n;
	SystemControlSendBytes(Length, 4, Sockfd, Debug);
	SystemControlSendBytes(ResponseString, strlen(ResponseString), Sockfd, Debug);
	SystemControlSendBytes(ResponseData, ResponseDataLength, Sockfd, Debug);
}


static void SystemControlSendBytes(const char* data, int length, int* sockfd, int debug)
{
  int i, n;

  if(debug == 1){ printf("Bytes sent: "); int i = 0; for(i = 0; i < length; i++) printf("%x-", (unsigned char)*(data+i)); printf("\n");}

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