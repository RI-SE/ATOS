/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : systemcontrol.c
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include "systemcontrol.h"



#include "util.h"


/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/
typedef enum {
  SERVER_STATUS_INIT,
  SERVER_STATUS_OBJECT_CONNECTED,
  SERVER_STATUS_OBJECT_LOADED,
  SERVER_STATUS_ARMED,
  SERVER_STATUS_RUNNING,
  SERVER_STATUS_STOPPED,
  SERVER_STATUS_ABORTED,
  SERVER_STATUS_DONE
} state_t;


#define SYSTEM_CONTROL_CONTROL_PORT   54241       // Default port, control channel
#define IPC_BUFFER_SIZE   256

#define SYSTEM_CONTROL_ARG_CHAR_COUNT 		2
#define SYSTEM_CONTROL_COMMAND_MAX_LENGTH 	10
#define SYSTEM_CONTROL_ARG_MAX_COUNT	 	6
#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH	80


typedef enum {
   idle_0, status_0, arm_0, start_0, stop_0, abort_0, replay_1, control_0, exit_0, cx_0, cc_0, nocommand
} SystemControlCommand_t;
const char* SystemControlCommandsArr[] = { "idle_0", "status_0", "arm_0", "start_0", "stop_0", "abort_0", "replay_1", "control_0", "exit_0", "cx_0", "cc_0"};
SystemControlCommand_t PreviousSystemControlCommand = nocommand;
char SystemControlCommandArgCnt[SYSTEM_CONTROL_ARG_CHAR_COUNT];
char SystemControlStrippedCommand[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
char SystemControlArgument[SYSTEM_CONTROL_ARG_MAX_COUNT][SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH];


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
SystemControlCommand_t SystemControlFindCommand(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *ArgCount);
int SystemControlInitServer(int *ClientSocket);
/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void systemcontrol_task()
{

	int ClientSocket;
	int ClientResult = 0;

	ClientResult = SystemControlInitServer(&ClientSocket);

	state_t server_state = SERVER_STATUS_INIT;
	SystemControlCommand_t SystemControlCommand = idle_0;

	int CommandArgCount=0, CurrentCommandArgCounter=0, CurrentInputArgCount;
	char pcBuffer[IPC_BUFFER_SIZE];
	char inchr;
	struct timeval tvTime;
	int iExit = 0;
	(void)iCommInit(IPC_SEND,MQ_SC,0);
	ObjectPosition OP;
	int i;
	char *StartPtr, *StopPtr;

	while(!iExit)
	{
		bzero(pcBuffer,IPC_BUFFER_SIZE);
		ClientResult = recv(ClientSocket, pcBuffer, IPC_BUFFER_SIZE, 0);
				
	  	if (ClientResult <= 0)
	  	{
	    	if(errno != EAGAIN && errno != EWOULDBLOCK)
	    	{
		  		perror("ERR: Failed to receive from command socket");
		  		exit(1);
			}
	    }
	    else
	    {

			for(i = 0; i < SYSTEM_CONTROL_ARG_MAX_COUNT; i ++ ) bzero(SystemControlArgument[i],SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH);
			CurrentInputArgCount = 0;
			StartPtr = pcBuffer;
			StopPtr = pcBuffer;
			while (StopPtr != NULL)
			{
				StopPtr = (char *)strchr(StartPtr, ' ');
				if(StopPtr != NULL) strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr, (uint64_t)StopPtr-(uint64_t)StartPtr);
				else strncpy(SystemControlArgument[CurrentInputArgCount], StartPtr, strlen(StartPtr));
				StartPtr = StopPtr+1;
				CurrentInputArgCount ++;
			}

	      	SystemControlFindCommand(SystemControlArgument[0], &SystemControlCommand, &CommandArgCount);
	  	}
		
		switch(SystemControlCommand)
		{
			case idle_0:
 				CurrentCommandArgCounter = 0;
			break;
			case status_0:
				printf("Server status: %d\n", server_state);
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;
			case arm_0:
				(void)iCommSend(COMM_ARMD,NULL);
			    server_state = SERVER_STATUS_ARMED;
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;
			case start_0:
				bzero(pcBuffer, IPC_BUFFER_SIZE);

				gettimeofday(&tvTime, NULL);

				uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - 
				MS_FROM_1970_TO_2004_NO_LEAP_SECS + 
				DIFF_LEAP_SECONDS_UTC_ETSI*1000;

				/* Add 5 seconds to get room for all objects to get command */
				uiTime += 5000;

				sprintf ( pcBuffer,"%" PRIu8 ";%" PRIu64 ";",0,uiTime);

				//#ifdef DEBUG
				printf("INF: System control Sending TRIG on IPC <%s>\n",pcBuffer);
				fflush(stdout);
				//#endif

				(void)iCommSend(COMM_TRIG,pcBuffer);
				server_state = SERVER_STATUS_RUNNING;
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;
			case stop_0:
				(void)iCommSend(COMM_STOP,NULL);
				server_state = SERVER_STATUS_STOPPED;
				SystemControlCommand = idle_0;
				CurrentCommandArgCounter = 0;
			break;
		    case abort_0:
		        (void)iCommSend(COMM_ABORT,NULL);
		        server_state = SERVER_STATUS_ABORTED;
		        SystemControlCommand = idle_0;
		        CurrentCommandArgCounter = 0;
		    break;
			case replay_1:
				if(CurrentCommandArgCounter == CommandArgCount)
				{
					if(!strcmp(SystemControlArgument[CurrentCommandArgCounter],"-help"))
					{
						printf("-----REPLAY-----\n");
						printf("Syntax: replay [arg]\n");
						printf("Ex: replay log/33/event.log\n");
						fflush(stdout);
					}
					else
					{
						(void)iCommSend(COMM_REPLAY, SystemControlArgument[CurrentCommandArgCounter]);
						printf("INF: System control sending REPLAY on IPC <%s>\n", SystemControlArgument[CurrentCommandArgCounter]);
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
				printf("[Server]Command waiting for input: %s\n", SystemControlCommandsArr[PreviousSystemControlCommand]);				
				printf("[Server]CurrentCommandArgCounter: %d\n", CurrentCommandArgCounter-1);
				fflush(stdout);
				SystemControlCommand = PreviousSystemControlCommand;
			break;
			
			default:

			break;

		}

  }
  (void)iCommClose();
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

SystemControlCommand_t SystemControlFindCommand(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *CommandArgCount)
{

	SystemControlCommand_t command; 	
	
	for (command = idle_0; command != nocommand; command++)
    {
		bzero(SystemControlCommandArgCnt, SYSTEM_CONTROL_ARG_CHAR_COUNT);    
        bzero(SystemControlStrippedCommand, SYSTEM_CONTROL_COMMAND_MAX_LENGTH);
		strncpy(SystemControlStrippedCommand, SystemControlCommandsArr[(int)command], (uint64_t)strchr(SystemControlCommandsArr[(int)command],'_') - (uint64_t)SystemControlCommandsArr[(int)command] );
		strncpy(SystemControlCommandArgCnt, strchr(SystemControlCommandsArr[(int)command],'_')+1, strlen(SystemControlCommandsArr[(int)command]) - ((uint64_t)strchr(SystemControlCommandsArr[(int)command],'_') - (uint64_t)SystemControlCommandsArr[(int)command] + 1));
		
		if (!strcmp(SystemControlStrippedCommand, CommandBuffer))
        {
			if(command != cc_0)
			{
				*CommandArgCount = atoi(SystemControlCommandArgCnt);				
				*CurrentCommand = command;            
				return command;
			} 
			else
			{
				PreviousSystemControlCommand = *CurrentCommand;
				*CurrentCommand = command;
				return command;
			}
        }
    }
    return nocommand;
}



int SystemControlInitServer(int *ClientSocket)
{
	int command_server_socket_fd;
	//int command_com_socket_fd;
	struct sockaddr_in command_server_addr;
	struct sockaddr_in cli_addr;
	socklen_t cli_length;
	unsigned int control_port = SYSTEM_CONTROL_CONTROL_PORT;
	int optval = 1;
	int result = 0;

	/* Init user control socket */
	printf("Init control socket\n");
	fflush(stdout);
	
	command_server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (command_server_socket_fd < 0)
	{
	  perror("ERR: Failed to create control socket");
	  exit(1);
	}
	bzero((char *) &command_server_addr, sizeof(command_server_addr));

	command_server_addr.sin_family = AF_INET;
	command_server_addr.sin_addr.s_addr = INADDR_ANY; 
	command_server_addr.sin_port = htons(control_port);

	optval = 1;
	result = setsockopt(command_server_socket_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

	if (result < 0)
	{
	  perror("ERR: Failed to call setsockopt");
	  exit(1);
	}

	if (bind(command_server_socket_fd, (struct sockaddr *) &command_server_addr, sizeof(command_server_addr)) < 0) 
	{
	  perror("ERR: Failed to bind to control socket");
	  exit(1);
	}

	/* Monitor and control sockets up. Wait for central to connect to control socket to get server address*/
	printf("Listening for connection on user control socket ...\n");
	fflush(stdout);
	

	listen(command_server_socket_fd, 5);
	cli_length = sizeof(cli_addr);

	printf("Connection received on user control socket: %i \n", htons(command_server_addr.sin_port));
	fflush(stdout);


	*ClientSocket = accept(command_server_socket_fd, (struct sockaddr *) &cli_addr, &cli_length);
	if (*ClientSocket < 0) 
	{
	  perror("ERR: Failed to accept from central");
	  exit(1);
	}

	/* set socket to non-blocking */
	result = fcntl(*ClientSocket, F_SETFL, fcntl(*ClientSocket, F_GETFL, 0) | O_NONBLOCK);

	return result;
}

