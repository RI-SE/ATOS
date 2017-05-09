/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2017 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : usercontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>

#include "usercontrol.h"
#include "util.h"

#define IPC_BUFFER_SIZE   256
#define USER_CONTROL_SYSTEM_CONTROL_IP "127.0.0.1"  
#define USER_CONTROL_SYSTEM_CONTROL_PORT 54241

#define USER_CONTROL_ARG_COUNT 			2
#define USER_CONTROL_COMMAND_MAX_LENGTH 	10


typedef enum {
   idle_0, status_0, arm_0, start_0, stop_0, abort_0, replay_1, control_0, exit_0, cx_0, cc_0, cp_0, sb_0, cb_0, tp_0, tsp_1, tc_0, ts_0, help_0, nocommand
} UserControlCommand_t;
const char* UserControlCommandsArr[] = { "idle_0", "status_0", "arm_0", "start_0", "stop_0", "abort_0", "replay_1", "control_0", "exit_0", "cx_0", "cc_0", "cp_0", "sb_0", "cb_0", "tp_0", "tsp_1", "tc_0", "ts_0", "help_0" };
UserControlCommand_t PreviousUserControlCommand = nocommand;
char UserControlCommandArgCnt[USER_CONTROL_ARG_COUNT];
char UserControlStrippedCommand[USER_CONTROL_COMMAND_MAX_LENGTH];

UserControlCommand_t UserControlCommand = idle_0;
int CommandArgCount=0, CurrentCommandArgCount=0;
char SendBuffer[IPC_BUFFER_SIZE*2];
char RecordBuffer[IPC_BUFFER_SIZE*2];
char *SendBufferPtr;


/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
UserControlCommand_t UserControlFindCommand(const char* CommandBuffer, UserControlCommand_t *CurrentCommand, int *ArgCount);
void UserControlResetInputVariables();
static void UserControlConnectServer(int* sockfd, const char* name, const uint32_t port);
static void UserControlSendString(const char* command, int* sockfd);
static void UserControlDisconnectObject(int* sockfd);


void usercontrol_task()
{

	char pcBuffer[IPC_BUFFER_SIZE];
	char inchr;
	struct timeval tvTime;
	int iExit = 0;
	ObjectPosition OP;
	SendBufferPtr = SendBuffer;
	bzero(SendBuffer,IPC_BUFFER_SIZE*2);

	printf("Connecting to server...\n");
	int socketfd;
	char object_address_name[50];
	bzero(object_address_name,50);
	strcat(object_address_name, USER_CONTROL_SYSTEM_CONTROL_IP);
	uint32_t object_tcp_port = USER_CONTROL_SYSTEM_CONTROL_PORT;
	UserControlConnectServer(&socketfd, object_address_name, object_tcp_port);
	if (socketfd >= 0) 
  	{
  		printf("UserControl client up.\n"); 
		while(!iExit)
		{
			
			bzero(pcBuffer,IPC_BUFFER_SIZE);
			scanf("%49s",pcBuffer);
		
			if(strcmp(pcBuffer, "cc") != 0 && strcmp(pcBuffer, "cx") != 0 && strcmp(pcBuffer, "cp") != 0 && strcmp(pcBuffer, "sb") != 0 && strcmp(pcBuffer, "help") != 0 && strcmp(pcBuffer, "cb") != 0)
			{
				strncat(SendBufferPtr, pcBuffer, strlen(pcBuffer));
				SendBuffer[strlen(SendBuffer)] = 32;
				SendBufferPtr = SendBuffer + strlen(SendBuffer);
				bzero(RecordBuffer, IPC_BUFFER_SIZE*2);
				strncpy(RecordBuffer, SendBuffer, strlen(SendBuffer));
			}			

			UserControlFindCommand(pcBuffer, &UserControlCommand, &CommandArgCount);
			
			
			switch(UserControlCommand)
			{
				case idle_0:
	 				CurrentCommandArgCount = 0;
	 			break;
				case status_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case arm_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case start_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case stop_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
			    case abort_0:
	   				UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
			    break;
				case replay_1:
					if(CurrentCommandArgCount == CommandArgCount)
					{
						if(!strcmp(pcBuffer,"-help"))
						{
							UserControlSendString(SendBuffer, &socketfd);
						}
						else
						{
							UserControlSendString(SendBuffer, &socketfd);
						}
						UserControlResetInputVariables();
					} else CurrentCommandArgCount ++;
				break;
				case control_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case exit_0:
					iExit = 1;
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case cx_0:
					UserControlSendString("cx", &socketfd);
					UserControlResetInputVariables();
				break;
				case cc_0:
					UserControlSendString("cc", &socketfd);
					printf("Sendbuffer: %s\n", SendBuffer);
					printf("Recordbuffer: %s\n", RecordBuffer);
					UserControlCommand = PreviousUserControlCommand;
				break;
				case cp_0:
					UserControlResetInputVariables();
					strncpy(SendBuffer, RecordBuffer, strlen(RecordBuffer));
					printf("Copy recorded buffer to send buffer\n");
				break;
				case sb_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case tp_0:
					//UserControlCommand = idle_0;
	//				UtilCalcPositionDelta(57.6626302333,12.1056869167,57.6626269972, 12.1057250694, &OP);

					//UtilCalcPositionDelta(57.466397884,12.469130065,57.466397865, 12.469129955, &OP);
					//UtilCalcPositionDelta(57.7773298066,12.7818834416,57.777329775, 12.7818832583, &OP);
					//UtilCalcPositionDelta(57.7771230833333,12.78156473, 57.777711,12.780829, &OP);
					UtilCalcPositionDelta(57.777360,12.780472, 57.777711,12.780829, &OP);

	//				UtilCalcPositionDelta(57.7773716086,12.7804629583,57.7773717086, 12.7804630583, &OP);
					printf("p1(57.777125795, 12.781569353) -> p2(57.777156948, 12.781550638) => Calc d = %4.3f m, iterations = %d\n", OP.OrigoDistance, OP.CalcIterations);
					printf("Latitude = %3.10f \n", OP.Latitude);
					printf("Longitude = %3.10f\n", OP.Longitude);
					printf("ForwardAzimuth1 = %3.10f \n", OP.ForwardAzimuth1);
					printf("ForwardAzimuth2 = %3.10f\n", OP.ForwardAzimuth2);
					printf("DeltaForwardAzimuth = %3.15f \n", OP.ForwardAzimuth1-OP.ForwardAzimuth2);
					printf("x = %4.15lf\n", OP.x);
					printf("y = %4.15lf\n", OP.y);

					//printf("d = 0.590 m, calc d = %4.3f m\n", UtilCalcPositionDelta(57.6626340000,12.1056979028,57.6626332417, 12.1057076556));
					//printf("d = 2.643 m, calc d = %4.3f m\n", UtilCalcPositionDelta(57.6626378750,12.1056752083,57.6626339472, 12.1057190472));
					//printf("d = 1.901 m, calc d = %4.3f m\n", UtilCalcPositionDelta(57.6626295889,12.1056851083,57.6626267528, 12.1057165278));
					//printf("d = 1.453 m, calc d = %4.3f m\n", UtilCalcPositionDelta(57.6626295444,12.1056859139,57.6626273222, 12.1057100556));
					UserControlResetInputVariables();

				break;
				case tsp_1:

					if(CurrentCommandArgCount == 1)
					{
						if(!strcmp(pcBuffer,"-help"))
						{
							//printf("-----TEST SYNCHRONIZATION POINT FUNCTIONS-----\n");
							//printf("The function returns the time until reaching a specific position in the trajectory.\n");
							//printf("Arguments: 1\n");
							//printf("Syntax: tsp something\n");
							//printf("Ex: ftp traj/traj.txt 45.568 -80.445 0\n");
							//printf("Ex: tsp \n");
						}
					}

					if(CurrentCommandArgCount == CommandArgCount)
					{
						FILE *Trajfd;
	  					Trajfd = fopen ("traj/192.168.0.1", "r");
	    				OP.TrajectoryPositionCount = UtilCountFileRows(Trajfd) - 1;
	    				float SpaceArr[OP.TrajectoryPositionCount];
	    				float TimeArr[OP.TrajectoryPositionCount];
	    				SpaceTime SpaceTimeArr[OP.TrajectoryPositionCount];
	    				fclose(Trajfd);
	    				OP.SpaceArr = SpaceArr;
						OP.TimeArr = TimeArr;
						OP.SpaceTimeArr = SpaceTimeArr;
						//UtilCalcPositionDelta(57.7773716086,12.7804629583, 57.7773620, 12.7819188, &OP); //1
						UtilCalcPositionDelta(57.7773716086,12.7804629583,57.7776699,12.7808555, &OP); //2
						//UtilCalcPositionDelta(57.7773716086,12.7804629583,57.7776528,12.7812795, &OP); //3
						//printf("OrigoDistance = %4.3f\n", OP.OrigoDistance);
						if(OP.OrigoDistance > -1)
						{	
							UtilPopulateSpaceTimeArr(&OP, "traj/192.168.0.1");
							UtilSetSyncPoint(&OP, 60.039, -25.547, 0, 0);
							if (OP.SyncIndex > -1)
							{
								printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP.SpaceArr[OP.SyncIndex], OP.TimeArr[OP.SyncIndex], OP.SyncIndex);
								double CurrentTime = 15.1;				
								//UtilFindCurrentTrajectoryPosition(&OP, 0, 30.9, 0.2);  //1
								UtilFindCurrentTrajectoryPosition(&OP, 0, CurrentTime, 0.2);	//2
								//UtilFindCurrentTrajectoryPosition(&OP, 0, 26.6, 0.2);	//3
								if(OP.BestFoundTrajectoryIndex > -1 && OP.SyncIndex > -1)
								{	
								    printf("Current origo distance=%4.3f m\n", OP.OrigoDistance);
								    printf("Matched origo distance=%4.3f m\n", OP.SpaceArr[OP.BestFoundTrajectoryIndex]);
								    printf("Distance error=%4.3f m\n", OP.OrigoDistance - OP.SpaceArr[OP.BestFoundTrajectoryIndex]);
								    printf("Current time=%4.3f s\n", CurrentTime);
								    printf("Expected time=%4.3f s (index=%d)\n", OP.TimeArr[OP.BestFoundTrajectoryIndex], OP.BestFoundTrajectoryIndex);
								    printf("Time error=%4.3f s\n", CurrentTime - OP.TimeArr[OP.BestFoundTrajectoryIndex]);
									printf("Time to sync point = %4.3f s\n", fabs(UtilCalculateTimeToSync(&OP) - (CurrentTime - OP.TimeArr[OP.BestFoundTrajectoryIndex]))); 
									printf("x=%4.3f m\n", OP.x);
									printf("y=%4.3f m\n", OP.y);
								} else printf("Failed to find current position in trajectory\n");
							
							} else printf("Failed to find sync point!\n");
						} else printf("Distance calculation to origo failed to converge.\n");
						UserControlResetInputVariables();
					} else CurrentCommandArgCount ++;
				break;
				case tc_0:
					printf("Test to connect to server\n");
					int socketfd;
					char object_address_name[50];
					bzero(object_address_name,50);
					strcat(object_address_name, USER_CONTROL_SYSTEM_CONTROL_IP);
					uint32_t object_tcp_port = USER_CONTROL_SYSTEM_CONTROL_PORT;
					UserControlConnectServer(&socketfd, object_address_name, object_tcp_port);
					UserControlResetInputVariables();

				break;
				case ts_0:
					UserControlSendString("tsp", &socketfd);
					UserControlDisconnectObject(&socketfd);
					UserControlResetInputVariables();
				break;
				case cb_0:
					bzero(RecordBuffer, IPC_BUFFER_SIZE*2);
				break;
				case help_0:
					printf("cc - Echo Current Command.\n");
					printf("cp - Copy recorded buffer to send buffer.\n");
					printf("sb - Send content in send buffer.\n");
					printf("cx - Reset input commands.\n");
					printf("cb - Reset recorded buffer.\n");
					printf("status - Query server status.\n");
					printf("arm - Tell server to send \"ARM\".\n");
					printf("start - Tell server to send \"START\".\n");
					printf("stop - Tell server to send \"STOP\".\n");
					printf("abort - Tell server to send \"ABORT\".\n");
					printf("replay - Tell server to send \"REPLAY\". Write \"replay -help\" for more info\n");
					printf("exit - Tell server to send \"EXIT\" and quit this client.\n");
				break;
			
				default:

				break;

			}

	  }
  } else printf("Failed to start UserControl client!\n"); 
}

/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

UserControlCommand_t UserControlFindCommand(const char* CommandBuffer, UserControlCommand_t *CurrentCommand, int *CommandArgCount)
{

	UserControlCommand_t command; 	
	
	for (command = idle_0; command != nocommand; command++)
    {
		bzero(UserControlCommandArgCnt, USER_CONTROL_ARG_COUNT);    
        bzero(UserControlStrippedCommand, USER_CONTROL_COMMAND_MAX_LENGTH);
		strncpy(UserControlStrippedCommand, UserControlCommandsArr[(int)command], (uint64_t)strchr(UserControlCommandsArr[(int)command],'_') - (uint64_t)UserControlCommandsArr[(int)command] );
		strncpy(UserControlCommandArgCnt, strchr(UserControlCommandsArr[(int)command],'_')+1, strlen(UserControlCommandsArr[(int)command]) - ((uint64_t)strchr(UserControlCommandsArr[(int)command],'_') - (uint64_t)UserControlCommandsArr[(int)command] + 1));
		
		if (!strcmp(UserControlStrippedCommand, CommandBuffer))
        {
			if(command != cc_0)
			{
				*CommandArgCount = atoi(UserControlCommandArgCnt);				
				*CurrentCommand = command;            
				return command;
			} 
			else
			{
				PreviousUserControlCommand = *CurrentCommand;
				*CurrentCommand = command;
				return command;
			}
        }
    }
    return nocommand;
}

void UserControlResetInputVariables()
{
	SendBufferPtr = SendBuffer;
	bzero(SendBuffer,IPC_BUFFER_SIZE*2);
	UserControlCommand = idle_0;
	CurrentCommandArgCount = 0;
	CommandArgCount = 0;

}


static void UserControlConnectServer(int* sockfd, const char* name, const uint32_t port)
{
  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  char buffer[256];
  int iResult;

  *sockfd = socket(AF_INET, SOCK_STREAM, 0);
   
  if (*sockfd < 0) 
  {
    util_error("ERR: Failed to open control socket");
  }

  server = gethostbyname(name);
  if (server == NULL) 
  {
    util_error("ERR: Unknown host ");
  }
  
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  
  bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(port);
  
  //#ifdef DEBUG
    printf("Try to connect to control socket: %s %i\n",name,port);
    fflush(stdout);
  //#endif
  
  do
  {
    iResult = connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

    if ( iResult < 0) 
    {
      if(errno == ECONNREFUSED)
      {
        printf("WAR: Was not able to connect to object, retry in 3 sec...\n");
        fflush(stdout);
        (void)sleep(3);
      }
      else
      {
        util_error("ERR: Failed to connect to control socket");
      }
    }
  } while(iResult < 0);

  //#ifdef DEBUG
    printf("Connected to user control socket: %s %i\n",name,port);
    fflush(stdout);
  //#endif

}

static void UserControlSendString(const char* command, int* sockfd)
{
  int n;

  printf("Sending: %s\n",command);
  fflush(stdout);
  

  n = write(*sockfd, command, strlen(command));
  if (n < 0)
  {
    util_error("ERR: Failed to send on control socket");
  }
}


static void UserControlDisconnectObject(int* sockfd)
{
  close(*sockfd);
}