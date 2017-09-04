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
#include <time.h>

#include "objectcontrol.h"
#include "util.h"


#define IPC_BUFFER_SIZE   256
#define USER_CONTROL_SYSTEM_CONTROL_IP "127.0.0.1"  
#define USER_CONTROL_SYSTEM_CONTROL_PORT 54241

#define USER_CONTROL_ARG_COUNT 			2
#define USER_CONTROL_COMMAND_MAX_LENGTH 	10

#define COMMAND_MESSAGE_HEADER_LENGTH 5
#define COMMAND_DOPM_ROW_MESSAGE_LENGTH 25 
#define COMMAND_DOPM_ROWS_IN_TRANSMISSION  40



typedef enum {
	idle_0,		status_0,		arm_0,		start_1,		stop_0,		abort_0,		replay_1,		control_0,		exit_0,		cx_0,		cc_0,
	cp_0,		sb_0,			cb_0,		tp_0,			tsp_1,		sx_0,			sc_0,			help_0,			tosem_0,	tstrt_0,	tdopm_0,
	tmonr_0,	disarm_0,		tt_0,		nocommand
} UserControlCommand_t;
const char* UserControlCommandsArr[] = {
	"idle_0",	"status_0",		"arm_0",	"start_1",		"stop_0",	"abort_0",		"replay_1",		"control_0",	"exit_0",	"cx_0",		"cc_0",
	"cp_0", 	"sb_0", 		"cb_0", 	"tp_0", 		"tsp_1", 	"sx_0", 		"sc_0", 		"help_0", 		"tosem_0", 	"tstrt_0",	"tdopm_0",
	"tmonr_0", 	"disarm_0",		"tt_0"};
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

/*------------------------------------------------------------
-- The main function.
------------------------------------------------------------*/
int main(int argc, char *argv[])
{

	char pcBuffer[IPC_BUFFER_SIZE];
	char inchr;
	struct timeval tvTime;
	struct timespec tTime;

	int iExit = 0;
	ObjectPosition OP;
	SendBufferPtr = SendBuffer;
	bzero(SendBuffer,IPC_BUFFER_SIZE*2);

	char Id[2];
	char Timestamp[20];
	char Latitude[20];
	char Longitude[20];
	char Altitude[20];
	char Heading[20];
	char Speed[20];
	char DriveDirection[2];
	char StatusFlag[2];
	char Buffer[100];

	unsigned char TestBuffer[] = {7,200,65,36,135,231,3,111,230,191,0,190,10,254,0,0,86,195,44,236,1,104,0,6};

	char MessageBuffer[100];
	int MessageLength;
	int RowCount, i, Rest;
	char TrajBuffer[COMMAND_DOPM_ROWS_IN_TRANSMISSION*COMMAND_DOPM_ROW_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH];
	FILE *fd;

	int socketfd=-1;
	char object_address_name[50];
	bzero(object_address_name,50);
	uint32_t object_tcp_port = USER_CONTROL_SYSTEM_CONTROL_PORT;

	if (argc > 1)
	{
		strcat(object_address_name, argv[1]);
  	}
	else
	{
		strcat(object_address_name, USER_CONTROL_SYSTEM_CONTROL_IP);
	}

	if (argc > 2)
	{
      object_tcp_port = atoi(argv[2]);
  	}

  	printf("Connecting to server... %s %d \n",object_address_name,object_tcp_port);


	UserControlConnectServer(&socketfd, object_address_name, object_tcp_port);
	if (socketfd >= 0) 
  	{
  		printf("Client is connected.\n"); 
		while(!iExit)
		{
			
			bzero(pcBuffer,IPC_BUFFER_SIZE);
			scanf("%49s",pcBuffer);
		
			if(	strcmp(pcBuffer, "cc") != 0 && 
				strcmp(pcBuffer, "cx") != 0 &&
				strcmp(pcBuffer, "cp") != 0 &&
				strcmp(pcBuffer, "sb") != 0 &&
				strcmp(pcBuffer, "help") != 0 &&
				strcmp(pcBuffer, "cb") != 0 &&
				strcmp(pcBuffer, "sx") != 0 &&
				strcmp(pcBuffer, "sc") != 0)
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
				case disarm_0:
					UserControlSendString(SendBuffer, &socketfd);
					UserControlResetInputVariables();
				break;
				case start_1:
					if(CurrentCommandArgCount == CommandArgCount)
					{
						UserControlSendString(SendBuffer, &socketfd);
						UserControlResetInputVariables();
					} else CurrentCommandArgCount ++;
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
				case tt_0:
					gettimeofday(&tvTime, NULL);
					uint64_t uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
					printf("Time: %lx\n",uiTime );
					printf("Time: %ld\n",uiTime );

					clock_gettime(CLOCK_MONOTONIC_RAW, &tTime);
					uiTime = (uint64_t)tTime.tv_sec*1000L + (uint64_t)tTime.tv_nsec/1000000L - (uint64_t)MS_FROM_1970_TO_2004_NO_LEAP_SECS + (uint64_t)DIFF_LEAP_SECONDS_UTC_ETSI*1000;
					printf("Time: %lx\n",uiTime );
					printf("Time: %ld\n",uiTime );

					UserControlResetInputVariables();
				break;
				case tp_0:
					//UserControlCommand = idle_0;
	//				UtilCalcPositionDelta(57.6626302333,12.1056869167,57.6626269972, 12.1057250694, &OP);
					//‭429.4967295‬
					//57,7773716
					UtilCalcPositionDelta(57.7773716086, 12.7804629583 ,57.7773716000, 12.7804629000, &OP);
					//UtilCalcPositionDelta(57.7773716086, 12.7804629583 ,57.7782802000, 12.7807861000, &OP);
					//UtilCalcPositionDelta(57.7773298066,12.7818834416,57.777329775, 12.7818832583, &OP);
					//UtilCalcPositionDelta(57.7771230833333,12.78156473, 57.777711,12.780829, &OP);
					//UtilCalcPositionDelta(57.777360,12.780472, 57.777711,12.780829, &OP);

	//				UtilCalcPositionDelta(57.7773716086,12.7804629583,57.7773717086, 12.7804630583, &OP);
					printf("(57.7773716086, 12.7804629583 ,57.7773716000, 12.7804629000) => Calc d = %4.4f m, iterations = %d\n", OP.OrigoDistance, OP.CalcIterations);
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
	  					Trajfd = fopen ("traj/192.168.0.16", "r");
	    				OP.TrajectoryPositionCount = UtilCountFileRows(Trajfd) - 2;
	    				float SpaceArr[OP.TrajectoryPositionCount];
	    				float TimeArr[OP.TrajectoryPositionCount];
	    				SpaceTime SpaceTimeArr[OP.TrajectoryPositionCount];
	    				fclose(Trajfd);
	    				OP.SpaceArr = SpaceArr;
						OP.TimeArr = TimeArr;
						OP.SpaceTimeArr = SpaceTimeArr;
						double CurrentTime = 19.472;
						UtilCalcPositionDelta(57.7773716086, 12.7804629583 ,96.3421249000, 12.7816472000, &OP); //2
						printf("Calc d = %4.4f m, iterations = %d\n", OP.OrigoDistance, OP.CalcIterations);
						printf("Latitude = %3.10f \n", OP.Latitude);
						printf("Longitude = %3.10f\n", OP.Longitude);
						printf("ForwardAzimuth1 = %3.10f \n", OP.ForwardAzimuth1);
						printf("ForwardAzimuth2 = %3.10f\n", OP.ForwardAzimuth2);
						printf("DeltaForwardAzimuth = %3.15f \n", OP.ForwardAzimuth1-OP.ForwardAzimuth2);
						printf("x = %4.15lf\n", OP.x);
						printf("y = %4.15lf\n", OP.y);

						if(OP.OrigoDistance > -1)
						{	
							UtilPopulateSpaceTimeArr(&OP, "traj/192.168.0.16");
							UtilSetSyncPoint(&OP, 0, 0, 0, 27.0);
							if (OP.SyncIndex > -1)
							{
								printf("Sync point found=%4.3f, Time=%4.3f, Index=%d\n", OP.SpaceArr[OP.SyncIndex], OP.TimeArr[OP.SyncIndex], OP.SyncIndex);
																
								//1.647, 57.7773716086, 12.7804629583 ,57.7776690000, 12.7809532000
								//19.472, 57.7773716086, 12.7804629583 ,96.3421249000, 12.7816472000


								UtilFindCurrentTrajectoryPosition(&OP, 0, CurrentTime, 0.75, 2.5, 1);	//2
								if(OP.BestFoundTrajectoryIndex > -1 && OP.SyncIndex > -1)
								{	
								    printf("\nCurrent origo distance=%4.3f m\n", OP.OrigoDistance);
								    printf("Matched origo distance=%4.3f m\n", OP.SpaceArr[OP.BestFoundTrajectoryIndex]);
								    printf("Distance error=%4.3f m\n", OP.OrigoDistance - OP.SpaceArr[OP.BestFoundTrajectoryIndex]);
								    printf("Current time=%4.3f s\n", CurrentTime);
								    printf("Expected time=%4.3f s (index=%d)\n", OP.TimeArr[OP.BestFoundTrajectoryIndex], OP.BestFoundTrajectoryIndex);
								    printf("Time error=%4.3f s\n", CurrentTime - OP.TimeArr[OP.BestFoundTrajectoryIndex]);
									//printf("Time to sync point = %4.3f s\n", fabs(UtilCalculateTimeToSync(&OP) - (CurrentTime - OP.TimeArr[OP.BestFoundTrajectoryIndex]))); 
									printf("Time to sync point = %4.3f\n", UtilCalculateTimeToSync(&OP)); 
									printf("x=%4.3f m\n", OP.x);
									printf("y=%4.3f m\n", OP.y);
								} 
				 				else if(OP.BestFoundTrajectoryIndex == -1) 
				                {
				                  printf("No trajectory position found.\n");
				                }
				                else if(OP.BestFoundTrajectoryIndex == -2)
				                {
				                  printf("Master not in time\n");
				                }

							} else printf("Failed to find sync point!\n");
						} else printf("Distance calculation to origo failed to converge.\n");
						UserControlResetInputVariables();
					} else CurrentCommandArgCount ++;
				break;
				case sc_0:
					UserControlConnectServer(&socketfd, object_address_name, object_tcp_port);
					UserControlResetInputVariables();
				break;
				case sx_0:
					UserControlDisconnectObject(&socketfd);
					close(socketfd);
					socketfd = -1;
					UserControlResetInputVariables();
				break;
				case tosem_0:
					 MessageLength = ObjectControlBuildOSEMMessage(MessageBuffer, 
                                UtilSearchTextFile("conf/test.conf", "OrigoLatidude=", "", Latitude),
                                UtilSearchTextFile("conf/test.conf", "OrigoLongitude=", "", Longitude),
                                UtilSearchTextFile("conf/test.conf", "OrigoAltitude=", "", Altitude),
                                UtilSearchTextFile("conf/test.conf", "OrigoHeading=", "", Heading),
                                1); 

					UserControlResetInputVariables();
				break;
				case tstrt_0:
					MessageLength = ObjectControlBuildSTRTMessage(MessageBuffer, 1, 1024, 1);
					UserControlResetInputVariables();
				break;
				case tdopm_0:
					fd = fopen ("traj/192.168.0.1", "r");
					RowCount = UtilCountFileRows(fd) - 1;
					fclose (fd);

					MessageLength = ObjectControlBuildDOPMMessageHeader(TrajBuffer, RowCount-1, 1);
					//MessageLength = ObjectControlBuildDOPMMessageHeader(TrajBuffer, 2, 1);
					/*Send DOPM header*/
					
					fd = fopen ("traj/192.168.0.1", "r");
					UtilReadLineCntSpecChars(fd, TrajBuffer);//Read first line
					Rest = 0, i = 0;
					do
					{
						Rest = RowCount - COMMAND_DOPM_ROWS_IN_TRANSMISSION;
						RowCount = RowCount - COMMAND_DOPM_ROWS_IN_TRANSMISSION; 
						if(Rest >= COMMAND_DOPM_ROWS_IN_TRANSMISSION)
						{
							MessageLength = ObjectControlBuildDOPMMessage(TrajBuffer, fd, COMMAND_DOPM_ROWS_IN_TRANSMISSION, 0);
							//MessageLength = ObjectControlBuildDOPMMessage(TrajBuffer, fd, 2, 1);
						}
						else
						{
						  MessageLength = ObjectControlBuildDOPMMessage(TrajBuffer, fd, Rest, 0);
						}
						printf("Transmission %d: %d bytes left to send.\n", ++i, Rest*COMMAND_DOPM_ROW_MESSAGE_LENGTH);

						/*Send DOPM data*/
						
					} while (Rest >= COMMAND_DOPM_ROWS_IN_TRANSMISSION /*i < 2*/); 

					fclose (fd);
					UserControlResetInputVariables();
				break;
				case tmonr_0:
					ObjectControlMONRToASCII(TestBuffer, 1, Id, Timestamp, Latitude, Longitude, Altitude, Speed, Heading, DriveDirection, StatusFlag, 1);
					bzero(Buffer,100);
					strcat(Buffer,Timestamp);
					strcat(Buffer,";");
					strcat(Buffer,Latitude);
					strcat(Buffer,";");
					strcat(Buffer,Longitude);
					strcat(Buffer,";");
					strcat(Buffer,Altitude);
					strcat(Buffer,";");
					strcat(Buffer,Speed);
					strcat(Buffer,";");
					strcat(Buffer,Heading);
					strcat(Buffer,";");
					strcat(Buffer,DriveDirection);
					strcat(Buffer,";");
					//strcat(Buffer,StatusFlag);
					//strcat(Buffer,";");
					printf("MONR message: %s\n", Buffer);
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
					printf("disarm - Tell server to send \"DISARM\".\n");
					printf("start - Tell server to send \"START\".\n");
					printf("stop - Tell server to send \"STOP\".\n");
					printf("abort - Tell server to send \"ABORT\".\n");
					printf("replay - Tell server to send \"REPLAY\". Write \"replay -help\" for more info.\n");
					printf("exit - Tell server to send \"EXIT\" and quit this client.\n");
					printf("sx - Close connection to server.\n");
					printf("sc - Open connection to server.\n");
					printf("tosem - Test to build OSEM message.\n");
					printf("tstrt - Test to build STRT message.\n");
					printf("tdopm - Test to build DOPM message.\n");
					printf("tmonr - Test to build MONR message.\n");
				break;
			
				default:

				break;
			}
			usleep(1);

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
  
  #ifdef DEBUG
    printf("Try to connect to control socket: %s %i\n",name,port);
    fflush(stdout);
  #endif
  
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
    printf("Client connected to server: %s %i\n",name,port);
    fflush(stdout);
  //#endif

}

static void UserControlSendString(const char *command, int *sockfd)
{
  int n;

  printf("Sending: %s\n",command);
  fflush(stdout);
  

  n = write(*sockfd, command, strlen(command));
  if (n < 0)
  {
    util_error("[UserControl] ERR: Failed to send on control socket");
  }
}


static void UserControlDisconnectObject(int *sockfd)
{
  close(*sockfd);
}
