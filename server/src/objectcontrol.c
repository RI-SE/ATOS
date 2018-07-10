/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.c
  -- Author      : Sebastian Loh Lindholm
  -- Description : CHRONOS
  -- Purpose     :
  -- Reference   :
  ------------------------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Include files.
  ------------------------------------------------------------*/
#include "objectcontrol.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <mqueue.h>
#include <netdb.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include "util.h"
 

/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/

#define LOCALHOST "127.0.0.1"

#define RECV_MESSAGE_BUFFER 1024
#define OBJECT_MESS_BUFFER_SIZE 1024

#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define OBJECT_CONTROL_CONTROL_MODE 0
#define OBJECT_CONTROL_REPLAY_MODE 1
#define OBJECT_CONTROL_ABORT_MODE 1

#define COMMAND_MESSAGE_HEADER_LENGTH sizeof(HeaderType) 
#define COMMAND_MESSAGE_FOOTER_LENGTH sizeof(FooterType)
#define COMMAND_CODE_INDEX 0
#define COMMAND_MESSAGE_LENGTH_INDEX 1

#define COMMAND_DOPM_CODE 1
#define COMMAND_DOPM_ROW_MESSAGE_LENGTH 25 
#define COMMAND_DOPM_ROWS_IN_TRANSMISSION  40

#define COMMAND_OSEM_CODE 2
#define COMMAND_OSEM_NOFV 3
#define COMMAND_OSEM_MESSAGE_LENGTH sizeof(OSEMType)-4

#define COMMAND_OSTM_CODE 3
#define COMMAND_OSTM_NOFV 1  
#define COMMAND_OSTM_MESSAGE_LENGTH sizeof(OSTMType)
#define COMMAND_OSTM_OPT_SET_ARMED_STATE 2
#define COMMAND_OSTM_OPT_SET_DISARMED_STATE 3 

#define COMMAND_STRT_CODE  4
#define COMMAND_STRT_NOFV 1
#define COMMAND_STRT_MESSAGE_LENGTH sizeof(STRTType)-2
#define COMMAND_STRT_OPT_START_IMMEDIATELY 1
#define COMMAND_STRT_OPT_START_AT_TIMESTAMP 2  

#define COMMAND_HEAB_CODE 5
#define COMMAND_HEAB_NOFV 2
#define COMMAND_HEAB_MESSAGE_LENGTH sizeof(HEABType)
#define COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING 0
#define COMMAND_HEAB_OPT_SERVER_STATUS_OK 1
#define COMMAND_HEAB_OPT_SERVER_STATUS_ABORT 2 

#define COMMAND_MONR_CODE 6
#define COMMAND_MONR_NOFV 12
#define COMMAND_MONR_MESSAGE_LENGTH sizeof(MONRType)-2


#define COMMAND_LLCM_CODE 8
#define COMMAND_LLCM_MESSAGE_LENGTH 5

#define COMMAND_SYPM_CODE 9
#define COMMAND_SYPM_MESSAGE_LENGTH 8

#define COMMAND_MTPS_CODE 10
#define COMMAND_MTPS_MESSAGE_LENGTH 6

#define COMMAND_ACM_CODE 11  //Action Configuration Message: Server->Object, TCP
#define COMMAND_ACM_MESSAGE_LENGTH 6 
 
#define COMMAND_EAM_CODE 12  //Execution Action Message: Server->Object, UDP
#define COMMAND_EAM_MESSAGE_LENGTH 6 

#define COMMAND_TCM_CODE 13  //Trigger Configuration Message: Server->Object, TCP 
#define COMMAND_TCM_MESSAGE_LENGTH 5 

#define COMMAND_TOM_CODE 14  //Trigger Occurred Message: Object->Server, UDP
#define COMMAND_TOM_MESSAGE_LENGTH 8 


#define CONF_FILE_PATH  "conf/test.conf"

#define SMALL_BUFFER_SIZE_0 20
#define SMALL_BUFFER_SIZE_1 2
#define SMALL_BUFFER_SIZE_2 1

#define TRAJECTORY_FILE_MAX_ROWS  4096

#define LOG_BUFFER_LENGTH 128

typedef enum {
  COMMAND_HEARTBEAT_GO,
  COMMAND_HEARTBEAT_ABORT
} hearbeatCommand_t;


typedef enum {
  OBC_STATE_UNDEFINED,
  OBC_STATE_IDLE,
  OBC_STATE_INITIALIZED,
  OBC_STATE_CONNECTED,
  OBC_STATE_ARMED,
  OBC_STATE_RUNNING,
  OBC_STATE_ERROR,
} OBCState_t;

char TrajBuffer[COMMAND_DOPM_ROWS_IN_TRANSMISSION*COMMAND_DOPM_ROW_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH];


/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static I32 vConnectObject(int* sockfd,const char* name,const uint32_t port, U8 *Disconnect);
static void vSendString(const char* command,int* sockfd);
static void vSendBytes(const char* command, int length, int* sockfd, int debug);
static void vSendFile(const char* object_traj_file, int* sockfd);
static void vDisconnectObject(int* sockfd);

static void vCreateSafetyChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static void vCloseSafetyChannel(int* sockfd);
static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand);
static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData);
int ObjectControlBuildOSEMMessage(char* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, char *Latitude, char *Longitude, char *Altitude, char *Heading, char debug);
int ObjectControlBuildSTRTMessage(char* MessageBuffer, STRTType *STRTData, unsigned char CommandOption, unsigned long TimeStamp, char debug);
int ObjectControlBuildOSTMMessage(char* MessageBuffer, OSTMType *OSTMData, unsigned char CommandOption, char debug);
int ObjectControlBuildHEABMessage(char* MessageBuffer, HEABType *HEABData, unsigned long TimeStamp, unsigned char CommandOption, char debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
int ObjectControlBuildSYPMMessage(char* MessageBuffer, unsigned int SyncPoint, unsigned int StopTime, char debug);
int ObjectControlBuildMTSPMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug);
int ObjectControlBuildDOPMMessageHeader(char* MessageBuffer, int RowCount, char debug);
int ObjectControlBuildDOPMMessage(char* MessageBuffer, FILE *fd, int RowCount, char debug);
int ObjectControlSendDOPMMEssage(char* Filename, int *Socket, int RowCount, char *IP, uint32_t Port,char debug);
int ObjectControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);
int ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed, char *LateralSpeed, char *LongitudinalAcc, char *LateralAcc, char *Heading, char *DriveDirection, char *StatusFlag, char *StateFlag, char debug);
int ObjectControlBuildMONRMessage(unsigned char *MonrData, MONRType *MONRData, char debug);
int ObjectControlTOMToASCII(unsigned char *TomData, char *TriggId ,char *TriggAction, char *TriggDelay, char debug);
int ObjectControlBuildTCMMessage(char* MessageBuffer, TriggActionType *TAA, char debug);
static void ObjectControlSendLog(C8 *Message);

static void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  int* nbr_objects);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void objectcontrol_task(TimeType *GPSTime)
{
  int safety_socket_fd[MAX_OBJECTS];
  struct sockaddr_in safety_object_addr[MAX_OBJECTS];
  int socket_fd[MAX_OBJECTS];
  char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
  uint32_t object_udp_port[MAX_OBJECTS];
  uint32_t object_tcp_port[MAX_OBJECTS];
  int nbr_objects=0;
  int iExit = 0;
  int iCommand;
  char pcRecvBuffer[RECV_MESSAGE_BUFFER];
  char pcBuffer[OBJECT_MESS_BUFFER_SIZE];
  char pcTempBuffer[512];
  unsigned char MessageBuffer[512];
  int iIndex = 0, i=0;
  monitor_t ldm[MAX_OBJECTS];
  struct timespec sleep_time, ref_time;
  int iForceObjectToLocalhost = 0;
  
  FILE *fd;
  char Id[SMALL_BUFFER_SIZE_0];
  char Timestamp[SMALL_BUFFER_SIZE_0];
  char Latitude[SMALL_BUFFER_SIZE_0];
  char Longitude[SMALL_BUFFER_SIZE_0];
  char Altitude[SMALL_BUFFER_SIZE_0];
  char Speed[SMALL_BUFFER_SIZE_0];
  char LongitudinalSpeed[SMALL_BUFFER_SIZE_0];
  char LateralSpeed[SMALL_BUFFER_SIZE_0];
  char LongitudinalAcc[SMALL_BUFFER_SIZE_0];
  char LateralAcc[SMALL_BUFFER_SIZE_0];  
  char Heading[SMALL_BUFFER_SIZE_0];
  char DriveDirection[SMALL_BUFFER_SIZE_1];
  char StatusFlag[SMALL_BUFFER_SIZE_1];
  char StateFlag[SMALL_BUFFER_SIZE_1];
  char MTSP[SMALL_BUFFER_SIZE_0];
  int MessageLength;
  char *MiscPtr;
  uint64_t StartTimeU64 = 0;
  uint64_t CurrentTimeU64 = 0;
  uint64_t OldTimeU64 = 0;
  uint64_t MasterTimeToSyncPointU64 = 0;
  uint64_t TimeCap1, TimeCap2;
  double TimeToSyncPoint = 0;
  double PrevTimeToSyncPoint = 0;
  double CurrentTimeDbl = 0;
  struct timeval CurrentTimeStruct;
  int HeartbeatMessageCounter = 0;

  ObjectPosition OP[MAX_OBJECTS];
  float SpaceArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
  float TimeArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
  SpaceTime SpaceTimeArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
  char OriginLatitude[SMALL_BUFFER_SIZE_0];
  char OriginLongitude[SMALL_BUFFER_SIZE_0];
  char OriginAltitude[SMALL_BUFFER_SIZE_0];
  char OriginHeading[SMALL_BUFFER_SIZE_0];
  char TextBuffer[SMALL_BUFFER_SIZE_0];
  double OriginLatitudeDbl;
  double OriginLongitudeDbl;
  double OriginAltitudeDbl;
  double OriginHeadingDbl;
  AdaptiveSyncPoint ASP[MAX_ADAPTIVE_SYNC_POINTS];
  int SyncPointCount = 0;
  int SearchStartIndex = 0;
  double TimeDiff = 0;
  double DistTraveled = 0;
  double ASPMaxTimeDiff = 0;
  double ASPMaxTrajDiff = 0;
  double ASPFilterLevel = 0;
  double TimeQuota = 0;
  double ASPMaxDeltaTime = 0;
  int ASPDebugRate = 1;
  int ASPStepBackCount = 0;
  TriggActionType TAA[MAX_TRIGG_ACTIONS];
  int TriggerActionCount = 0;
  double DeltaTime = 0;
  struct timeval tvTime;
  char pcSendBuffer[MQ_MAX_MESSAGE_LENGTH];
  char ObjectIP[SMALL_BUFFER_SIZE_0];
  char ObjectPort[SMALL_BUFFER_SIZE_0];
  char TriggId[SMALL_BUFFER_SIZE_1];
  char TriggAction[SMALL_BUFFER_SIZE_1];
  char TriggDelay[SMALL_BUFFER_SIZE_0];
  HeaderType HeaderData;
  OSEMType OSEMData;
  STRTType STRTData;
  OSTMType OSTMData;
  HEABType HEABData;
  MONRType MONRData;
  GeoPosition OriginPosition;

  unsigned char ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING;

  OBCState_t OBCState = OBC_STATE_IDLE;

  U8 uiTimeCycle = 0;
  int ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

  C8 Buffer2[SMALL_BUFFER_SIZE_1];
  C8 LogBuffer[LOG_BUFFER_LENGTH];

  U8 DisconnectU8 = 0;
  I32 iResult;
  (void)iCommInit(IPC_RECV_SEND,MQ_OC,1);

  while(!iExit)
  {
    
    if(OBCState == OBC_STATE_RUNNING || OBCState == OBC_STATE_ARMED || OBCState == OBC_STATE_CONNECTED)
    {
     /*HEAB*/
      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
        if(uiTimeCycle == 0)
        {
          HeartbeatMessageCounter ++;
          MessageLength = ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, CurrentTimeU64, ObjectControlServerStatus, 0);
          ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
        }
      }
    }

    if(OBCState == OBC_STATE_RUNNING)
    {
      char buffer[RECV_MESSAGE_BUFFER];
      int recievedNewData = 0;

      gettimeofday(&CurrentTimeStruct, NULL);
      CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
      if(TIME_COMPENSATE_LAGING_VM) CurrentTimeU64 = CurrentTimeU64 - TIME_COMPENSATE_LAGING_VM_VAL;

      /*HEAB*/
      /*
      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
        if(uiTimeCycle == 0)
        {
          HeartbeatMessageCounter ++;
          MessageLength = ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, CurrentTimeU64, ObjectControlServerStatus, 0);
          ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
        }
      }
      */

      /*MTSP*/
      if(HeartbeatMessageCounter == 10)
      {
        HeartbeatMessageCounter = 0;
        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
              for(i = 0; i < SyncPointCount; i++)
              {
                if(TEST_SYNC_POINTS == 1 && iIndex == 1 && MasterTimeToSyncPointU64 > 0 && TimeToSyncPoint > -1 )
                {
                    /*Send Master time to adaptive sync point*/
                    MessageLength =ObjectControlBuildMTSPMessage(MessageBuffer, MasterTimeToSyncPointU64, 0);
                    ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                }
                else if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL && MasterTimeToSyncPointU64 > 0 && TimeToSyncPoint > -1)
                {
                    /*Send Master time to adaptive sync point*/
                    MessageLength =ObjectControlBuildMTSPMessage(MessageBuffer, MasterTimeToSyncPointU64, 0);
                    ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                }
              }
        }
      }


      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
        bzero(buffer,RECV_MESSAGE_BUFFER);
        vRecvMonitor(&safety_socket_fd[iIndex],buffer, RECV_MESSAGE_BUFFER, &recievedNewData);

        #ifdef DEBUG
          printf("INF: Did we recieve new data from %s %d: %d\n",object_address_name[iIndex],object_udp_port[iIndex],recievedNewData);
          fflush(stdout);
        #endif

        if(recievedNewData)
        {

          #ifdef DEBUG
            printf("INF: Did we recieve new data from %s %d %d: %s \n",object_address_name[iIndex],object_udp_port[iIndex],recievedNewData,buffer);
            fflush(stdout);
        	#endif
          
            if(buffer[0] == COMMAND_TOM_CODE)
            {
              for(i = 0; i < TriggerActionCount; i ++)
              {
                sprintf(LogBuffer, "[ObjectControl] External trigg received. %s\n", TAA[i].TriggerIP); ObjectControlSendLog(LogBuffer);
                if(strstr(TAA[i].TriggerIP, object_address_name[iIndex]) != NULL)
                {
                  //printf("[ObjectControl] External trigg received\n");
                  fflush(stdout);            
                  ObjectControlTOMToASCII(buffer, TriggId, TriggAction, TriggDelay, 1);
                  bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
                  bzero(pcSendBuffer,MQ_MAX_MESSAGE_LENGTH);
                  bzero(ObjectPort, SMALL_BUFFER_SIZE_0);
                  sprintf(ObjectPort, "%d", object_udp_port[iIndex]);
                  strcat(pcSendBuffer,object_address_name[iIndex]);strcat(pcSendBuffer,";");
                  strcat(pcSendBuffer, ObjectPort);strcat(pcSendBuffer,";");
                  strcat(pcSendBuffer,TriggId);strcat(pcSendBuffer,";");
                  strcat(pcSendBuffer,TriggAction);strcat(pcSendBuffer,";");
                  strcat(pcSendBuffer,TriggDelay);strcat(pcSendBuffer,";");
                  (void)iCommSend(COMM_TOM, pcSendBuffer);
                }
              }
            }

            ObjectControlBuildMONRMessage(buffer, &MONRData, 0);
            ObjectControlMONRToASCII(&MONRData, &OriginPosition, iIndex, Id, Timestamp, Latitude, Longitude, Altitude, LongitudinalSpeed, LateralSpeed, LongitudinalAcc, LateralAcc, Heading, DriveDirection, StatusFlag, StateFlag, 1);
            bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
            strcat(buffer,object_address_name[iIndex]); strcat(buffer,";");
            strcat(buffer, "0"); strcat(buffer,";");
            strcat(buffer,Timestamp); strcat(buffer,";");
            strcat(buffer,Latitude); strcat(buffer,";");
            strcat(buffer,Longitude); strcat(buffer,";");
            strcat(buffer,Altitude); strcat(buffer,";");
            strcat(buffer,Heading); strcat(buffer,";");
            strcat(buffer,LongitudinalSpeed); strcat(buffer,";");
            strcat(buffer,LateralSpeed); strcat(buffer,";");
            strcat(buffer,LongitudinalAcc); strcat(buffer,";");
            strcat(buffer,LateralAcc); strcat(buffer,";");
            strcat(buffer,DriveDirection); strcat(buffer,";"); strcat(buffer,StatusFlag); strcat(buffer,";");
            strcat(buffer,StateFlag); strcat(buffer,";");
            strcat(buffer,StatusFlag); strcat(buffer,";");

            //printf("<%s>\n",buffer);

            
            for(i = 0; i < SyncPointCount; i++)
            {
              if( TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL && CurrentTimeU64 > StartTimeU64 && StartTimeU64 > 0 && TimeToSyncPoint > -1 ||
                  TEST_SYNC_POINTS == 1 && ASP[0].TestPort == object_udp_port[iIndex] && StartTimeU64 > 0 && iIndex == 0 && TimeToSyncPoint > -1)
              {

                if(Latitude != 0 && Longitude != 0)
                { 
                  gettimeofday(&CurrentTimeStruct, NULL);
                  TimeCap1 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;

                  UtilCalcPositionDelta(OriginLatitudeDbl,OriginLongitudeDbl,atof(Latitude)/1e7,atof(Longitude)/1e7, &OP[iIndex]);
               
                  if(OP[iIndex].BestFoundTrajectoryIndex <= OP[iIndex].SyncIndex)
                  {
                    CurrentTimeDbl = (((double)CurrentTimeU64-(double)StartTimeU64)/1000);
                    SearchStartIndex = OP[iIndex].BestFoundTrajectoryIndex - ASPStepBackCount;
                    UtilFindCurrentTrajectoryPosition(&OP[iIndex], SearchStartIndex, CurrentTimeDbl, ASPMaxTrajDiff, ASPMaxTimeDiff, 1);
                    
                  	if(OP[iIndex].BestFoundTrajectoryIndex != TRAJ_POSITION_NOT_FOUND)
                    {
                      TimeToSyncPoint = UtilCalculateTimeToSync(&OP[iIndex]);
                    	if(TimeToSyncPoint > 0)
                    	{
                        if(PrevTimeToSyncPoint != 0 && ASPFilterLevel > 0)
                        {
                          if(TimeToSyncPoint/PrevTimeToSyncPoint > (1 + ASPFilterLevel/100)) TimeToSyncPoint = PrevTimeToSyncPoint + ASPMaxDeltaTime;//TimeToSyncPoint*ASPFilterLevel/500;
                          else if(TimeToSyncPoint/PrevTimeToSyncPoint < (1 - ASPFilterLevel/100)) TimeToSyncPoint = PrevTimeToSyncPoint - ASPMaxDeltaTime;//TimeToSyncPoint*ASPFilterLevel/500;
                        }
                    		MasterTimeToSyncPointU64 = CurrentTimeU64 + TimeToSyncPoint*1000;
                        PrevTimeToSyncPoint = TimeToSyncPoint;
                        OldTimeU64 = CurrentTimeU64;
                    	}
                    	else
                    	{
                    		MasterTimeToSyncPointU64 = 0;
                    		TimeToSyncPoint = -1;
                    	}
                      
                      bzero(MTSP, SMALL_BUFFER_SIZE_0);
                      sprintf(MTSP, "%" PRIu64, MasterTimeToSyncPointU64);
                      strcat(buffer, MTSP); strcat(buffer,";");                 
                    }

                    gettimeofday(&CurrentTimeStruct, NULL);
                    TimeCap2 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;

                  	if(atoi(Timestamp)%ASPDebugRate == 0)
                  	{
                  		printf("TtS= %3.3f, %d, %d, %d, %ld, %d, %3.0f\n",TimeToSyncPoint, OP[iIndex].BestFoundTrajectoryIndex, OP[iIndex].SyncIndex, SearchStartIndex, MasterTimeToSyncPointU64, iIndex, ((double)(TimeCap2)-(double)TimeCap1));
                      printf("%3.3f, %3.7f, %3.7f ,%3.7f, %3.7f\n\n",CurrentTimeDbl, OriginLatitudeDbl,OriginLongitudeDbl, atof(Latitude)/1e7, atof(Longitude)/1e7);  
                  	}
                  }
                }
              }
            }

          OP[iIndex].Speed = atof(Speed);

          #ifdef DEBUG
            printf("INF: Send MONITOR message: %s\n",buffer);
            fflush(stdout);
          #endif

          if(ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) (void)iCommSend(COMM_MONI,buffer);
        }
      }
    }

    bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);
    //Have we recieved a command?
    if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER))
    {

      #ifdef DEBUG
        printf("INF: Object control command %d\n",iCommand);
        fflush(stdout);
      #endif

  		if(iCommand == COMM_ARMD && OBCState == OBC_STATE_CONNECTED)
     	{
        if(pcRecvBuffer[0] == COMMAND_OSTM_OPT_SET_ARMED_STATE)
        {
          sprintf(LogBuffer, "[ObjectControl] Sending ARM: %d\n", pcRecvBuffer[0]); ObjectControlSendLog(LogBuffer);
          OBCState = OBC_STATE_ARMED;
        }
  			else if(pcRecvBuffer[0] == COMMAND_OSTM_OPT_SET_DISARMED_STATE)
        {
          sprintf(LogBuffer, "[ObjectControl] Sending DISARM: %d\n", pcRecvBuffer[0]); ObjectControlSendLog(LogBuffer);
          OBCState = OBC_STATE_CONNECTED;
        }
  			MessageLength = ObjectControlBuildOSTMMessage(MessageBuffer, &OSTMData, pcRecvBuffer[0], 0);

  			for(iIndex=0;iIndex<nbr_objects;++iIndex)
  			{
  		    /*Send AROM message*/
  		    vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
  			}
     	}
    	else if(iCommand == COMM_STRT && OBCState == OBC_STATE_ARMED)
  		{
        
        sprintf(LogBuffer, "[ObjectControl] START received <%s>\n",pcRecvBuffer); ObjectControlSendLog(LogBuffer);
  			bzero(Timestamp, SMALL_BUFFER_SIZE_0);
  			MiscPtr =strchr(pcRecvBuffer,';');
  			strncpy(Timestamp, MiscPtr+1, (uint64_t)strchr(MiscPtr+1, ';') - (uint64_t)MiscPtr  - 1);
  			StartTimeU64 = atol(Timestamp);
  			MasterTimeToSyncPointU64 = 0;
  			TimeToSyncPoint = 0;
  			SearchStartIndex = -1;
  			PrevTimeToSyncPoint = 0;
        OldTimeU64 = CurrentTimeU64;
  			ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;

  			#ifdef DEBUG
  				printf("INF: Sending START trig from object control <%s>\n",pcBuffer);
  				fflush(stdout);
  			#endif
        MessageLength = ObjectControlBuildSTRTMessage(MessageBuffer, &STRTData, COMMAND_STRT_OPT_START_AT_TIMESTAMP, StartTimeU64, 0);
  			for(iIndex=0;iIndex<nbr_objects;++iIndex) { vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);}
        OBCState = OBC_STATE_RUNNING;
  		}
  		else if(iCommand == COMM_REPLAY)
  		{
  			ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
  			printf("[ObjectControl] Object control REPLAY mode <%s>\n", pcRecvBuffer);
  			fflush(stdout);
  		}
  		else if(iCommand == COMM_ABORT && (OBCState == OBC_STATE_CONNECTED || OBCState == OBC_STATE_ARMED || OBCState == OBC_STATE_RUNNING))
  		{
  			OBCState = OBC_STATE_CONNECTED;
        ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
  			sprintf(LogBuffer, "[ObjectControl] ABORT received.\n"); ObjectControlSendLog(LogBuffer);
  		}
  		else if(iCommand == COMM_CONTROL)
  		{
  			ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
  			printf("[ObjectControl] Object control in CONTROL mode\n");     
  		}
      else if(iCommand == COMM_INIT)
      {
        
        sprintf(LogBuffer, "[ObjectControl] INIT received.\n"); ObjectControlSendLog(LogBuffer);
        /* Get objects; name and drive file */
        nbr_objects = 0;
        vFindObjectsInfo(object_traj_file,object_address_name,&nbr_objects);

        (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",&iForceObjectToLocalhost);
        //printf("ForceObjectToLocalhost = %d\n", iForceObjectToLocalhost);
        sprintf(LogBuffer, "[ObjectControl] ForceObjectToLocalhost = %d\n", iForceObjectToLocalhost); ObjectControlSendLog(LogBuffer);

        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          if(0 == iForceObjectToLocalhost)
          {
            object_udp_port[iIndex] = SAFETY_CHANNEL_PORT;
            object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT;
          }
          else
          {
            object_udp_port[iIndex] = SAFETY_CHANNEL_PORT + iIndex*2;
            object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT + iIndex*2;
          }
        }
         
        /*Setup Adaptive Sync Points (ASP)*/
        fd = fopen (ADAPTIVE_SYNC_POINT_CONF, "r");
        if(fd)
        {
          SyncPointCount = UtilCountFileRows(fd) - 1;
          fclose (fd);
          fd = fopen (ADAPTIVE_SYNC_POINT_CONF, "r");
          UtilReadLineCntSpecChars(fd, pcTempBuffer); //Read header   
          
          for(i = 0; i < SyncPointCount; i++)
          {
            UtilSetAdaptiveSyncPoint(&ASP[i], fd, 1);      
            if(TEST_SYNC_POINTS == 1) ASP[i].TestPort = SAFETY_CHANNEL_PORT;
          }  
          fclose (fd);
        }

        /*Setup Trigger and Action*/
        fd = fopen (TRIGG_ACTION_CONF, "r");
        if(fd)
        {
          TriggerActionCount = UtilCountFileRows(fd) - 1;
          fclose (fd);
          fd = fopen (TRIGG_ACTION_CONF, "r");
          UtilReadLineCntSpecChars(fd, pcTempBuffer); //Read header   
          
          for(i = 0; i < TriggerActionCount; i++)
          {
            UtilSetTriggActions(&TAA[i], fd, 1);
          }  
          fclose (fd);
        }

        bzero(TextBuffer, SMALL_BUFFER_SIZE_0);
        UtilSearchTextFile(CONF_FILE_PATH, "ASPMaxTimeDiff=", "", TextBuffer);
        ASPMaxTimeDiff = atof(TextBuffer);
        bzero(TextBuffer, SMALL_BUFFER_SIZE_0);
        UtilSearchTextFile(CONF_FILE_PATH, "ASPMaxTrajDiff=", "", TextBuffer);
        ASPMaxTrajDiff = atof(TextBuffer);
        bzero(TextBuffer, SMALL_BUFFER_SIZE_0); 
        UtilSearchTextFile(CONF_FILE_PATH, "ASPStepBackCount=", "", TextBuffer);
        ASPStepBackCount = atoi(TextBuffer);
        bzero(TextBuffer, SMALL_BUFFER_SIZE_0);
        UtilSearchTextFile(CONF_FILE_PATH, "ASPFilterLevel=", "", TextBuffer);
        ASPFilterLevel = atof(TextBuffer);
        bzero(TextBuffer, SMALL_BUFFER_SIZE_0);
        UtilSearchTextFile(CONF_FILE_PATH, "ASPMaxDeltaTime=", "", TextBuffer);
        ASPMaxDeltaTime = atof(TextBuffer);
        //printf("ASPMaxDeltaTime %3.3f\n", ASPMaxDeltaTime);
        bzero(TextBuffer, SMALL_BUFFER_SIZE_0);
        UtilSearchTextFile(CONF_FILE_PATH, "ASPDebugRate=", "", TextBuffer);
        ASPDebugRate = atoi(TextBuffer);

        sprintf(LogBuffer, "[ObjectControl] Objects to be controlled by server: %d\n", nbr_objects); ObjectControlSendLog(LogBuffer);
        sprintf(LogBuffer, "[ObjectControl] ASP in system: %d\n", SyncPointCount); ObjectControlSendLog(LogBuffer);
        sprintf(LogBuffer, "[ObjectControl] TAA in system: %d\n", TriggerActionCount); ObjectControlSendLog(LogBuffer);


        
        OBCState = OBC_STATE_INITIALIZED;
        sprintf(LogBuffer, "[ObjectControl] ObjectControl is initialized.\n"); ObjectControlSendLog(LogBuffer);
      }
      else if(iCommand == COMM_CONNECT && OBCState == OBC_STATE_INITIALIZED)
      {

        sprintf(LogBuffer, "[ObjectControl] CONNECT received.\n"); ObjectControlSendLog(LogBuffer);

        /* Connect and send drive files */
        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          UtilSetObjectPositionIP(&OP[iIndex], object_address_name[iIndex]);

          MessageLength =ObjectControlBuildOSEMMessage(MessageBuffer, &OSEMData, GPSTime,
                                      UtilSearchTextFile(CONF_FILE_PATH, "OrigoLatidude=", "", OriginLatitude),
                                      UtilSearchTextFile(CONF_FILE_PATH, "OrigoLongitude=", "", OriginLongitude),
                                      UtilSearchTextFile(CONF_FILE_PATH, "OrigoAltitude=", "", OriginAltitude),
                                      UtilSearchTextFile(CONF_FILE_PATH, "OrigoHeading=", "", OriginHeading),
                                      0); 
          DisconnectU8 = 0;

          do
          {
            
            iResult = vConnectObject(&socket_fd[iIndex],object_address_name[iIndex],object_tcp_port[iIndex], &DisconnectU8);

            if ( iResult < 0) 
            {
              if(errno == ECONNREFUSED)
              {
                
                sprintf(LogBuffer, "[ObjectControl] Was not able to connect to object, [IP: %s] [PORT: %d], retry in %d sec...\n",object_address_name[iIndex],object_tcp_port[iIndex], (!(1 & DisconnectU8))*3); ObjectControlSendLog(LogBuffer);
                (void)sleep(3);
              }
              else
              {
                util_error("ERR: Failed to connect to control socket");
              }
            }

            bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);
            //Have we received a command?
            if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER))
            {
              if(iCommand == COMM_DISCONNECT)
              {
                DisconnectU8 = 1;
                sprintf(LogBuffer, "[ObjectControl] DISCONNECT received.\n"); ObjectControlSendLog(LogBuffer);
              }

            }

          } while(iResult < 0 && DisconnectU8 == 0);

          if(iResult >= 0)
          {
            /* Send OSEM command */
            sprintf(LogBuffer, "[ObjectControl] Sending OSEM.\n"); ObjectControlSendLog(LogBuffer);
            vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);

            fd = fopen (object_traj_file[iIndex], "r");
            int RowCount = UtilCountFileRows(fd);
            fclose (fd);

            /*DOPM*/
            //MessageLength = ObjectControlBuildDOPMMessageHeader(TrajBuffer, RowCount-2, 0);

            /*Send DOPM header*/
           // vSendBytes(TrajBuffer, MessageLength, &socket_fd[iIndex], 0);

            /*Send DOPM data*/
            if(TEST_SYNC_POINTS == 1) printf("Trajfile: %s\n", object_traj_file[iIndex] ); 
           // ObjectControlSendDOPMMEssage(object_traj_file[iIndex], &socket_fd[iIndex], RowCount-2, (char *)&object_address_name[iIndex], object_tcp_port[iIndex], 0);

            /* Adaptive Sync Points object configuration start...*/
            OP[iIndex].TrajectoryPositionCount = RowCount-2;
            OP[iIndex].SpaceArr = SpaceArr[iIndex];
            OP[iIndex].TimeArr = TimeArr[iIndex];
            OP[iIndex].SpaceTimeArr = SpaceTimeArr[iIndex];
            UtilPopulateSpaceTimeArr(&OP[iIndex], object_traj_file[iIndex]);

            for(i = 0; i < SyncPointCount; i++)
            {
              if(TEST_SYNC_POINTS == 1 && iIndex == 1)
              {
                /*Send SYPM to slave*/
                MessageLength =ObjectControlBuildSYPMMessage(MessageBuffer, ASP[i].SlaveTrajSyncTime*1000, ASP[i].SlaveSyncStopTime*1000, 0);
                vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
              }
              else if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL)
              {
                /*Send SYPM to slave*/
                MessageLength =ObjectControlBuildSYPMMessage(MessageBuffer, ASP[i].SlaveTrajSyncTime*1000, ASP[i].SlaveSyncStopTime*1000, 0);
                vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
              }
            }

            /*Set Sync point in OP*/
            for(i = 0; i < SyncPointCount; i++)
            {
              if(TEST_SYNC_POINTS == 1 && iIndex == 0) UtilSetSyncPoint(&OP[iIndex], 0, 0, 0, ASP[i].MasterTrajSyncTime);
              else if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL) UtilSetSyncPoint(&OP[iIndex], 0, 0, 0, ASP[i].MasterTrajSyncTime); 
            }
            /* ...end*/


            /* Trigg And Action object configuration start...*/
            for(i = 0; i < TriggerActionCount; i++)
            {
              if(strstr(object_address_name[iIndex], TAA[i].TriggerIP) != NULL)
              {
                MessageLength = ObjectControlBuildTCMMessage(MessageBuffer, &TAA[i], 0);      
                vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
              }
            }
            /* ...end*/
          }

          for(iIndex=0;iIndex<nbr_objects;++iIndex)
          {
            if(USE_TEST_HOST == 0) vCreateSafetyChannel(object_address_name[iIndex], object_udp_port[iIndex], &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
            else if(USE_TEST_HOST == 1) vCreateSafetyChannel(TESTSERVER_IP, object_udp_port[iIndex], &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
          }
        
        }
        uiTimeCycle = 0;

        /* Execution mode*/
        ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

        /*Set server status*/
        ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;

        OriginLatitudeDbl = atof(OriginLatitude);
        OriginLongitudeDbl = atof(OriginLongitude);
        OriginAltitudeDbl = atof(OriginAltitude);
        OriginHeadingDbl = atof(OriginHeading);

        OriginPosition.Latitude = OriginLatitudeDbl;
        OriginPosition.Longitude = OriginLongitudeDbl;
        OriginPosition.Altitude = OriginAltitudeDbl;
        OriginPosition.Heading = OriginHeadingDbl;

        if(DisconnectU8 == 0) OBCState = OBC_STATE_CONNECTED;
        else if(DisconnectU8 == 1) OBCState = OBC_STATE_IDLE;
      }
  		else if(iCommand == COMM_DISCONNECT)
      {
        #ifndef NOTCP
          for(iIndex=0;iIndex<nbr_objects;++iIndex)
          {
            vDisconnectObject(&socket_fd[iIndex]);
          }
        #endif //NOTCP

        sprintf(LogBuffer, "[ObjectControl] DISCONNECT received.\n"); ObjectControlSendLog(LogBuffer);
          /* Close safety socket */
        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          vCloseSafetyChannel(&safety_socket_fd[iIndex]);
        }
        OBCState = OBC_STATE_IDLE;
      }
      else if(iCommand == COMM_EXIT)
  		{
        (void)iCommClose();
  			iExit = 1;
  		}
  		else
  		{
  		  #ifdef DEBUG
  		    printf("Unhandled command in object control\n");
  		    fflush(stdout);
  		  #endif
  		}
    }

    bzero(Buffer2, SMALL_BUFFER_SIZE_1);
    Buffer2[0] = OBCState;
    (void)iCommSend(COMM_OBC_STATE,Buffer2);
 
    if(!iExit)
    {
      /* Make call periodic */
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = TASK_PERIOD_MS*1000000;

      ++uiTimeCycle;
      if(uiTimeCycle >= HEARTBEAT_TIME_MS/TASK_PERIOD_MS)
      {
        uiTimeCycle = 0;
      }

      (void)nanosleep(&sleep_time,&ref_time);
    }
  }
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/


static void ObjectControlSendLog(C8* Message)
{
  (void)iCommSend(COMM_LOG, Message);
  printf("%s", Message);
  fflush(stdout);
}


int ObjectControlBuildTCMMessage(char* MessageBuffer, TriggActionType *TAA, char debug)
{
  int MessageIndex = 0;
  uint8_t MessageData = 0;
  uint16_t MessageDataU16 = 0;
  
  bzero(MessageBuffer, COMMAND_TCM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);


  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_TCM_CODE);


  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, TAA->TriggerId);


  if(strstr(TAA->TriggerType, "DI") != NULL && strstr(TAA->TriggerTypeVar, "LOW") != NULL) MessageData = TAA_TRIGGER_DI_LOW;
  else if(strstr(TAA->TriggerType, "DI") != NULL && strstr(TAA->TriggerTypeVar, "HIGH") != NULL) MessageData = TAA_TRIGGER_DI_HIGH; 
  else if(strstr(TAA->TriggerType, "DI") != NULL && strstr(TAA->TriggerTypeVar, "RISING_EDGE") != NULL) MessageData = TAA_TRIGGER_DI_RISING_EDGE; 
  else if(strstr(TAA->TriggerType, "DI") != NULL && strstr(TAA->TriggerTypeVar, "FALLING_EDGE") != NULL) MessageData = TAA_TRIGGER_DI_FALLING_EDGE; 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex, MessageData);

  MessageData = 0;
  if(strstr(TAA->ActionType, "SERVER") != NULL && TAA->Action == TAA_ACTION_EXT_START) MessageData = TAA_ACTION_EXT_START;
  else if(strstr(TAA->ActionType, "SERVER") != NULL && TAA->Action == TAA_ACTION_TEST_SIGNAL) MessageData = TAA_ACTION_TEST_SIGNAL; 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex, MessageData);
  
  MessageDataU16 = atoi(TAA->ActionDelay);
  MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, MessageDataU16);
  

  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);    

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}

int ObjectControlBuildACMMessage(char* MessageBuffer, TriggActionType *TAA, char debug)
{
  int MessageIndex = 0;
  uint8_t MessageData = 0;
  
  bzero(MessageBuffer, COMMAND_TCM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_TCM_CODE);
 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, (uint8_t)TAA->Action);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);    

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


int ObjectControlBuildMONRMessage(unsigned char *MonrData, MONRType *MONRData, char debug)
{
  int MessageIndex = 0, i = 0;
  double Data;
  U16 Crc = 0, U16Data = 0;
  I16 I16Data = 0;
  U32 U32Data = 0;
  I32 I32Data = 0;
  U64 U64Data = 0;
  char *p;

  U16Data = (U16Data | *MonrData) << 8;
  U16Data = U16Data | *(MonrData+1);
  MONRData->Header.SyncWordU16 = U16Data;
  MONRData->Header.TransmitterIdU8 = *(MonrData+2);
  MONRData->Header.MessageCounterU8 = *(MonrData+3);
  MONRData->Header.AckReqProtVerU8 = *(MonrData+4);
  U32Data = (U32Data | *(MonrData+5)) << 8;
  U32Data = (U32Data | *(MonrData+6)) << 8;
  U32Data = (U32Data | *(MonrData+7)) << 8;
  U32Data = U32Data | *(MonrData+8);
  MONRData->Header.MessageLengthU32 = U32Data;
  
  U16Data = 0;
  U16Data = (U16Data | *(MonrData+9)) << 8;
  U16Data = U16Data | *(MonrData+10);
  MONRData->MessageIdU16 = U16Data;
  U32Data = 0;
  U32Data = (U32Data | *(MonrData+11)) << 8;
  U32Data = (U32Data | *(MonrData+12)) << 8;
  U32Data = (U32Data | *(MonrData+13)) << 8;
  U32Data = U32Data | *(MonrData+14);
  MONRData->NOFValuesU32 = U32Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+15)) << 8;
  U16Data = U16Data | *(MonrData+16);
  MONRData->PositionTimeValueIdU16 = U16Data;
  MONRData->PositionTimeValueTypeU8 = *(MonrData+17);
  U64Data = 0;
  U64Data = (U64Data | *(MonrData+18)) << 8;
  U64Data = (U64Data | *(MonrData+19)) << 8;
  U64Data = (U64Data | *(MonrData+20)) << 8;
  U64Data = (U64Data | *(MonrData+21)) << 8;
  U64Data = (U64Data | *(MonrData+22)) << 8;
  U64Data = U64Data | *(MonrData+23);
  MONRData->PositionTimeU64 = U64Data;
  
  U16Data = 0;
  U16Data = (U16Data | *(MonrData+24)) << 8;
  U16Data = U16Data | *(MonrData+25);
  MONRData->XPositionValueIdU16 = U16Data;
  MONRData->XPositionValueTypeU8 = *(MonrData+26);
  I32Data = 0;
  I32Data = (I32Data | *(MonrData+27)) << 8;
  I32Data = (I32Data | *(MonrData+28)) << 8;
  I32Data = (I32Data | *(MonrData+29)) << 8;
  I32Data = I32Data | *(MonrData+30);
  MONRData->XPositionI32 = I32Data;
  
  U16Data = 0;
  U16Data = (U16Data | *(MonrData+31)) << 8;
  U16Data = U16Data | *(MonrData+32);
  MONRData->YPositionValueIdU16 = U16Data;
  MONRData->YPositionValueTypeU8 = *(MonrData+33);
  I32Data = 0;
  I32Data = (I32Data | *(MonrData+34)) << 8;
  I32Data = (I32Data | *(MonrData+35)) << 8;
  I32Data = (I32Data | *(MonrData+36)) << 8;
  I32Data = I32Data | *(MonrData+37);
  MONRData->YPositionI32 = (I32)I32Data;
  
  U16Data = 0;
  U16Data = (U16Data | *(MonrData+38)) << 8;
  U16Data = U16Data | *(MonrData+39);
  MONRData->ZPositionValueIdU16 = U16Data;
  MONRData->ZPositionValueTypeU8 = *(MonrData+40);
  I32Data = 0;
  I32Data = (I32Data | *(MonrData+41)) << 8;
  I32Data = (I32Data | *(MonrData+42)) << 8;
  I32Data = (I32Data | *(MonrData+43)) << 8;
  I32Data = I32Data | *(MonrData+44);
  MONRData->ZPositionI32 = (I32)I32Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+45)) << 8;
  U16Data = U16Data | *(MonrData+46);
  MONRData->HeadingValueIdU16 = U16Data;
  MONRData->HeadingValueTypeU8 = *(MonrData+47);
  U16Data = 0;
  U16Data = (U16Data | *(MonrData+48)) << 8;
  U16Data = U16Data | *(MonrData+49);
  MONRData->HeadingU16 = U16Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+50)) << 8;
  U16Data = U16Data | *(MonrData+51);
  MONRData->LongitudinalSpeedValueIdU16 = U16Data;
  MONRData->LongitudinalSpeedValueTypeU8 = *(MonrData+52);
  I16Data = 0;
  I16Data = (I16Data | *(MonrData+53)) << 8;
  I16Data = I16Data | *(MonrData+54);
  MONRData->LongitudinalSpeedI16 = I16Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+55)) << 8;
  U16Data = U16Data | *(MonrData+56);
  MONRData->LateralSpeedValueIdU16 = U16Data;
  MONRData->LateralSpeedValueTypeU8 = *(MonrData+57);
  I16Data = 0;
  I16Data = (I16Data | *(MonrData+58)) << 8;
  I16Data = I16Data | *(MonrData+59);
  MONRData->LateralSpeedI16 = I16Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+60)) << 8;
  U16Data = U16Data | *(MonrData+61);
  MONRData->LongitudinalAccValueIdU16 = U16Data;
  MONRData->LongitudinalAccValueTypeU8 = *(MonrData+62);
  I16Data = 0;
  I16Data = (I16Data | *(MonrData+63)) << 8;
  I16Data = I16Data | *(MonrData+64);
  MONRData->LongitudinalAccI16 = I16Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+65)) << 8;
  U16Data = U16Data | *(MonrData+66);
  MONRData->LateralAccValueIdU16 = U16Data;
  MONRData->LateralAccValueTypeU8 = *(MonrData+67);
  I16Data = 0;
  I16Data = (I16Data | *(MonrData+68)) << 8;
  I16Data = I16Data | *(MonrData+69);
  MONRData->LateralAccI16 = I16Data;

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+70)) << 8;
  U16Data = U16Data | *(MonrData+71);
  MONRData->DriveDirectionValueIdU16 = U16Data;
  MONRData->DriveDirectionValueTypeU8 = *(MonrData+72);
  MONRData->DriveDirectionU8 = *(MonrData+73);

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+74)) << 8;
  U16Data = U16Data | *(MonrData+75);
  MONRData->StateValueIdU16 = U16Data;
  MONRData->StateValueTypeU8 = *(MonrData+76);
  MONRData->StateU8 = *(MonrData+77);

  U16Data = 0;
  U16Data = (U16Data | *(MonrData+78)) << 8;
  U16Data = U16Data | *(MonrData+79);
  MONRData->StatusValueIdU16 = U16Data;
  MONRData->StatusValueTypeU8 = *(MonrData+80);
  MONRData->StatusU8 = *(MonrData+81);



  if(debug)
  {
    printf("SyncWord = %d\n", MONRData->Header.SyncWordU16);
    printf("TransmitterId = %d\n", MONRData->Header.TransmitterIdU8);
    printf("PackageCounter = %d\n", MONRData->Header.MessageCounterU8);
    printf("AckReq = %d\n", MONRData->Header.AckReqProtVerU8);
    printf("MessageLength = %d\n", MONRData->Header.MessageLengthU32);
    printf("NOFValues = %d\n", MONRData->NOFValuesU32);
  } 

  return 0;
}


//int ObjectControlMONRToASCII(MONRType *MONRData, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed, char *Heading, char *DriveDirection, char *StatusFlag, char debug)
int ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *LongitudinalSpeed, char *LateralSpeed, char *LongitudinalAcc, char *LateralAcc, char *Heading, char *DriveDirection, char *StatusFlag, char *StateFlag, char debug)
{
	char Buffer[6];
	long unsigned int MonrValueU64;
	unsigned int MonrValueU32;
	unsigned short MonrValueU16;
	unsigned char MonrValueU8;
	int i,j;
  double LatitudeDbl;
  double LongitudeDbl;
  double AltitudeDbl;
  double HeadingDbl;
  double iLlh[3] = {0, 0, 0};
  double xyz[3] = {0, 0, 0};
  double Llh[3] = {0, 0, 0};


	bzero(Id, SMALL_BUFFER_SIZE_1);
	bzero(Timestamp, SMALL_BUFFER_SIZE_0);
	bzero(Latitude, SMALL_BUFFER_SIZE_0);
	bzero(Longitude, SMALL_BUFFER_SIZE_0);
	bzero(Altitude, SMALL_BUFFER_SIZE_0);
  bzero(LongitudinalSpeed, SMALL_BUFFER_SIZE_0);
  bzero(LateralSpeed, SMALL_BUFFER_SIZE_0);
  bzero(LongitudinalAcc, SMALL_BUFFER_SIZE_0); 
  bzero(LateralAcc, SMALL_BUFFER_SIZE_0);  
	bzero(Heading, SMALL_BUFFER_SIZE_0);
	bzero(DriveDirection, SMALL_BUFFER_SIZE_1);
	bzero(StatusFlag, SMALL_BUFFER_SIZE_1);
  bzero(StateFlag, SMALL_BUFFER_SIZE_1);


	if(MONRData->MessageIdU16 == COMMAND_MONR_CODE)
	{
		//Index
		sprintf(Id, "%" PRIu8, (unsigned char)Idn);

		//Timestamp
		MonrValueU64 = 0;
		j=5;
		//for(i = 0; i <= 5; i++, j++) MonrValueU64 = *(MonrData+j) | (MonrValueU64 << 8);
		sprintf(Timestamp, "%" PRIu64, MONRData->PositionTimeU64);

		if(debug && MONRData->PositionTimeU64%400 == 0)
		{
  		//for(i = 0; i < 29; i ++) printf("%x-", (unsigned char)*(MONRData+i));
  		//printf("\n");

      printf("%x-", MONRData->MessageIdU16);
      printf("%x-", MONRData->Header.SyncWordU16);
      printf("%x-", MONRData->Header.TransmitterIdU8);
      printf("%x-", MONRData->Header.MessageCounterU8);
      printf("%x-", MONRData->Header.AckReqProtVerU8);
      printf("%x-", MONRData->Header.MessageLengthU32);
      printf("%x-", MONRData->NOFValuesU32);
      printf("%ld-", MONRData->PositionTimeU64);
      printf("%d-", MONRData->XPositionI32);
      printf("%d-", MONRData->YPositionI32);
      printf("%d-", MONRData->ZPositionI32);
      printf("%d-", MONRData->LongitudinalSpeedI16);
      printf("%d-", MONRData->HeadingU16);
      printf("%d-", MONRData->DriveDirectionU8);
      printf("\n");
		}

    iLlh[0] = OriginPosition->Latitude;
    iLlh[1] = OriginPosition->Longitude;
    iLlh[2] = OriginPosition->Altitude;

    xyz[0] = ((double)MONRData->XPositionI32)/1000;
    xyz[1] = ((double)MONRData->YPositionI32)/1000;
    xyz[2] = ((double)MONRData->ZPositionI32)/1000;

    enuToLlh(iLlh, xyz, Llh);

		//Latitude
		MonrValueU32 = 0;
		//for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
		sprintf(Latitude, "%" PRIi32, (I32)(Llh[0]*1e7));

		//Longitude
		MonrValueU32 = 0;
		//for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
		sprintf(Longitude, "%" PRIi32, (I32)(Llh[1]*1e7));

		//Altitude
		MonrValueU32 = 0;
		//for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
		sprintf(Altitude, "%" PRIi32, (I32)(Llh[2]));

		//Speed
		MonrValueU16 = 0;
		//for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
    sprintf(LongitudinalSpeed, "%" PRIi16, MONRData->LongitudinalSpeedI16);

    //LatSpeed
    MonrValueU16 = 0;
    //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
    sprintf(LateralSpeed, "%" PRIi16, MONRData->LateralSpeedI16);

   //LongAcc
    MonrValueU16 = 0;
    //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
    sprintf(LongitudinalAcc, "%" PRIi16, MONRData->LongitudinalAccI16);

   //LatAcc
    MonrValueU16 = 0;
    //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
    sprintf(LateralAcc, "%" PRIi16, MONRData->LateralAccI16);

		//Heading
		MonrValueU16 = 0;
		//for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(Heading, "%" PRIu16, MONRData->HeadingU16);

		//Driving direction
		//MonrValueU8 = (unsigned char)*(MonrData+j);
		//printf("D: %d\n", MonrValueU8 );
		j++;
		sprintf(DriveDirection, "%" PRIu8, MONRData->DriveDirectionU8);

		//Status flag
		//MonrValueU8 = (unsigned char)*(MonrData+j);
		sprintf(StatusFlag, "%" PRIu8, MONRData->StatusU8);
  
    //State flag
    //MonrValueU8 = (unsigned char)*(MonrData+j);
    sprintf(StateFlag, "%" PRIu8, MONRData->StateU8);
	}

  return 0;
}

int ObjectControlTOMToASCII(unsigned char *TomData, char *TriggId, char *TriggAction, char *TriggDelay, char debug)
{
  char Buffer[6];
  long unsigned int MonrValueU64;
  unsigned int MonrValueU32;
  unsigned short MonrValueU16;
  unsigned char MonrValueU8;
  int i,j;

  bzero(TriggId, SMALL_BUFFER_SIZE_1);
  bzero(TriggAction, SMALL_BUFFER_SIZE_1);
  bzero(TriggDelay, SMALL_BUFFER_SIZE_0);

  if(*TomData == COMMAND_TOM_CODE)
  {
  
    if(debug == 1)
    {
      for(i = 0; i < COMMAND_MESSAGE_HEADER_LENGTH+COMMAND_TOM_MESSAGE_LENGTH; i ++) printf("%x-", (unsigned char)TomData[i]);
      printf("\n");
    }
    
    //Trigg id
    j=5;
    MonrValueU8 = (unsigned char)*(TomData+j);
    sprintf(TriggId, "%" PRIu8, MonrValueU8);
    j++;

    //Trigg type
    MonrValueU8 = (unsigned char)*(TomData+j);
    sprintf(TriggAction, "%" PRIu8, MonrValueU8);
    j++;
  
    //Delay
    MonrValueU64 = 0;
    for(i = 0; i <= 5; i++, j++) MonrValueU64 = *(TomData+j) | (MonrValueU64 << 8);
    sprintf(TriggDelay, "%" PRIu64, MonrValueU64);

  }

  return 0;
}




int ObjectControlSendDOPMMEssage(char* Filename, int *Socket, int RowCount, char *IP, uint32_t Port,char debug)
{

  FILE *fd;
  fd = fopen (Filename, "r");
  UtilReadLineCntSpecChars(fd, TrajBuffer);//Read first line
  int Rest = 0, i = 0, MessageLength = 0, SumMessageLength = 0, Modulo = 0, Transmissions = 0;
  Transmissions = RowCount / COMMAND_DOPM_ROWS_IN_TRANSMISSION;
  Rest = RowCount % COMMAND_DOPM_ROWS_IN_TRANSMISSION;
 
  for(i = 0; i < Transmissions; i ++)
  {
    MessageLength = ObjectControlBuildDOPMMessage(TrajBuffer, fd, COMMAND_DOPM_ROWS_IN_TRANSMISSION, 0);
    vSendBytes(TrajBuffer, MessageLength, Socket, 0);
    SumMessageLength = SumMessageLength + MessageLength;
    if(debug) printf("Transmission %d: %d bytes sent.\n", i, MessageLength);
  }

  if(Rest > 0)
  {
    MessageLength = ObjectControlBuildDOPMMessage(TrajBuffer, fd, Rest, 0);
    vSendBytes(TrajBuffer, MessageLength, Socket, 0);
    SumMessageLength = SumMessageLength + MessageLength;
    if(debug) printf("Transmission %d: %d bytes sent.\n", i, MessageLength);
  }

  printf("[ObjectControl] %d DOPM bytes sent to %s, port %d\n", SumMessageLength, IP, Port);

  fclose (fd);

  return 0;
}


int ObjectControlBuildOSEMMessage(char* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, char *Latitude, char *Longitude, char *Altitude, char *Heading, char debug)
{
  int MessageIndex = 0, i = 0;
  double Data;
  U16 Crc = 0;
  char *p;
  
  bzero(MessageBuffer, COMMAND_OSEM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

  OSEMData->Header.SyncWordU16 = SYNC_WORD;
  OSEMData->Header.TransmitterIdU8 = 0;
  OSEMData->Header.MessageCounterU8 = 0;
  OSEMData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
  OSEMData->Header.MessageLengthU32 = sizeof(OSEMType) - sizeof(HeaderType) - 4;
  OSEMData->LatitudeValueIdU16 = VALUE_ID_LATITUDE;
  OSEMData->LatitudeI64 = (I64) ((atof((char *)Latitude) * 1e10));
  OSEMData->LongitudeValueIdU16 = VALUE_ID_LONGITUDE;
  OSEMData->LongitudeI64 = (I64)((atof((char *)Longitude) * 1e10));
  OSEMData->AltitudeValueIdU16 = VALUE_ID_ALTITUDE;
  OSEMData->AltitudeI32 = (I32)(atof((char *)Altitude) * 1e2);
  OSEMData->DateValueIdU16 = VALUE_ID_DATE_ISO8601;
  OSEMData->DateU32 = GPSTime->YearU16;
  OSEMData->DateU32 = (OSEMData->DateU32 << 8) | GPSTime->MonthU8;
  OSEMData->DateU32 = (OSEMData->DateU32 << 8) | GPSTime->DayU8;
 
  p=(char *)OSEMData;
  for(i=0; i<17; i++) *(MessageBuffer + i) = *p++;
  *p++; *p++;
  for(; i<23; i++) *(MessageBuffer + i) = *p++;
  *p++; *p++;
  for(; i<sizeof(OSEMType)-4; i++) *(MessageBuffer + i) = *p++;

  Crc = crc_16((const unsigned char *)MessageBuffer, sizeof(OSEMType)-4);
  Crc = 0;
  *(MessageBuffer + i++) = (U8)(Crc >> 8);
  *(MessageBuffer + i++) = (U8)(Crc);
  MessageIndex = i;
   
  if(debug)
  {
    printf("OSEM total length = %d bytes (header+message+footer)\n", (int)(COMMAND_OSEM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
    printf("----HEADER----\n");
    for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
    printf("\n----MESSAGE----\n");
    for(;i < sizeof(OSEMType)-4; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
    printf("\n----FOOTER----\n");
    for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
    printf("\n");
  }
  return MessageIndex; //Total number of bytes 
}


int ObjectControlBuildSTRTMessage(char* MessageBuffer, STRTType *STRTData, unsigned char CommandOption, unsigned long TimeStamp, char debug)
{
  int MessageIndex = 0, i = 0;
  U16 Crc = 0;
  char *p;
  
  bzero(MessageBuffer, COMMAND_STRT_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

  STRTData->Header.SyncWordU16 = (U16)(SYNC_WORD << 8 | SYNC_WORD >> 8);
  STRTData->Header.TransmitterIdU8 = 0;
  STRTData->Header.MessageCounterU8 = 0;
  STRTData->Header.AckReqProtVerU8 = 0;
  STRTData->Header.MessageLengthU32 = sizeof(STRTType) - 2  - sizeof(HeaderType);
  STRTData->Header.MessageLengthU32 = ((STRTData->Header.MessageLengthU32 << 24)&0xFF000000 | (STRTData->Header.MessageLengthU32 << 8)&0x00FF0000 | (STRTData->Header.MessageLengthU32 >> 8)&0x0000FF00 | (STRTData->Header.MessageLengthU32 >> 24)&0x000000FF);
  STRTData->MessageIdU16 = (COMMAND_STRT_CODE << 8 | COMMAND_STRT_CODE >> 8);
  STRTData->NOFValuesU32 = ((COMMAND_STRT_NOFV << 24)&0xFF000000 | (COMMAND_STRT_NOFV << 8)&0x00FF0000 | (COMMAND_STRT_NOFV >> 8)&0x0000FF00 | (COMMAND_STRT_NOFV >> 24)&0x000000FF);
  STRTData->StartTimeValueIdU16 = (VALUE_ID_GPS_SECOND_OF_WEEK << 8 | VALUE_ID_GPS_SECOND_OF_WEEK >> 8);
  STRTData->StartTimeValueTypeU8 = U48_CODE;
  STRTData->StartTimeU64 = TimeStamp;
  STRTData->StartTimeU64 = ((STRTData->StartTimeU64 << 40)&0xFF0000000000 | (STRTData->StartTimeU64 << 24)&0x00FF00000000 | (STRTData->StartTimeU64 << 8)&0x0000FF000000 |
                            (STRTData->StartTimeU64 >> 8)&0x000000FF0000 | (STRTData->StartTimeU64 >> 24)&0x00000000FF00 | (STRTData->StartTimeU64 >> 40)&0x0000000000FF);
  
  p=(char *)STRTData;
  for(i=0; i<sizeof(STRTType) - 2; i++) *(MessageBuffer + i) = *p++;
  Crc = crc_16((const unsigned char *)MessageBuffer, sizeof(STRTType) - 2);
  Crc = 0;
  *(MessageBuffer + i++) = (U8)(Crc >> 8);
  *(MessageBuffer + i++) = (U8)(Crc);
  MessageIndex = i;
   
  if(debug)
  {
    printf("STRT total length = %d bytes (header+message+footer)\n", (int)(COMMAND_STRT_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
    printf("----HEADER----\n");
    for(i = 0;i < sizeof(HeaderType); i ++) printf("[%d]=%x\n", i, (unsigned char)MessageBuffer[i]);
    printf("----MESSAGE----\n");
    for(;i < sizeof(STRTType) - 2; i ++) printf("[%d]=%x\n", i, (unsigned char)MessageBuffer[i]);
    printf("----FOOTER----\n");
    for(;i < MessageIndex; i ++) printf("[%d]=%x\n", i, (unsigned char)MessageBuffer[i]);
  }
  
  return MessageIndex; //Total number of bytes 
}


int ObjectControlBuildOSTMMessage(char* MessageBuffer, OSTMType *OSTMData, unsigned char CommandOption, char debug)
{
  int MessageIndex = 0, i;
  U16 Crc = 0;
  char *p;
  
  bzero(MessageBuffer, COMMAND_OSTM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

  OSTMData->Header.SyncWordU16 = (U16)(SYNC_WORD << 8 | SYNC_WORD >> 8);
  OSTMData->Header.TransmitterIdU8 = 0;
  OSTMData->Header.MessageCounterU8 = 0;
  OSTMData->Header.AckReqProtVerU8 = 0;
  OSTMData->Header.MessageLengthU32 = sizeof(OSTMType) - sizeof(HeaderType);
  OSTMData->Header.MessageLengthU32 = ((OSTMData->Header.MessageLengthU32 << 24)&0xFF000000 | (OSTMData->Header.MessageLengthU32 << 8)&0x00FF0000 | (OSTMData->Header.MessageLengthU32 >> 8)&0x0000FF00 | (OSTMData->Header.MessageLengthU32 >> 24)&0x000000FF);
  OSTMData->MessageIdU16 = (COMMAND_OSTM_CODE << 8 | COMMAND_OSTM_CODE >> 8);
  OSTMData->NOFValuesU32 = ((COMMAND_OSTM_NOFV << 24)&0xFF000000 | (COMMAND_OSTM_NOFV << 8)&0x00FF0000 | (COMMAND_OSTM_NOFV >> 8)&0x0000FF00 | (COMMAND_OSTM_NOFV >> 24)&0x000000FF);
  OSTMData->StateValueIdU16 = (VALUE_ID_FLAG << 8 | VALUE_ID_FLAG >> 8);
  OSTMData->StateValueTypeU8 = U8_CODE;
  OSTMData->StateU8 = (U8)CommandOption;
  
  p=(char *)OSTMData;
  for(i=0; i<sizeof(OSTMType); i++) *(MessageBuffer + i) = *p++;
  Crc = crc_16((const unsigned char *)MessageBuffer, sizeof(OSTMType));
  Crc = 0;
  *(MessageBuffer + i++) = (U8)(Crc >> 8);
  *(MessageBuffer + i++) = (U8)(Crc);
  MessageIndex = i;
   
  if(debug)
  {
    printf("OSTM total length = %d bytes (header+message+footer)\n", (int)(COMMAND_OSTM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
    printf("----HEADER----\n");
    for(i = 0;i < sizeof(HeaderType); i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
    printf("----MESSAGE----\n");
    for(;i < sizeof(OSTMType); i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
    printf("----FOOTER----\n");
    for(;i < MessageIndex; i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes 
}


int ObjectControlBuildHEABMessage(char* MessageBuffer, HEABType *HEABData, unsigned long TimeStamp, unsigned char CommandOption, char debug)
{
  int MessageIndex = 0, i;
  U16 Crc = 0;
  char *p;
  
  bzero(MessageBuffer, COMMAND_HEAB_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

  HEABData->Header.SyncWordU16 = (U16)(SYNC_WORD << 8 | SYNC_WORD >> 8);
  HEABData->Header.TransmitterIdU8 = 0;
  HEABData->Header.MessageCounterU8 = 0;
  HEABData->Header.AckReqProtVerU8 = 0;
  HEABData->Header.MessageLengthU32 = sizeof(HEABType) - 2 - sizeof(HeaderType);
  HEABData->Header.MessageLengthU32 = ((HEABData->Header.MessageLengthU32 << 24)&0xFF000000 | (HEABData->Header.MessageLengthU32 << 8)&0x00FF0000 | (HEABData->Header.MessageLengthU32 >> 8)&0x0000FF00 | (HEABData->Header.MessageLengthU32 >> 24)&0x000000FF);
  HEABData->MessageIdU16 = (COMMAND_HEAB_CODE << 8 | COMMAND_HEAB_CODE >> 8);
  HEABData->NOFValuesU32 = ((COMMAND_HEAB_NOFV << 24)&0xFF000000 | (COMMAND_HEAB_NOFV << 8)&0x00FF0000 | (COMMAND_HEAB_NOFV >> 8)&0x0000FF00 | (COMMAND_HEAB_NOFV >> 24)&0x000000FF);
  HEABData->TimeValueIdU16 = (VALUE_ID_GPS_SECOND_OF_WEEK << 8 | VALUE_ID_GPS_SECOND_OF_WEEK >> 8);
  HEABData->TimeValueTypeU8 = U48_CODE;
  HEABData->TimeU64 = TimeStamp;
  HEABData->TimeU64 = ((HEABData->TimeU64 << 40)&0xFF0000000000 | (HEABData->TimeU64 << 24)&0x00FF00000000 | (HEABData->TimeU64 << 8)&0x0000FF000000 |
                       (HEABData->TimeU64 >> 8)&0x000000FF0000 | (HEABData->TimeU64 >> 24)&0x00000000FF00 | (HEABData->TimeU64 >> 40)&0x0000000000FF);
  HEABData->StatusValueIdU16 = (VALUE_ID_FLAG << 8 | VALUE_ID_FLAG >> 8);
  HEABData->StatusValueTypeU8 = U8_CODE;
  HEABData->StatusU8 = (U8)CommandOption;
  
  p=(char *)HEABData;
  for(i=0; i<sizeof(HEABType)-6; i++) *(MessageBuffer + i) = *p++;
  *(MessageBuffer + i++) = (U8)(HEABData->StatusValueIdU16);
  *(MessageBuffer + i++) = (U8)(HEABData->StatusValueIdU16 >> 8);
  *(MessageBuffer + i++) = HEABData->StatusValueTypeU8;
  *(MessageBuffer + i++) = HEABData->StatusU8;
  Crc = crc_16((const unsigned char *)MessageBuffer, sizeof(HEABType)-2);
  Crc = 0;
  *(MessageBuffer + i++) = (U8)(Crc >> 8);
  *(MessageBuffer + i++) = (U8)(Crc);
  MessageIndex = i;
   
  if(debug)
  {
    printf("HEAB total length = %d bytes (header+message+footer)\n", (int)(COMMAND_HEAB_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
    printf("----HEADER----\n");
    for(i = 0;i < sizeof(HeaderType); i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
    printf("----MESSAGE----\n");
    for(;i < sizeof(HEABType)-2; i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
    printf("----FOOTER----\n");
    for(;i < MessageIndex; i ++) printf("[%d]=%d\n", i, (unsigned char)MessageBuffer[i]);
  }
  
  return MessageIndex; //Total number of bytes 

}


int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_LLCM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_LLCM_CODE);
 
  MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, Speed);
  
  MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, Curvature);

  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex, Mode);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}

int ObjectControlBuildSYPMMessage(char* MessageBuffer, unsigned int SyncPoint, unsigned int StopTime, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_SYPM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_SYPM_CODE);
 
  MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, SyncPoint);
  
  MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, StopTime);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}

int ObjectControlBuildMTSPMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_MTPS_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_MTPS_CODE);
 
  MessageIndex = UtilAddSixBytesMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, SyncTimestamp);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


int ObjectControlBuildDOPMMessage(char* MessageBuffer, FILE *fd, int RowCount, char debug)
{
  int MessageIndex = 0;
  char RowBuffer[100];
  char DataBuffer[20];
  double Data;
  char *src;
  
 
  bzero(MessageBuffer, COMMAND_DOPM_ROW_MESSAGE_LENGTH*RowCount);

  int i = 0;
  for(i = 0; i <= RowCount - 1; i++)
  {
    bzero(RowBuffer, 100);
    UtilReadLineCntSpecChars(fd, RowBuffer);

    //Read to ';' in row = LINE;0.00;21.239000;39.045000;0.000000;-1.240001;0.029103;0.004005;0.000000;3;ENDLINE;
    //Time
    src = strchr(RowBuffer, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e3;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned int)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);
    
    //x
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e3;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned int)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //y
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e3;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned int)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //z
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e3;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned int)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //Heading
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = UtilRadToDeg(atof(DataBuffer)*1e1);
    Data = 4500 - Data; //Turn heading back pi/2 
    while(Data<0) Data+=3600;
    while(Data>3600) Data-=3600;

    MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //Speed
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e2;
    MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //Acceleration
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*1e1;
    MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //Curvature
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = atof(DataBuffer)*3e4;
    MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
    //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

    //Mode
    src = strchr(src + 1, ';');
    bzero(DataBuffer, 20);
    strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
    Data = (double)atoi(DataBuffer);
    MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex, (unsigned char)Data);
    //printf("DataBuffer=%s  float=%d\n", DataBuffer, (unsigned char)Data);
  }

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++)
    {
      if((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15) printf("0");
      printf("%x-", (unsigned char)MessageBuffer[i]);
    }
    printf("\n");
  }
  return MessageIndex; //Total number of bytes
}


int ObjectControlBuildDOPMMessageHeader(char* MessageBuffer, int RowCount, char debug)
{
  
  bzero(MessageBuffer, COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_DOPM_CODE);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, COMMAND_DOPM_ROW_MESSAGE_LENGTH*RowCount);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < COMMAND_MESSAGE_HEADER_LENGTH; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return COMMAND_MESSAGE_HEADER_LENGTH; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH
}




static I32 vConnectObject(int* sockfd, const char* name, const uint32_t port, U8 *Disconnect)
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
    printf("INF: Try to connect to socket: %s %i\n",name,port);
    fflush(stdout);
  #endif
  
 // do
  {
    iResult = connect(*sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));

    /*if ( iResult < 0) 
    {
      if(errno == ECONNREFUSED)
      {
        printf("WARNiNG: Was not able to connect to object, [IP: %s] [PORT: %d], %d retry in 3 sec...\n",name,port, *Disconnect);
        fflush(stdout);
        (void)sleep(3);
      }
      else
      {
        util_error("ERR: Failed to connect to control socket");
      }*/
    }
  //} while(iResult < 0 && *Disconnect == 0);

  #ifdef DEBUG
    printf("INF: Connected to command socket: %s %i\n",name,port);
    fflush(stdout);
  #endif

  return iResult;
}

static void vDisconnectObject(int* sockfd)
{
  close(*sockfd);
}

static void vSendString(const char* command, int* sockfd)
{
  int n;

  #ifdef DEBUG
    printf("INF: Sending: <%s>\n",command);
    fflush(stdout);
  #endif

  n = write(*sockfd, command, strlen(command));
  if (n < 0)
  {
    util_error("[ObjectControl] ERR: Failed to send on control socket");
  }
}


static void vSendBytes(const char* data, int length, int* sockfd, int debug)
{
  int n;

  if(debug){ printf("Bytes sent: "); int i = 0; for(i = 0; i < n; i++) printf("%x-", (unsigned char)*(data+i)); printf("\n");}

  n = write(*sockfd, data, length);
  if (n < 0)
  {
    util_error("[ObjectControl] ERR: Failed to send on control socket");
  }
}



static void vSendFile(const char* object_traj_file, int* sockfd)
{
  FILE *filefd;
  char buffer[1024];
  int n;
  size_t readBytes;

  #ifdef DEBUG
    printf("INF: Open file %s\n",object_traj_file);
  #endif

  filefd = fopen (object_traj_file, "rb");
  if (filefd == NULL)
  {
    util_error("ERR: Failed to open trajectory file");
  }

  do
  {
    readBytes = fread(buffer,1,1024,filefd);
    if(readBytes > 0)
    {
      #ifdef DEBUG
        printf("INF: Sending: <%s>\n",buffer);
        printf("INF: Try to sending nbr of bytes: <%lu>\n",readBytes);
        fflush(stdout);
      #endif
      n = write(*sockfd, buffer, readBytes);
      if (n < 0)
      {
        util_error("ERR: Failed to send on control socket");
      }
    }
  } while(readBytes > 0);

  fclose(filefd);
}

static void vCreateSafetyChannel(const char* name, const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
  int result;
  struct hostent *object;

  /* Connect to object safety socket */
  #ifdef DEBUG
    printf("INF: Creating safety socket\n");
    fflush(stdout);
  #endif

  *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
  {
    util_error("ERR: Failed to connect to monitor socket");
  }

  /* Set address to object */
  object = gethostbyname(name);
  
  if (object==0)
  {
    util_error("ERR: Unknown host");
  }

  bcopy((char *) object->h_addr, (char *)&addr->sin_addr.s_addr, object->h_length);
  addr->sin_family = AF_INET;
  addr->sin_port = htons(port);

   /* set socket to non-blocking */
  result = fcntl(*sockfd, F_SETFL, 
    fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
  if (result < 0)
  {
    util_error("ERR: calling fcntl");
  }

  #ifdef DEBUG
    printf("INF: Created socket and safety address: %s %d\n",name,port);
    fflush(stdout);
  #endif

}

static void vCloseSafetyChannel(int* sockfd)
{
  close(*sockfd);
}

int ObjectControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug)
{
    int result;
 
    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));

    if (result < 0)
    {
      util_error("ERR: Failed to send on monitor socket");
    }

    return 0;
}


static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand)
{
    int result;
    char pcCommand[10];

    bzero(pcCommand,10);

    #ifdef DEBUG
      printf("INF: Sending: <HEBT>\n");
      fflush(stdout);
    #endif

    if(COMMAND_HEARTBEAT_GO == tCommand)
    {
      strcat(pcCommand,"HEBT;g;");
    }
    else
    {
      strcat(pcCommand,"HEBT;A;");
    }

    #ifdef DEBUG
     // printf("INF: Sending: <%s>\n",pcCommand);
     // fflush(stdout);
    #endif

    result = sendto(*sockfd,
      pcCommand,
      10,
      0,
      (const struct sockaddr *) addr,
      sizeof(struct sockaddr_in));

    if (result < 0)
    {
      util_error("ERR: Failed to send on monitor socket");
    }
}

static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData)
{
  int result;
  *recievedNewData = 0;
    do
    {
      result = recv(*sockfd, buffer, length, 0);
      
      if (result < 0)
      {
        if(errno != EAGAIN && errno != EWOULDBLOCK)
        {
          util_error("ERR: Failed to receive from monitor socket");
        }
        else
        {
          #ifdef DEBUG
            printf("INF: No data receive\n");
            fflush(stdout);
          #endif
        }
      }
      else
      {
        *recievedNewData = 1;
        #ifdef DEBUG
          printf("INF: Received: <%s>\n",buffer);
          fflush(stdout);
        #endif
      }
    } while(result > 0 );
}

void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], char object_address_name[MAX_OBJECTS][MAX_FILE_PATH], int* nbr_objects)
{
  DIR* traj_directory;
  struct dirent *directory_entry;
  int iForceObjectToLocalhost;

  iForceObjectToLocalhost = 0;

  traj_directory = opendir(TRAJECTORY_PATH);
  if(traj_directory == NULL)
  {
    util_error("ERR: Failed to open trajectory directory");
  }

  (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",&iForceObjectToLocalhost);

  while ((directory_entry = readdir(traj_directory)) && ((*nbr_objects) < MAX_OBJECTS))
  {

    /* Check so it's not . or .. */
    if (strncmp(directory_entry->d_name,".",1) && (strstr(directory_entry->d_name, "sync") == NULL) )
    {
      bzero(object_address_name[(*nbr_objects)],MAX_FILE_PATH);

      bzero(object_traj_file[(*nbr_objects)],MAX_FILE_PATH);
      (void)strcat(object_traj_file[(*nbr_objects)],TRAJECTORY_PATH);
      (void)strcat(object_traj_file[(*nbr_objects)],directory_entry->d_name);

      if(0 == iForceObjectToLocalhost)
      {
        (void)strncat(object_address_name[(*nbr_objects)],directory_entry->d_name,strlen(directory_entry->d_name));
      }
      else
      {
        if(USE_TEST_HOST == 0) (void)strcat(object_address_name[(*nbr_objects)],LOCALHOST);
        else if (USE_TEST_HOST == 1)(void)strcat(object_address_name[(*nbr_objects)],TESTHOST_IP);
        
      }

      ++(*nbr_objects);
    }
  }
  (void)closedir(traj_directory);
}
