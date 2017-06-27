/*------------------------------------------------------------------------------
  -- Copyright   : (C) 2016 CHRONOS project
  ------------------------------------------------------------------------------
  -- File        : objectcontrol.c
  -- Author      : Karl-Johan Ode, Sebastian Loh Lindholm
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
#define BYTEBASED

#define COMMAND_MESSAGE_HEADER_LENGTH 5
#define COMMAND_CODE_INDEX 0
#define COMMAND_MESSAGE_LENGTH_INDEX 1

#define COMMAND_DOPM_CODE 1
#define COMMAND_DOPM_ROW_MESSAGE_LENGTH 25 
#define COMMAND_DOPM_ROWS_IN_TRANSMISSION  40

#define COMMAND_OSEM_CODE 2
#define COMMAND_OSEM_MESSAGE_LENGTH 14 

#define COMMAND_AROM_CODE 3
#define COMMAND_AROM_MESSAGE_LENGTH 1
#define COMMAND_AROM_OPT_SET_ARMED_STATE 1
#define COMMAND_AROM_OPT_SET_DISARMED_STATE 2 

#define COMMAND_STRT_CODE  4
#define COMMAND_STRT_MESSAGE_LENGTH  7
#define COMMAND_STRT_OPT_START_IMMEDIATELY 1
#define COMMAND_STRT_OPT_START_AT_TIMESTAMP 2  

#define COMMAND_HEAB_CODE 5
#define COMMAND_HEAB_MESSAGE_LENGTH 1
#define COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING 0
#define COMMAND_HEAB_OPT_SERVER_STATUS_OK 1
#define COMMAND_HEAB_OPT_SERVER_STATUS_ABORT 2 


#define COMMAND_LLCM_CODE 8
#define COMMAND_LLCM_MESSAGE_LENGTH 5

#define COMMAND_SYPM_CODE 9
#define COMMAND_SYPM_MESSAGE_LENGTH 8

#define COMMAND_MTPS_CODE 10
#define COMMAND_MTPS_MESSAGE_LENGTH 6

#define CONF_FILE_PATH  "conf/test.conf"

#define SMALL_BUFFER_SIZE_0 20
#define SMALL_BUFFER_SIZE_1 2

#define TRAJECTORY_FILE_MAX_ROWS  4096

typedef enum {
  COMMAND_HEARTBEAT_GO,
  COMMAND_HEARTBEAT_ABORT
} hearbeatCommand_t;

char TrajBuffer[COMMAND_DOPM_ROWS_IN_TRANSMISSION*COMMAND_DOPM_ROW_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH];


/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static void vConnectObject(int* sockfd,const char* name,const uint32_t port);
static void vSendString(const char* command,int* sockfd);
static void vSendBytes(const char* command, int length, int* sockfd, int debug);
static void vSendFile(const char* object_traj_file, int* sockfd);
static void vDisconnectObject(int* sockfd);

static void vCreateSafetyChannel(const char* name,const uint32_t port,
  int* sockfd, struct sockaddr_in* addr);
static void vCloseSafetyChannel(int* sockfd);
static void vSendHeartbeat(int* sockfd, struct sockaddr_in* addr, hearbeatCommand_t tCommand);
static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData);
int ObjectControlBuildOSEMMessage(char* MessageBuffer, char *Latitude, char *Longitude, char *Altitude, char *Heading, char debug);
int ObjectControlBuildSTRTMessage(char* MessageBuffer, unsigned char CommandOption, unsigned long TimeStamp, char debug);
int ObjectControlBuildAROMMessage(char* MessageBuffer, unsigned char CommandOption, char debug);
int ObjectControlBuildHEABMessage(char* MessageBuffer, unsigned char CommandOption, char debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
int ObjectControlBuildSYPMMessage(char* MessageBuffer, unsigned int SyncPoint, unsigned int StopTime, char debug);
int ObjectControlBuildMTPSMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug);
int ObjectControlBuildDOPMMessageHeader(char* MessageBuffer, int RowCount, char debug);
int ObjectControlBuildDOPMMessage(char* MessageBuffer, FILE *fd, int RowCount, char debug);
int ObjectControlSendDOPMMEssage(char* Filename, int *Socket, int RowCount, char *IP, char debug);
int ObjectControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);
int ObjectControlMONRToASCII(unsigned char *MonrData, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed ,char *Heading, char *DriveDirection, char *StatusFlag, char debug);
int ObjectControlBuildMONRMessage(unsigned char *MonrData, uint64_t *Timestamp, int32_t *Latitude, int32_t * Longitude, int32_t *Altitude, uint16_t *Speed, uint16_t *Heading, uint8_t *DriveDirection);


static void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  int* nbr_objects);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/
void objectcontrol_task()
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
  char Id[SMALL_BUFFER_SIZE_1];
  char Timestamp[SMALL_BUFFER_SIZE_0];
  char Latitude[SMALL_BUFFER_SIZE_0];
  char Longitude[SMALL_BUFFER_SIZE_0];
  char Altitude[SMALL_BUFFER_SIZE_0];
  char Speed[SMALL_BUFFER_SIZE_0];
  char Heading[SMALL_BUFFER_SIZE_0];
  char DriveDirection[SMALL_BUFFER_SIZE_1];
  char StatusFlag[SMALL_BUFFER_SIZE_1];
  int MessageLength;
  char *MiscPtr;
  uint64_t StartTimeU64 = 0;
  uint64_t CurrentTimeU64 = 0;
  uint64_t MasterTimeToSyncPointU64 = 0;
  double TimeToSyncPoint = 0;
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
  double OriginLatitudeDbl;
  double OriginLongitudeDbl;
  double OriginAltitudeDbl;
  double OriginHeadingDbl;
  AdaptiveSyncPoint ASP[MAX_ADAPTIVE_SYNC_POINTS];
  int SyncPointCount = 0;


  unsigned char ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING;


  (void)iCommInit(IPC_RECV_SEND,MQ_OC,1);

    /* Get objects; name and drive file */
  vFindObjectsInfo(object_traj_file,object_address_name,&nbr_objects);


  (void)iUtilGetIntParaConfFile("ForceObjectToLocalhost",&iForceObjectToLocalhost);

  
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

#ifdef BYTEBASED
 

  
  /*Setup Adaptive Sync Points (ASP)*/
  fd = fopen (ADAPTIVE_SYNC_POINT_CONF, "r");
  SyncPointCount = UtilCountFileRows(fd) - 1;
  fclose (fd);
  fd = fopen (ADAPTIVE_SYNC_POINT_CONF, "r");
  UtilReadLineCntSpecChars(fd, pcTempBuffer); //Read header   
  
  for(i = 0; i < SyncPointCount; i++)  UtilSetAdaptiveSyncPoint(&ASP[i], fd, 0);      
  fclose (fd);

#else

  bzero(pcBuffer,OBJECT_MESS_BUFFER_SIZE);
  strcat(pcBuffer,"OSEM;");
  if(iUtilGetParaConfFile("OrigoLatidude",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoLatidude");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoLongitude",pcTempBuffer))
  {
    strcat(pcBuffer,"OrigoLongitude");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoAltitude",pcTempBuffer))
  {
    printf("%s\n",pcTempBuffer);
    strcat(pcBuffer,"OrigoAltitude");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  if(iUtilGetParaConfFile("OrigoHeading",pcTempBuffer))
  {
    printf("%s\n",pcTempBuffer);
    strcat(pcBuffer,"OrigoHeading");
    strcat(pcBuffer,pcTempBuffer);
    strcat(pcBuffer,";");
  }
  strcat(pcBuffer,"ENDOSEM;");

    printf("[ObjectControl] Created OSEM: \"%s\"\n",pcBuffer);
    fflush(stdout);

  #endif

  
  /* Connect and send drive files */
  printf("[ObjectControl] Objects controlled by server: %d\n", nbr_objects);
  printf("[ObjectControl] ASP in the system: %d\n", SyncPointCount);
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    UtilSetObjectPositionIP(&OP[iIndex], object_address_name[iIndex]);

    MessageLength =ObjectControlBuildOSEMMessage(MessageBuffer, 
                                UtilSearchTextFile(CONF_FILE_PATH, "OrigoLatidude=", "", OriginLatitude),
                                UtilSearchTextFile(CONF_FILE_PATH, "OrigoLongitude=", "", OriginLongitude),
                                UtilSearchTextFile(CONF_FILE_PATH, "OrigoAltitude=", "", OriginAltitude),
                                UtilSearchTextFile(CONF_FILE_PATH, "OrigoHeading=", "", OriginHeading),
                                0); 
 
    vConnectObject(&socket_fd[iIndex],object_address_name[iIndex],object_tcp_port[iIndex]);


    /* Send OSEM command */
    #ifdef BYTEBASED
      vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);

    #else
      vSendString(pcBuffer,&socket_fd[iIndex]);
    #endif
   
    #ifdef BYTEBASED
      fd = fopen (object_traj_file[iIndex], "r");
      int RowCount = UtilCountFileRows(fd);
      fclose (fd);

      /*DOPM*/
      MessageLength = ObjectControlBuildDOPMMessageHeader(TrajBuffer, RowCount-2, 0);

      /*Send DOPM header*/
      vSendBytes(TrajBuffer, MessageLength, &socket_fd[iIndex], 0);

      /*Send DOPM data*/
      ObjectControlSendDOPMMEssage(object_traj_file[iIndex], &socket_fd[iIndex], RowCount-2, (char *)&object_address_name[iIndex], 0);

      /* Adaptive Sync Points...*/
      OP[iIndex].TrajectoryPositionCount = RowCount-2;
      OP[iIndex].SpaceArr = SpaceArr[iIndex];
      OP[iIndex].TimeArr = TimeArr[iIndex];
      OP[iIndex].SpaceTimeArr = SpaceTimeArr[iIndex];
      UtilPopulateSpaceTimeArr(&OP[iIndex], object_traj_file[iIndex]);

      for(i = 0; i < SyncPointCount; i++)
      {
        if(TEST_SYNC_POINTS == 1 && iIndex != 0)
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

    #else
      vSendString("DOPM;",&socket_fd[iIndex]);
      vSendFile(object_traj_file[iIndex],&socket_fd[iIndex]);
      vSendString("ENDDOPM;",&socket_fd[iIndex]);
    #endif
  }

  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    if(USE_TEST_HOST == 0)
    {
    vCreateSafetyChannel(object_address_name[iIndex],object_udp_port[iIndex],
      &safety_socket_fd[iIndex],&safety_object_addr[iIndex]);
    }
    else if(USE_TEST_HOST == 1)
    {
    vCreateSafetyChannel(TESTSERVER_IP,object_udp_port[iIndex],
      &safety_socket_fd[iIndex],&safety_object_addr[iIndex]);
    }
  }

  #ifdef DEBUG
    printf("INF: Object control start waiting for command\n");
    fflush(stdout);
  #endif

  uint8_t uiTimeCycle = 0;


  /* Execution mode*/
  int ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

  /*Set server status*/
  ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;
  
  OriginLatitudeDbl = atof(OriginLatitude);
  OriginLongitudeDbl = atof(OriginLongitude);
  OriginAltitudeDbl = atof(OriginAltitude);
  OriginHeadingDbl = atof(OriginHeading);

  while(!iExit)
  {
    char buffer[RECV_MESSAGE_BUFFER];
    int recievedNewData = 0;
    gettimeofday(&CurrentTimeStruct, NULL);
    CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;

    /*HEAB*/
    for(iIndex=0;iIndex<nbr_objects;++iIndex)
    {
      if(uiTimeCycle == 0)
      {
        #ifdef BYTEBASED
          HeartbeatMessageCounter ++;
          MessageLength = ObjectControlBuildHEABMessage(MessageBuffer, ObjectControlServerStatus, 0);
          ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
        #else
          vSendHeartbeat(&safety_socket_fd[iIndex],&safety_object_addr[iIndex],COMMAND_HEARTBEAT_GO);
        #endif
      }
    }

    /*MTPS*/
    if(HeartbeatMessageCounter == 10)
    {
      HeartbeatMessageCounter = 0;
      for(iIndex=0;iIndex<nbr_objects;++iIndex)
      {
            #ifdef BYTEBASED
            for(i = 0; i < SyncPointCount; i++)
            {
              if(TEST_SYNC_POINTS == 1 && iIndex != 0 && MasterTimeToSyncPointU64 != 0)
              {
                  /*Send Master time to adaptive sync point*/
                  MessageLength =ObjectControlBuildMTPSMessage(MessageBuffer, MasterTimeToSyncPointU64, 0);
                  ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
              }
              else if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL && MasterTimeToSyncPointU64 != 0)
              {
                  /*Send Master time to adaptive sync point*/
                  MessageLength =ObjectControlBuildMTPSMessage(MessageBuffer, MasterTimeToSyncPointU64, 0);
                  ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
              }
            }
            #endif
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
        
        #ifdef BYTEBASED
            ObjectControlMONRToASCII(buffer, iIndex, Id, Timestamp, Latitude, Longitude, Altitude, Speed, Heading, DriveDirection, StatusFlag, 1);
            bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
            strcat(buffer, "MONR;"); strcat(buffer,Id); strcat(buffer,";"); strcat(buffer,Timestamp); strcat(buffer,";"); strcat(buffer,Latitude); strcat(buffer,";"); strcat(buffer,Longitude);
            strcat(buffer,";"); strcat(buffer,Altitude); strcat(buffer,";"); strcat(buffer,Speed); strcat(buffer,";"); strcat(buffer,Heading); strcat(buffer,";");
            strcat(buffer,DriveDirection); strcat(buffer,";"); //strcat(pcBuffer,StatusFlag); strcat(pcBuffer,";");

            for(i = 0; i < SyncPointCount; i++)
            {
              if(strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL && iIndex == 0)
              {

                UtilCalcPositionDelta(OriginLatitudeDbl,OriginLongitudeDbl,atof(Latitude)/1e7,atof(Longitude)/1e7, &OP[iIndex]);
                //printf("OrigoDistance = %3.5f\n", OP[iIndex].OrigoDistance );
                //printf("Current time: %ld\n", CurrentTimeU64);
                //printf("Current time: %ld\n", StartTimeU64);
                //printf("Current time diff: %3.2f\n", (((double)CurrentTimeU64-(double)StartTimeU64)/1000));
                UtilFindCurrentTrajectoryPosition(&OP[iIndex], 0, (((double)CurrentTimeU64-(double)StartTimeU64)/1000), 0.2);
                TimeToSyncPoint = (UtilCalculateTimeToSync(&OP[iIndex]) - ((((double)CurrentTimeU64-(double)StartTimeU64)/1000) - OP[iIndex].TimeArr[OP[iIndex].BestFoundTrajectoryIndex]));
                if(atoi(Timestamp)%100 == 0) printf("Time to sync= %3.3f\n", TimeToSyncPoint);
                if(TimeToSyncPoint > 0) MasterTimeToSyncPointU64 = StartTimeU64 + (uint64_t)(TimeToSyncPoint*1000);
                else MasterTimeToSyncPointU64 = 0;
               }
            }

        #else
        /* Get monitor data */
        sscanf(buffer,"MONR;%" SCNu64 ";%" SCNd32 ";%" SCNd32 ";%" SCNd32 ";%" SCNu16 ";%" SCNu16 ";%" SCNu8 ";",
          &ldm[iIndex].timestamp,&ldm[iIndex].latitude,&ldm[iIndex].longitude,
          &ldm[iIndex].altitude,&ldm[iIndex].speed,&ldm[iIndex].heading,&ldm[iIndex].drivedirection);

        bzero(buffer,RECV_MESSAGE_BUFFER);
        sprintf ( buffer,
          "%" PRIu16 ";0;%" PRIu64 ";%" PRId32 ";%" PRId32 ";%" PRId32 ";%" PRIu16 ";%" PRIu16 ";%" PRIu8 ";",
          iIndex,ldm[iIndex].timestamp,ldm[iIndex].latitude,ldm[iIndex].longitude,
          ldm[iIndex].altitude,ldm[iIndex].speed,ldm[iIndex].heading,ldm[iIndex].drivedirection);
        #endif

        #ifdef DEBUG
          printf("INF: Send MONITOR message: %s\n",buffer);
          fflush(stdout);
        #endif

        if(ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) (void)iCommSend(COMM_MONI,buffer);
      }
    }

    bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);

    // Have we recieved a command?
    if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER))
    {

      #ifdef DEBUG
        printf("INF: Object control command %d\n",iCommand);
        fflush(stdout);
      #endif

      if(iCommand == COMM_ARMD)
      {
         //MessageLength = ObjectControlBuildAROMMessage(MessageBuffer, COMMAND_AROM_OPT_SET_ARMED_STATE, 0);
        if(pcRecvBuffer[0] == COMMAND_AROM_OPT_SET_ARMED_STATE) printf("[ObjectControl] Sending ARM: %d\n", pcRecvBuffer[0]);
        else if(pcRecvBuffer[0] == COMMAND_AROM_OPT_SET_DISARMED_STATE) printf("[ObjectControl] Sending DISARM: %d\n", pcRecvBuffer[0]);
        MessageLength = ObjectControlBuildAROMMessage(MessageBuffer, pcRecvBuffer[0], 0);

        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          #ifdef BYTEBASED
            /*Send AROM message*/
            vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
          #else
            vSendString("AROM;ENDAROM;",&socket_fd[iIndex]);
          #endif
        }
      }
      else if(iCommand == COMM_STRT)
      {  
        //#ifdef DEBUG
          printf("[ObjectControl] STRT trig recieved <%s>\n",pcRecvBuffer);
          fflush(stdout);
        //#endif

        #ifdef BYTEBASED
          bzero(Timestamp, SMALL_BUFFER_SIZE_0);
          MiscPtr =strchr(pcRecvBuffer,';');
          strncpy(Timestamp, MiscPtr+1, (uint64_t)strchr(MiscPtr+1, ';') - (uint64_t)MiscPtr  - 1);
          StartTimeU64 = atol(Timestamp);
          MasterTimeToSyncPointU64 = 0;
          MessageLength = ObjectControlBuildSTRTMessage(MessageBuffer, COMMAND_STRT_OPT_START_AT_TIMESTAMP, StartTimeU64, 0);
        #else
          bzero(pcBuffer,OBJECT_MESS_BUFFER_SIZE);
          strncat(pcBuffer,"TRIG;",5);
          strncat(pcBuffer,pcRecvBuffer,OBJECT_MESS_BUFFER_SIZE-13);
          strncat(pcBuffer,"ENDTRIG;",8);
        #endif

        #ifdef DEBUG
          printf("INF: Sending START trig from object control <%s>\n",pcBuffer);
          fflush(stdout);
        #endif

        for(iIndex=0;iIndex<nbr_objects;++iIndex)
        {
          #ifdef BYTEBASED
            vSendBytes(MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
          #else
            vSendString(pcBuffer,&socket_fd[iIndex]);
          #endif
        }
      }
    	else if(iCommand == COMM_REPLAY)
    	{
  			ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
        printf("[ObjectControl] Object control REPLAY mode <%s>\n", pcRecvBuffer);
  			fflush(stdout);
  		}
      else if(iCommand == COMM_ABORT)
      {
        ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
        printf("[ObjectControl] Object control ABORT mode <%s>\n", pcRecvBuffer);
        fflush(stdout);
      }
      else if(iCommand == COMM_CONTROL)
      {
        ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
        printf("[ObjectControl] Object control in CONTROL mode\n");     
      }	
      else if(iCommand == COMM_EXIT)
      {
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

#ifndef NOTCP
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vDisconnectObject(&socket_fd[iIndex]);
  }
#endif //NOTCP

    /* Close safety socket */
  for(iIndex=0;iIndex<nbr_objects;++iIndex)
  {
    vCloseSafetyChannel(&safety_socket_fd[iIndex]);
  }

  (void)iCommClose();
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

int ObjectControlBuildMONRMessage(unsigned char *MonrData, uint64_t *Timestamp, int32_t *Latitude, int32_t * Longitude, int32_t *Altitude, uint16_t *Speed, uint16_t *Heading, uint8_t *DriveDirection)
{


  return 0;
}


int ObjectControlMONRToASCII(unsigned char *MonrData, int Idn, char *Id, char *Timestamp, char *Latitude, char *Longitude, char *Altitude, char *Speed, char *Heading, char *DriveDirection, char *StatusFlag, char debug)
{
  char Buffer[6];
  long unsigned int MonrValueU64;
  unsigned int MonrValueU32;
  unsigned short MonrValueU16;
  unsigned char MonrValueU8;
  int i,j;

  bzero(Id, SMALL_BUFFER_SIZE_1);
  bzero(Timestamp, SMALL_BUFFER_SIZE_0);
  bzero(Latitude, SMALL_BUFFER_SIZE_0);
  bzero(Longitude, SMALL_BUFFER_SIZE_0);
  bzero(Altitude, SMALL_BUFFER_SIZE_0);
  bzero(Speed, SMALL_BUFFER_SIZE_0);
  bzero(Heading, SMALL_BUFFER_SIZE_0);
  bzero(DriveDirection, SMALL_BUFFER_SIZE_1);
  bzero(StatusFlag, SMALL_BUFFER_SIZE_1);


  //Index
  sprintf(Id, "%" PRIu8, (unsigned char)Idn);

  //Timestamp
  MonrValueU64 = 0;
  j=5;
  for(i = 0; i <= 5; i++, j++) MonrValueU64 = *(MonrData+j) | (MonrValueU64 << 8);
  sprintf(Timestamp, "%" PRIu64, MonrValueU64);
  
  if(debug && MonrValueU64%200 == 0)
  {
    for(i = 5; i < 29; i ++) printf("%x-", (unsigned char)MonrData[i]);
    printf("\n");
  }

  //Latitude
  MonrValueU32 = 0;
  for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
  sprintf(Latitude, "%" PRIi32, MonrValueU32);
    
  //Longitude
  MonrValueU32 = 0;
  for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
  sprintf(Longitude, "%" PRIi32, MonrValueU32);

  //Altitude
  MonrValueU32 = 0;
  for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
  sprintf(Altitude, "%" PRIi32, MonrValueU32);
  
  //Speed
  MonrValueU16 = 0;
  for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
  sprintf(Speed, "%" PRIu16, MonrValueU16);
  
  //Heading
  MonrValueU16 = 0;
  for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
  sprintf(Heading, "%" PRIu16, MonrValueU16);
 
  //Driving direction
  MonrValueU8 = (unsigned char)*(MonrData+j);
  //printf("D: %d\n", MonrValueU8 );
  j++;
  sprintf(DriveDirection, "%" PRIu8, MonrValueU8);

  //Status flag
  MonrValueU8 = (unsigned char)*(MonrData+j);
  sprintf(StatusFlag, "%" PRIu8, MonrValueU8);


  return 0;
}

int ObjectControlSendDOPMMEssage(char* Filename, int *Socket, int RowCount, char *IP, char debug)
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

  printf("[ObjectControl] %d DOPM bytes sent to %s\n", SumMessageLength, IP);

  fclose (fd);

  return 0;
}


int ObjectControlBuildOSEMMessage(char* MessageBuffer, char *Latitude, char *Longitude, char *Altitude, char *Heading, char debug)
{


  int MessageIndex = 0;
  double Data;
  
  bzero(MessageBuffer, COMMAND_OSEM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_OSEM_CODE);
  
  //if(iUtilGetParaConfFile("OrigoLatitude",pcTempBuffer))
  {
    //printf("Latitude: %s\n", Latitude);
    //Data = (atof("57.77737160") * 1e8)/10;
    Data = (atof((char *)Latitude) * 1e8)/10;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, (unsigned int)Data);
  }
  //if(iUtilGetParaConfFile("OrigoLongitude",pcTempBuffer))
  {
    //printf("Longitude: %s\n", Longitude);
    //Data = (atof("12.7804630") * 1e8)/10;
    Data = (atof((char *)Longitude) * 1e8)/10;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned int)Data);
  }
  //if(iUtilGetParaConfFile("OrigoAltitude",pcTempBuffer))
  {
    //printf("Altitude: %s\n", Altitude);
    //Data = atof("202.934115075") * 1e2;
    Data = atof((char *)Altitude) * 1e2;
    MessageIndex = UtilAddFourBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
  }
  //if(iUtilGetParaConfFile("OrigoHeading",pcTempBuffer))
  {
    //printf("Heading: %s\n", Heading);
    //Data = atof("0.0") * 1e1;
    Data = UtilRadToDeg(atof((char *)Heading) * 1e1);
    MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, (unsigned short)Data);
  }
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]=%x\n", i, (unsigned char)MessageBuffer[i]);
  }


  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


int ObjectControlBuildSTRTMessage(char* MessageBuffer, unsigned char CommandOption, unsigned long TimeStamp, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_STRT_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_STRT_CODE);
 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, CommandOption);
  
  MessageIndex = UtilAddSixBytesMessageData(MessageBuffer, MessageIndex, TimeStamp);
  
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug) 
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


int ObjectControlBuildAROMMessage(char* MessageBuffer, unsigned char CommandOption, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_AROM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_AROM_CODE);
 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, CommandOption);
    
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug) 
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


int ObjectControlBuildHEABMessage(char* MessageBuffer, unsigned char CommandOption, char debug)
{
  int MessageIndex = 0;
  
  bzero(MessageBuffer, COMMAND_HEAB_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

  UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_HEAB_CODE);
 
  MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex+COMMAND_MESSAGE_HEADER_LENGTH, CommandOption);
    
  UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX, (unsigned int) MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

  if(debug)
  {
    int i = 0;
    for(i = 0; i < MessageIndex; i ++) printf("[%d]= %x\n", i, (unsigned char)MessageBuffer[i]);
  }

  return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
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

int ObjectControlBuildMTPSMessage(char* MessageBuffer, unsigned long SyncTimestamp, char debug)
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




static void vConnectObject(int* sockfd, const char* name, const uint32_t port)
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

  #ifdef DEBUG
    printf("INF: Connected to command socket: %s %i\n",name,port);
    fflush(stdout);
  #endif

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

static void vCreateSafetyChannel(const char* name,const uint32_t port,
  int* sockfd, struct sockaddr_in* addr)
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

  bcopy((char *) object->h_addr, 
    (char *)&addr->sin_addr.s_addr, object->h_length);
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
      result = recv(*sockfd, 
        buffer,
        length,
        0);
      
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

void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH], 
  char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
  int* nbr_objects)
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
        else if (USE_TEST_HOST == 1) (void)strcat(object_address_name[(*nbr_objects)],TESTHOST_IP);
      }

      ++(*nbr_objects);
    }
  }
  (void)closedir(traj_directory);
}
