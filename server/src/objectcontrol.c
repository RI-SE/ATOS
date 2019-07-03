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
#include "timecontrol.h"
#include "datadictionary.h"
#include "maestroTime.h"


/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/

// Macro for determining individual struct member sizes
#define member_sizeof(type, member) sizeof(((type *)0)->member)

#define LOCALHOST "127.0.0.1"

#define RECV_MESSAGE_BUFFER 6200
#define BUFFER_SIZE_3100 3100
#define OBJECT_MESS_BUFFER_SIZE 1024

#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define OBC_STATE_REPORT_PERIOD_S 1
#define OBC_STATE_REPORT_PERIOD_US 0
#define OBJECT_CONTROL_CONTROL_MODE 0
#define OBJECT_CONTROL_REPLAY_MODE 1
#define OBJECT_CONTROL_ABORT_MODE 1

#define COMMAND_MESSAGE_HEADER_LENGTH sizeof(HeaderType)
#define COMMAND_MESSAGE_FOOTER_LENGTH sizeof(FooterType)
#define COMMAND_CODE_INDEX 0
#define COMMAND_MESSAGE_LENGTH_INDEX 1

#define COMMAND_DOTM_CODE 1
#define COMMAND_DOTM_ROW_MESSAGE_LENGTH sizeof(DOTMType)
#define COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH sizeof(TRAJInfoType)
#define COMMAND_DOTM_ROWS_IN_TRANSMISSION  40
#define COMMAND_DTM_BYTES_IN_ROW  30


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
#define COMMAND_STRT_MESSAGE_LENGTH sizeof(STRTType)
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
#define COMMAND_MONR_MESSAGE_LENGTH sizeof(MONRType)

#define COMMAND_VOIL_CODE 0xA100
//#define COMMAND_VOIL_NOFV 2
#define COMMAND_VOIL_MESSAGE_LENGTH (16 * sizeof(Sim1Type) + sizeof(HeaderType) + 5)

#define COMMAND_LLCM_CODE 8
#define COMMAND_LLCM_MESSAGE_LENGTH 5

#define COMMAND_SYPM_CODE 0xA103
#define COMMAND_SYPM_MESSAGE_LENGTH sizeof(SYPMType)

#define COMMAND_MTSP_CODE 0xA104
#define COMMAND_MTSP_MESSAGE_LENGTH sizeof(MTSPType)

#define COMMAND_ACM_CODE 11  //Action Configuration Message: Server->Object, TCP
#define COMMAND_ACM_MESSAGE_LENGTH 6

#define COMMAND_EAM_CODE 12  //Execution Action Message: Server->Object, UDP
#define COMMAND_EAM_MESSAGE_LENGTH 6

#define COMMAND_TCM_CODE 13  //Trigger Configuration Message: Server->Object, TCP
#define COMMAND_TCM_MESSAGE_LENGTH 5

#define COMMAND_TOM_CODE 14  //Trigger Occurred Message: Object->Server, UDP
#define COMMAND_TOM_MESSAGE_LENGTH 8

#define ASP_MESSAGE_LENGTH sizeof(ASPType)

#define CONF_FILE_PATH  "conf/test.conf"

#define SMALL_BUFFER_SIZE_0 20
#define SMALL_BUFFER_SIZE_1 2
#define SMALL_BUFFER_SIZE_2 1
#define SMALL_BUFFER_SIZE_254 254

#define TRAJECTORY_FILE_MAX_ROWS  4096

#define LOG_BUFFER_LENGTH 128

#define USE_TEMP_LOGFILE 0
#define TEMP_LOG_FILE "log/temp.log"


typedef enum {
    COMMAND_HEARTBEAT_GO,
    COMMAND_HEARTBEAT_ABORT
} hearbeatCommand_t;


typedef enum {
    TRANSITION_RESULT_UNDEFINED,
    TRANSITION_OK,
    TRANSITION_INVALID,
    TRANSITION_MEMORY_ERROR
} StateTransitionResult;
/* Small note: syntax for declaring a function pointer is (example for a function taking an int and a float,
   returning nothing) where the function foo(int a, float b) is declared elsewhere:
      void (*fooptr)(int,float) = foo;
      fooptr(10,1.5);

   Consequently, the below typedef defines a StateTransition type as a function pointer to a function taking
   (OBCState_t, OBCState_t) as input, and returning a StateTransitionResult
*/
typedef StateTransitionResult (*StateTransition)(OBCState_t *currentState, OBCState_t requestedState);


C8 TrajBuffer[COMMAND_DOTM_ROWS_IN_TRANSMISSION*COMMAND_DOTM_ROW_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH];


/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static I32 vConnectObject(int* sockfd,const char* name,const uint32_t port, U8 *Disconnect);
static void vDisconnectObject(int* sockfd);
static I32 vCheckRemoteDisconnected(int* sockfd);

static void vCreateSafetyChannel(const char* name,const uint32_t port, int* sockfd, struct sockaddr_in* addr);
static void vCloseSafetyChannel(int* sockfd);
static void vRecvMonitor(int* sockfd, char* buffer, int length, int* recievedNewData);
I32 ObjectControlBuildOSEMMessage(C8* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, C8 *Latitude, C8 *Longitude, C8 *Altitude, U8 debug);
I32 ObjectControlBuildSTRTMessage(C8* MessageBuffer, STRTType *STRTData, TimeType *GPSTime, U32 ScenarioStartTime, U32 DelayStart, U32 *OutgoingStartTime, U8 debug);
I32 ObjectControlBuildOSTMMessage(C8* MessageBuffer, OSTMType *OSTMData, C8 CommandOption, U8 debug);
I32 ObjectControlBuildHEABMessage(C8* MessageBuffer, HEABType *HEABData, TimeType *GPSTime, U8 CCStatus, U8 debug);
int ObjectControlBuildLLCMMessage(char* MessageBuffer, unsigned short Speed, unsigned short Curvature, unsigned char Mode, char debug);
I32 ObjectControlBuildSYPMMessage(C8* MessageBuffer, SYPMType *SYPMData, U32 SyncPoint, U32 StopTime, U8 debug);
I32 ObjectControlBuildMTSPMessage(C8* MessageBuffer, MTSPType *MTSPData, U32 SyncTimestamp, U8 debug);
I32 ObjectControlBuildDOTMMessageHeader(C8* MessageBuffer, I32 RowCount, HeaderType *HeaderData, TRAJInfoType *TRAJInfoData, U8 debug);
//I32 ObjectControlBuildDOTMMessageHeader(C8* MessageBuffer, I32 RowCount, HeaderType *HeaderData, U8 debug);
I32 ObjectControlBuildDOTMMessage(C8* MessageBuffer, FILE *fd, I32 RowCount, DOTMType *DOTMData, U8 debug);
I32 ObjectControlSendDOTMMEssage(C8* Filename, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug);
int ObjectControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug);
I32 ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, I32 Idn, C8 *Id, C8 *Timestamp, C8 *XPosition, C8 *YPosition, C8 *ZPosition, C8 *LongitudinalSpeed, C8 *LateralSpeed, C8 *LongitudinalAcc, C8 *LateralAcc, C8 *Heading, C8 *DriveDirection, C8 *StatusFlag, C8 *StateFlag, C8 debug);
I32 ObjectControlBuildMONRMessage(C8 *MonrData, MONRType *MONRData, U8 debug);
int ObjectControlTOMToASCII(unsigned char *TomData, char *TriggId ,char *TriggAction, char *TriggDelay, char debug);
int ObjectControlBuildTCMMessage(char* MessageBuffer, TriggActionType *TAA, char debug);
I32 ObjectControlBuildVOILMessage(C8* MessageBuffer, VOILType *VOILData, C8* SimData, U8 debug);
I32 ObjectControlSendDTMMessage(C8 *DTMData, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug);
I32 ObjectControlBuildDTMMessage(C8 *MessageBuffer, C8 *DTMData, I32 RowCount, DOTMType *DOTMData, U8 debug);
I32 ObjectControlBuildASPMessage(C8* MessageBuffer, ASPType *ASPData, U8 debug);

static void vFindObjectsInfo(char object_traj_file[MAX_OBJECTS][MAX_FILE_PATH],
                             char object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
                             int* nbr_objects);

OBCState_t vInitializeState(OBCState_t firstState, GSDType *GSD);
inline OBCState_t vGetState(GSDType *GSD);
StateTransitionResult vSetState(OBCState_t requestedState, GSDType *GSD);
StateTransition tGetTransition(OBCState_t fromState);
StateTransitionResult tFromIdle(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromInitialized(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromConnected(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromArmed(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromRunning(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromError(OBCState_t *currentState, OBCState_t requestedState);
StateTransitionResult tFromUndefined(OBCState_t *currentState, OBCState_t requestedState);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

#define MODULE_NAME "ObjectControl"

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void objectcontrol_task(TimeType *GPSTime, GSDType *GSD, LOG_LEVEL logLevel)
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
    enum COMMAND iCommand;
    char pcRecvBuffer[RECV_MESSAGE_BUFFER];
    char pcTempBuffer[512];
    C8 MessageBuffer[BUFFER_SIZE_3100];
    int iIndex = 0, i=0;
    struct timespec sleep_time, ref_time;

    /*! Timers for reporting state over message bus */
    struct timeval currentTime, nextStateReportTime;
    const struct timeval stateReportInterval = {OBC_STATE_REPORT_PERIOD_S, OBC_STATE_REPORT_PERIOD_US};

    U8 iForceObjectToLocalhostU8 = 0;

    FILE *fd;
    char Id[SMALL_BUFFER_SIZE_0];
    char GPSWeek[SMALL_BUFFER_SIZE_0];
    char Timestamp[SMALL_BUFFER_SIZE_0];
    char XPosition[SMALL_BUFFER_SIZE_0];
    char YPosition[SMALL_BUFFER_SIZE_0];
    char ZPosition[SMALL_BUFFER_SIZE_0];
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
    C8 MiscText[SMALL_BUFFER_SIZE_0];
    U32 StartTimeU32 = 0;
    U32 OutgoingStartTimeU32 = 0;
    U32 DelayedStartU32 = 0;
    U32 CurrentTimeU32 = 0;
    U32 OldTimeU32 = 0;
    U64 TimeCap1, TimeCap2;
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
    TriggActionType TAA[MAX_TRIGG_ACTIONS];
    int TriggerActionCount = 0;
    char pcSendBuffer[MBUS_MAX_DATALEN];
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
    DOTMType DOTMData;
    TRAJInfoType TRAJInfoData;
    VOILType VOILData;
    SYPMType SYPMData;
    MTSPType MTSPData;
    GeoPosition OriginPosition;
    ASPType ASPData;
    ASPData.MTSPU32 = 0;
    ASPData.TimeToSyncPointDbl = 0;
    ASPData.PrevTimeToSyncPointDbl = 0;
    ASPData.CurrentTimeDbl = 0;
    AdaptiveSyncPoint ASP[MAX_ADAPTIVE_SYNC_POINTS];
    int SyncPointCount = 0;
    int SearchStartIndex = 0;
    double ASPMaxTimeDiffDbl = 0;
    double ASPMaxTrajDiffDbl = 0;
    double ASPFilterLevelDbl = 0;
    double ASPMaxDeltaTimeDbl = 0;
    int ASPDebugRate = 1;
    int ASPStepBackCount = 0;

    unsigned char ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING;

    OBCState_t state = vInitializeState(OBC_STATE_IDLE, GSD);
    U8 uiTimeCycle = 0;
    int ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

    C8 Buffer2[SMALL_BUFFER_SIZE_1];
    C8 LogBuffer[LOG_BUFFER_LENGTH];
    C8 VOILReceivers[SMALL_BUFFER_SIZE_254];
    C8 DTMReceivers[SMALL_BUFFER_SIZE_254];
    U32 RowCount;
    U32 DTMIpU32;
    U32 DTMLengthU32;

    U8 DisconnectU8 = 0;
    I32 iResult;

    FILE *TempFd;
    U16 MiscU16;
    I32 j = 0;
    GSD->MONRSizeU8 = 0;
    GSD->HEABSizeU8 = 0;
    //GSD->OSTMSizeU8 = 0;
    //GSD->STRTSizeU8 = 0;
    //GSD->OSEMSizeU8 = 0;
    //U8 OSEMSentU8 = 0;
    U8 STRTSentU8 = 0;


    // Create log
    LogInit(MODULE_NAME,logLevel);
    LogMessage(LOG_LEVEL_INFO, "Object control task running with PID: %i", getpid());

  
    // Set up message bus connection
    if (iCommInit())
        util_error("Unable to connect to message queue bus");
   
    // Initialize timer for sending state
    TimeSetToCurrentSystemTime(&currentTime);
    nextStateReportTime = currentTime;

    while(!iExit)
    {
        state = vGetState(GSD);

        if(state == OBC_STATE_ERROR)
        {
            ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
            MessageLength = ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, GPSTime, ObjectControlServerStatus, 0);
            UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);

        }

        if(state == OBC_STATE_RUNNING || state == OBC_STATE_ARMED || state == OBC_STATE_CONNECTED)
        {
            /*HEAB*/
            for(iIndex=0;iIndex<nbr_objects;++iIndex)
            {
                if(uiTimeCycle == 0)
                {
                    //HeartbeatMessageCounter ++;
                    MessageLength = ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, GPSTime, ObjectControlServerStatus, 0);
                    //ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                    UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                }
            }

            //Store HEAB in GSD
            //for(j = 0; j < MessageLength; j++) GSD->HEABData[j] = MessageBuffer[j];
            //GSD->HEABSizeU8 = (U8)MessageLength;

            // Check if any object has disconnected - if so, disconnect all objects and return to idle
            DisconnectU8 = 0;
            for (iIndex = 0; iIndex < nbr_objects; ++iIndex)
            {
                DisconnectU8 |= vCheckRemoteDisconnected(&socket_fd[iIndex]);
                if (DisconnectU8){
                    LogMessage(LOG_LEVEL_WARNING,"Lost connection to IP %s - returning to IDLE",object_address_name[iIndex]);

                    for (iIndex = 0; iIndex < nbr_objects; ++iIndex)
                    {
                        vDisconnectObject(&socket_fd[iIndex]);
                    }

                    /* Close safety socket */
                    for (iIndex = 0; iIndex < nbr_objects; ++iIndex)
                    {
                        vCloseSafetyChannel(&safety_socket_fd[iIndex]);
                    }
                    vSetState(OBC_STATE_IDLE, GSD);
                    break;
                }
            }
        }

        if(state == OBC_STATE_RUNNING || state == OBC_STATE_CONNECTED || state == OBC_STATE_ARMED)
        {
            char buffer[RECV_MESSAGE_BUFFER];
            int recievedNewData = 0;
            // this is etsi time lets remov it ans use utc instead
            //gettimeofday(&CurrentTimeStruct, NULL);

            CurrentTimeU32 = ((GPSTime->GPSSecondsOfWeekU32*1000 + (U32)TimeControlGetMillisecond(GPSTime)) << 2) + GPSTime->MicroSecondU16;


            /*MTSP*/
            if(HeartbeatMessageCounter == 0)
            {
                HeartbeatMessageCounter = 0;
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    for(i = 0; i < SyncPointCount; i++)
                    {
                        if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL && ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1)
                        {
                            /*Send Master time to adaptive sync point*/
                            MessageLength =ObjectControlBuildMTSPMessage(MessageBuffer, &MTSPData, ASPData.MTSPU32, 0);
                            //ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                            UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                        }
                        /*else if(TEST_SYNC_POINTS == 1 && iIndex == 1 && ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1 )
                        {
                            Send Master time to adaptive sync point
                            MessageLength =ObjectControlBuildMTSPMessage(MessageBuffer, &MTSPData, (U32)ASPData.MTSPU32, 0);
                            ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                        }*/
                    }
                }
            }


            for(iIndex=0;iIndex<nbr_objects;++iIndex)
            {
                bzero(buffer,RECV_MESSAGE_BUFFER);
                vRecvMonitor(&safety_socket_fd[iIndex],buffer, RECV_MESSAGE_BUFFER, &recievedNewData);


                if(recievedNewData)
                {
                    LogMessage(LOG_LEVEL_DEBUG,"Recieved new data from %s %d %d: %s",object_address_name[iIndex],object_udp_port[iIndex],recievedNewData,buffer);

                    if(buffer[0] == COMMAND_TOM_CODE)
                    {
                        for(i = 0; i < TriggerActionCount; i ++)
                        {
                            LOG_SEND(LogBuffer,"[ObjectControl] External trigg received. %s\n", TAA[i].TriggerIP);
                            if(strstr(TAA[i].TriggerIP, object_address_name[iIndex]) != NULL)
                            {
                                //printf("[ObjectControl] External trigg received\n");
                                fflush(stdout);
                                ObjectControlTOMToASCII(buffer, TriggId, TriggAction, TriggDelay, 1);
                                bzero(buffer, OBJECT_MESS_BUFFER_SIZE);
                                bzero(pcSendBuffer, sizeof(pcSendBuffer));
                                bzero(ObjectPort, SMALL_BUFFER_SIZE_0);
                                sprintf(ObjectPort, "%d", object_udp_port[iIndex]);
                                strcat(pcSendBuffer,object_address_name[iIndex]);strcat(pcSendBuffer,";");
                                strcat(pcSendBuffer, ObjectPort);strcat(pcSendBuffer,";");
                                strcat(pcSendBuffer,TriggId);strcat(pcSendBuffer,";");
                                strcat(pcSendBuffer,TriggAction);strcat(pcSendBuffer,";");
                                strcat(pcSendBuffer,TriggDelay);strcat(pcSendBuffer,";");
                                if(iCommSend(COMM_TOM, pcSendBuffer, strlen(pcSendBuffer)+1) < 0)
                                {
                                    LogMessage(LOG_LEVEL_ERROR, "Fatal communication fault when sending TOM command - entering error state");
                                    vSetState(OBC_STATE_ERROR, GSD);
                                    ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;

                                }
                            }
                        }
                    }

                    if (ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE)
                    {
                        // Send MONR message on new byte format
                        //LogMessage(LOG_LEVEL_INFO, "Sending raw MONR message");
                        if(iCommSend(COMM_MONR, buffer, COMMAND_MONR_MESSAGE_LENGTH) < 0)
                        {
                            LogMessage(LOG_LEVEL_ERROR,"Fatal communication fault when sending MONR command - entering error state");
                            vSetState(OBC_STATE_ERROR, GSD);
                            ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
                        }
                    }

                    ObjectControlBuildMONRMessage(buffer, &MONRData, 0);

                    //Store MONR in GSD
                    //UtilSendUDPData("ObjectControl", &ObjectControlUDPSocketfdI32, &simulator_addr, &MONRData, sizeof(MONRData), 0);

                    for(i = 0; i < (MONRData.Header.MessageLengthU32 + COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH); i++) GSD->MONRData[i] = buffer[i];
                    GSD->MONRSizeU8 = MONRData.Header.MessageLengthU32 + COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH;

                    ObjectControlMONRToASCII(&MONRData, &OriginPosition, iIndex, Id, Timestamp, XPosition, YPosition, ZPosition, LongitudinalSpeed, LateralSpeed, LongitudinalAcc, LateralAcc, Heading, DriveDirection, StatusFlag, StateFlag, 1);
                    bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
                    strcat(buffer,object_address_name[iIndex]); strcat(buffer,";");
                    strcat(buffer,"0"); strcat(buffer,";");
                    strcat(buffer,Timestamp); strcat(buffer,";");
                    strcat(buffer,XPosition); strcat(buffer,";");
                    strcat(buffer,YPosition); strcat(buffer,";");
                    strcat(buffer,ZPosition); strcat(buffer,";");
                    strcat(buffer,Heading); strcat(buffer,";");
                    strcat(buffer,LongitudinalSpeed); strcat(buffer,";");
                    strcat(buffer,LateralSpeed); strcat(buffer,";");
                    strcat(buffer,LongitudinalAcc); strcat(buffer,";");
                    strcat(buffer,LateralAcc); strcat(buffer,";");
                    strcat(buffer,DriveDirection); strcat(buffer,";"); strcat(buffer,StatusFlag); strcat(buffer,";");
                    strcat(buffer,StateFlag); strcat(buffer,";");
                    strcat(buffer,StatusFlag); strcat(buffer,";");


                    if(ASPData.MTSPU32 != 0)
                    {
                        //Add MTSP to MONR if not 0
                        bzero(MTSP, SMALL_BUFFER_SIZE_0);
                        sprintf(MTSP, "%" PRIu32, ASPData.MTSPU32);
                        strcat(buffer, MTSP); strcat(buffer,";");
                    }

                    LogMessage(LOG_LEVEL_DEBUG, "Sending MONR message: %s", buffer);

                    if(ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE)
                    {
                        // Send MONR message on old (ASCII) format
                        if(iCommSend(COMM_MONI,buffer,strlen(buffer)) < 0)
                        {
                            LogMessage(LOG_LEVEL_ERROR,"Fatal communication fault when sending MONI command - entering error state");
                            vSetState(OBC_STATE_ERROR, GSD);
                            ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
                        }
                    }



                    //Ok, let's do the ASP
                    for(i = 0; i < SyncPointCount; i++)
                    {
                        if( TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL && CurrentTimeU32 > StartTimeU32 && StartTimeU32 > 0 && ASPData.TimeToSyncPointDbl > -1
                            /*|| TEST_SYNC_POINTS == 1 && ASP[0].TestPort == object_udp_port[iIndex] && StartTimeU32 > 0 && iIndex == 0 && TimeToSyncPoint > -1*/)
                        {
                            // Use the util.c function for time here but it soent mather
                            gettimeofday(&CurrentTimeStruct, NULL); //Capture time

                            TimeCap1 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000; //Calculate initial timestamp

                            OP[iIndex].x = ((dbl)atoi(XPosition))/1000; //Set x and y on OP (ObjectPosition)
                            OP[iIndex].y = ((dbl)atoi(YPosition))/1000;

                            //OP[iIndex].OrigoDistance = sqrt(pow(OP[iIndex].x,2) + pow(OP[iIndex].y,2)); //Calculate hypotenuse

                            UtilCalcPositionDelta(OriginLatitudeDbl,OriginLongitudeDbl,atof(XPosition)/1e7,atof(YPosition)/1e7, &OP[iIndex]);

                            if(OP[iIndex].BestFoundTrajectoryIndex <= OP[iIndex].SyncIndex)
                            {
                                ASPData.CurrentTimeU32 = CurrentTimeU32;
                                ASPData.CurrentTimeDbl = (((double)CurrentTimeU32-(double)StartTimeU32)/1000);
                                SearchStartIndex = OP[iIndex].BestFoundTrajectoryIndex - ASPStepBackCount;
                                UtilFindCurrentTrajectoryPosition(&OP[iIndex], SearchStartIndex, ASPData.CurrentTimeDbl, ASPMaxTrajDiffDbl, ASPMaxTimeDiffDbl, 0);
                                ASPData.BestFoundIndexI32 = OP[iIndex].BestFoundTrajectoryIndex;

                                if(OP[iIndex].BestFoundTrajectoryIndex != TRAJ_POSITION_NOT_FOUND)
                                {
                                    ASPData.TimeToSyncPointDbl = UtilCalculateTimeToSync(&OP[iIndex]);
                                    if(ASPData.TimeToSyncPointDbl > 0)
                                    {
                                        if(ASPData.PrevTimeToSyncPointDbl != 0 && ASPFilterLevelDbl > 0)
                                        {
                                            if(ASPData.TimeToSyncPointDbl/ASPData.PrevTimeToSyncPointDbl > (1 + ASPFilterLevelDbl/100)) ASPData.TimeToSyncPointDbl = ASPData.PrevTimeToSyncPointDbl + ASPMaxDeltaTimeDbl;//TimeToSyncPoint*ASPFilterLevelDbl/500;
                                            else if(ASPData.TimeToSyncPointDbl/ASPData.PrevTimeToSyncPointDbl < (1 - ASPFilterLevelDbl/100)) ASPData.TimeToSyncPointDbl = ASPData.PrevTimeToSyncPointDbl - ASPMaxDeltaTimeDbl;//TimeToSyncPoint*ASPFilterLevelDbl/500;
                                        }
                                        ASPData.MTSPU32 = CurrentTimeU32 + (U32)(ASPData.TimeToSyncPointDbl*4000);

                                        ASPData.PrevTimeToSyncPointDbl = ASPData.TimeToSyncPointDbl;
                                        OldTimeU32 = CurrentTimeU32;
                                    }
                                    else
                                    {
                                        CurrentTimeU32 = 0;
                                        ASPData.TimeToSyncPointDbl = -1;
                                    }

                                }

                                gettimeofday(&CurrentTimeStruct, NULL);
                                TimeCap2 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;

                                ASPData.SyncPointIndexI32 = OP[iIndex].SyncIndex;
                                ASPData.IterationTimeU16 = (U16)(TimeCap2 - TimeCap1);
                                //Build ASP debug data and set to GSD
                                //bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
                                //ObjectControlBuildASPMessage(buffer, &ASPData, 0);
                                DataDictionarySetRVSSAsp(GSD, &ASPData);

                                if(atoi(Timestamp)%ASPDebugRate == 0)
                                {
                                    printf("%d, %d, %3.3f, %s, %s\n", CurrentTimeU32, StartTimeU32, ASPData.TimeToSyncPointDbl, object_address_name[iIndex], ASP[i].MasterIP);
                                    printf("TtS=%3.3f, BestIndex=%d, MTSP=%d, iIndex=%d, IterationTime=%3.0f ms\n",ASPData.TimeToSyncPointDbl, OP[iIndex].BestFoundTrajectoryIndex, ASPData.MTSPU32, iIndex, ((double)(TimeCap2)-(double)TimeCap1));
                                    printf("CurrentTime=%3.3f, x=%3.3f mm, y=%3.3f\n\n",ASPData.CurrentTimeDbl, OP[iIndex].x, OP[iIndex].y);

                                    //Build and send ASP on message queue
                                    //(void)iCommSend(COMM_ASP,buffer);
                                }
                            }
                        }
                    }

                    OP[iIndex].Speed = atof(Speed);

                }
            }
        }


        if(GSD->SupChunkSize > 0 && (state == OBC_STATE_CONNECTED || state == OBC_STATE_ARMED || state == OBC_STATE_RUNNING) )
        {

            //bzero(TrajBuffer, strlen(pcRecvBuffer));
            //DTMLengthU32 = UtilHexTextToBinary(strlen(pcRecvBuffer), pcRecvBuffer, TrajBuffer, 0);
            //UtilISOBuildTRAJInfo(GSD->SupChunk, &TRAJInfoData, 1);

            //MiscU16 = sizeof(TRAJInfoType); //72 = Number of bytes in [Ip, Id, Name, Version]
            DTMIpU32 = 0;
            DTMIpU32 = (C8)GSD->SupChunk[98];
            DTMIpU32 = (C8)GSD->SupChunk[97] | (DTMIpU32 << 8);
            DTMIpU32 = (C8)GSD->SupChunk[96] | (DTMIpU32 << 8);
            DTMIpU32 = (C8)GSD->SupChunk[95] | (DTMIpU32 << 8);
            /*DTM*/
            for(iIndex=0;iIndex<nbr_objects;++iIndex)
            {
                printf("[ObjectControl] ObjectIp = %x, DTMIp = %x\n", UtilIPStringToInt(object_address_name[iIndex]), DTMIpU32);
                if(DTMIpU32 == UtilIPStringToInt(object_address_name[iIndex]))
                {
                    //UtilSendTCPData("[ObjectControl]", TrajBuffer, DTMLengthU32, &socket_fd[iIndex], 0);
                    /*DTM Header*/
                    //MessageLength = ObjectControlBuildDOTMMessageHeader(MessageBuffer, (DTMLengthU32-sizeof(TRAJInfoType))/COMMAND_DTM_BYTES_IN_ROW , &HeaderData, &TRAJInfoData, 0);
                    /*Send DTM header*/
                    //UtilSendTCPData("[ObjectControl]", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                    /*DTM Data*/
                    //MessageLength = ObjectControlBuildDTMMessage(MessageBuffer, TrajBuffer, (DTMLengthU32-sizeof(TRAJInfoType))/COMMAND_DTM_BYTES_IN_ROW, &DOTMData, 0);
                    /*Send DTM data*/
                    //UtilSendTCPData("[ObjectControl]", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                    //printf("Send to object\n");
                    UtilSendTCPData("[ObjectControl]", GSD->SupChunk, GSD->SupChunkSize, &socket_fd[iIndex], 0);
                }
            }

            GSD->SupChunkSize = 0;

        }


        bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);
        //Have we recieved a command?
        if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER,NULL))
        {
            LogMessage(LOG_LEVEL_INFO, "Received command %d", iCommand);


            if(iCommand == COMM_ARMD && (state == OBC_STATE_CONNECTED || state == OBC_STATE_ARMED))
            {
                if(pcRecvBuffer[0] == COMMAND_OSTM_OPT_SET_ARMED_STATE)
                {
                    LOG_SEND(LogBuffer,"[ObjectControl] Sending ARM %d", pcRecvBuffer[0]);
                    vSetState(OBC_STATE_ARMED, GSD);
                }
                else if(pcRecvBuffer[0] == COMMAND_OSTM_OPT_SET_DISARMED_STATE)
                {
                    LOG_SEND(LogBuffer,"[ObjectControl] Sending DISARM: %d", pcRecvBuffer[0]);
                    vSetState(OBC_STATE_CONNECTED, GSD);
                }
                MessageLength = ObjectControlBuildOSTMMessage(MessageBuffer, &OSTMData, pcRecvBuffer[0], 0);

                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    /*Send OSTM message*/
                    UtilSendTCPData("[Object Control test]", MessageBuffer, MessageLength, &socket_fd[iIndex], 1);
                }

                //Store OSTM in GSD
                //for(i = 0; i < MessageLength; i++) GSD->OSTMData[i] = MessageBuffer[i];
                //GSD->OSTMSizeU8 = (U8)MessageLength;

                ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK; //Set server to READY
            }
            else if(iCommand == COMM_STRT && (state == OBC_STATE_ARMED) /*|| OBC_STATE_INITIALIZED)*/)  //OBC_STATE_INITIALIZED is temporary!
            {
                bzero(Timestamp, SMALL_BUFFER_SIZE_0);
                MiscPtr =strchr(pcRecvBuffer,';');
                strncpy(Timestamp, MiscPtr+1, (uint64_t)strchr(MiscPtr+1, ';') - (uint64_t)MiscPtr  - 1);
                StartTimeU32 = atol(Timestamp);
                bzero(Timestamp, SMALL_BUFFER_SIZE_0);
                MiscPtr += 1;
                MiscPtr =strchr(pcRecvBuffer,';');
                strncpy(Timestamp, MiscPtr+1, (uint64_t)strchr(MiscPtr+1, ';') - (uint64_t)MiscPtr  - 1);
                DelayedStartU32 = atoi(Timestamp);
                ASPData.MTSPU32 = 0;
                ASPData.TimeToSyncPointDbl = 0;
                SearchStartIndex = -1;
                ASPData.PrevTimeToSyncPointDbl = 0;
                OldTimeU32 = CurrentTimeU32;
                ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK; //Set server to READY
                MessageLength = ObjectControlBuildSTRTMessage(MessageBuffer, &STRTData, GPSTime, (U32)StartTimeU32, DelayedStartU32, &OutgoingStartTimeU32, 0);
                for(iIndex=0;iIndex<nbr_objects;++iIndex) {
                  UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                }
                vSetState(OBC_STATE_RUNNING, GSD);

                //Store STRT in GSD
                if(STRTSentU8 == 0)
                {
                    //for(i = 0; i < MessageLength; i++) GSD->STRTData[i] = MessageBuffer[i];
                    //GSD->STRTSizeU8 = (U8)MessageLength;
                    STRTSentU8 = 1;
                }
                //OBCState = OBC_STATE_INITIALIZED; //This is temporary!
                //printf("OutgoingStartTimeU32 = %d\n", OutgoingStartTimeU32);
                GSD->ScenarioStartTimeU32 = OutgoingStartTimeU32;
                bzero(MiscText, SMALL_BUFFER_SIZE_0);
                sprintf(MiscText, "%" PRIu32, GSD->ScenarioStartTimeU32 << 2);
                LOG_SEND(LogBuffer, "[ObjectControl] START received <%s>, GPS time <%s>\n",pcRecvBuffer, MiscText);

            }
            else if(iCommand == COMM_TRAJ && (state == OBC_STATE_CONNECTED || state == OBC_STATE_ARMED || state == OBC_STATE_RUNNING) )
            {

                DTMLengthU32 = UtilHexTextToBinary(strlen(pcRecvBuffer), pcRecvBuffer, TrajBuffer, 0);
                DTMIpU32 = (C8)TrajBuffer[0];
                DTMIpU32 = (C8)TrajBuffer[1] | (DTMIpU32 << 8);
                DTMIpU32 = (C8)TrajBuffer[2] | (DTMIpU32 << 8);
                DTMIpU32 = (C8)TrajBuffer[3] | (DTMIpU32 << 8);
                TRAJInfoData.IpAddressU32 = DTMIpU32;

                MiscU16 = (C8)TrajBuffer[4];
                MiscU16 = (C8)TrajBuffer[5] | (MiscU16 << 8);
                TRAJInfoData.TrajectoryIDU16 = MiscU16;
                i = 0;
                do
                {
                    TRAJInfoData.TrajectoryNameC8[i] = (C8)TrajBuffer[i + 6];
                    i ++;
                } while(TRAJInfoData.TrajectoryNameC8[i-1] != 0);

                MiscU16 = (C8)TrajBuffer[70];
                MiscU16 = (C8)TrajBuffer[71] | (MiscU16 << 8);

                TRAJInfoData.TrajectoryVersionU16 = MiscU16;

                MiscU16 = 72; //Number of bytes in [Ip, Id, Name, Version]

                /*DTM*/
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    printf("[ObjectControl] ObjectIp = %x, DTMIp = %x\n", UtilIPStringToInt(object_address_name[iIndex]), DTMIpU32);
                    if( DTMIpU32 == UtilIPStringToInt(object_address_name[iIndex]))
                    {
                        /*DTM Header*/
                        MessageLength = ObjectControlBuildDOTMMessageHeader(MessageBuffer, (DTMLengthU32-MiscU16)/ COMMAND_DTM_BYTES_IN_ROW , &HeaderData, &TRAJInfoData, 0);
                        /*Send DTM header*/
                        UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 1);
                        /*DTM Data*/
                        MessageLength = ObjectControlBuildDTMMessage(MessageBuffer, TrajBuffer+MiscU16, (DTMLengthU32-MiscU16)/COMMAND_DTM_BYTES_IN_ROW, &DOTMData, 0);
                        /*Send DTM data*/
                        UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 1);
                    }
                }

            }
            else if(/*iCommand == COMM_TRAJ_FROMSUP*/ GSD->SupChunkSize > 0 && (state == OBC_STATE_CONNECTED || state == OBC_STATE_ARMED || state == OBC_STATE_RUNNING) )
            {

                //bzero(TrajBuffer, strlen(pcRecvBuffer));
                //DTMLengthU32 = UtilHexTextToBinary(strlen(pcRecvBuffer), pcRecvBuffer, TrajBuffer, 0);
                UtilISOBuildTRAJInfo(GSD->SupChunk, &TRAJInfoData, 0);

                //MiscU16 = sizeof(TRAJInfoType); //72 = Number of bytes in [Ip, Id, Name, Version]
                printf("[ObjectControl] Send DTM %d\n", DTMLengthU32);
                /*DTM*/
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    printf("[ObjectControl] ObjectIp = %x, DTMIp = %x\n", UtilIPStringToInt(object_address_name[iIndex]), TRAJInfoData.IpAddressU32);
                    if(TRAJInfoData.IpAddressU32 == UtilIPStringToInt(object_address_name[iIndex]))
                    {
                        //UtilSendTCPData("[ObjectControl]", TrajBuffer, DTMLengthU32, &socket_fd[iIndex], 0);
                        /*DTM Header*/
                        //MessageLength = ObjectControlBuildDOTMMessageHeader(MessageBuffer, (DTMLengthU32-sizeof(TRAJInfoType))/COMMAND_DTM_BYTES_IN_ROW , &HeaderData, &TRAJInfoData, 0);
                        /*Send DTM header*/
                        //UtilSendTCPData("[ObjectControl]", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                        /*DTM Data*/
                        //MessageLength = ObjectControlBuildDTMMessage(MessageBuffer, TrajBuffer, (DTMLengthU32-sizeof(TRAJInfoType))/COMMAND_DTM_BYTES_IN_ROW, &DOTMData, 0);
                        /*Send DTM data*/
                        //UtilSendTCPData("[ObjectControl]", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                        UtilSendTCPData("[ObjectControl]", GSD->SupChunk, GSD->SupChunkSize, &socket_fd[iIndex], 0);
                    }
                }

                GSD->SupChunkSize = 0;

            }
            else if(iCommand == COMM_VIOP && state == OBC_STATE_RUNNING/*OBC_STATE_INITIALIZED*/)
            {
                /*Build the VOIL message*/
                MessageLength = ObjectControlBuildVOILMessage(MessageBuffer, &VOILData, (C8*)GSD->VOILData, 0);
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    /*Here we send the VOIL message, if IP-address found*/
                    if(strstr(VOILReceivers, object_address_name[iIndex]))
                    {
                        //ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                        UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
                    }
                }

                if(USE_TEMP_LOGFILE) (void)fwrite(MessageBuffer,1,MessageLength,TempFd); //Write VOIL data to file
            }
            else if(iCommand == COMM_REPLAY)
            {
                ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
                LogMessage(LOG_LEVEL_INFO,"Entering REPLAY mode <%s>", pcRecvBuffer);
            }
            else if(iCommand == COMM_ABORT && state == OBC_STATE_RUNNING)
            {
                vSetState(OBC_STATE_CONNECTED, GSD);
                ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT; //Set server to ABORT
                LogMessage(LOG_LEVEL_WARNING,"ABORT received");
                LOG_SEND(LogBuffer, "[ObjectControl] ABORT received.");
            }
            else if(iCommand == COMM_CONTROL)
            {
                ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
                printf("[ObjectControl] Object control in CONTROL mode\n");
            }
            else if(iCommand == COMM_INIT)
            {
                LogMessage(LOG_LEVEL_INFO,"INIT received");
                LOG_SEND(LogBuffer, "[ObjectControl] INIT received.");
                /* Get objects; name and drive file */
                nbr_objects = 0;
                vFindObjectsInfo(object_traj_file,object_address_name,&nbr_objects);
                DataDictionaryGetForceToLocalhostU8(GSD, &iForceObjectToLocalhostU8);

                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    if(0 == iForceObjectToLocalhostU8)
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
                        UtilSetAdaptiveSyncPoint(&ASP[i], fd, 0);
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


                vSetState(OBC_STATE_INITIALIZED, GSD);
                LogMessage(LOG_LEVEL_INFO,"ObjectControl is initialized");
                LOG_SEND(LogBuffer, "[ObjectControl] ObjectControl is initialized.");

                //Remove temporary file
                remove(TEMP_LOG_FILE);
                if(USE_TEMP_LOGFILE)
                {
                    //Create temporary file
                    TempFd = fopen (TEMP_LOG_FILE, "w+");
                }

                //OSEMSentU8 = 0;
                STRTSentU8 = 0;
            }
            else if(iCommand == COMM_CONNECT && state == OBC_STATE_INITIALIZED)
            {
                LogMessage(LOG_LEVEL_INFO,"CONNECT received");
                LOG_SEND(LogBuffer, "[ObjectControl] CONNECT received.");

                /* Connect and send drive files */
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {

                    UtilSetObjectPositionIP(&OP[iIndex], object_address_name[iIndex]);

                    MessageLength =ObjectControlBuildOSEMMessage(MessageBuffer, &OSEMData, GPSTime,
                                                                 OriginLatitude,
                                                                 OriginLongitude,
                                                                 OriginAltitude,
                                                                 0);

                    DisconnectU8 = 0;

                    do
                    {

                        iResult = vConnectObject(&socket_fd[iIndex],object_address_name[iIndex],object_tcp_port[iIndex], &DisconnectU8);

                        if ( iResult < 0)
                        {
                            switch (errno)
                            {
                            case ECONNREFUSED:
                                LogMessage(LOG_LEVEL_INFO,"Unable to connect to object %s:%d, retry in %d sec...",object_address_name[iIndex],object_tcp_port[iIndex], (!(1 & DisconnectU8))*3);
                                LOG_SEND(LogBuffer, "[ObjectControl] Was not able to connect to object, [IP: %s] [PORT: %d], retry in %d sec...",object_address_name[iIndex],object_tcp_port[iIndex], (!(1 & DisconnectU8))*3);
                                (void)sleep(3); // TODO: Move this to the rest of the sleep operations? Also, remove the hardcoded 3
                                break;
                            case EADDRINUSE:
                                util_error("[ObjectControl] Local address/port already in use");
                                break;
                            case EALREADY:
                                util_error("[ObjectControl] Previous connection attempt still in progress");
                                break;
                            case EISCONN:
                                util_error("[ObjectControl] Socket is already connected");
                                break;
                            case ENETUNREACH:
                                util_error("[ObjectControl] Network unreachable");
                                break;
                            case ETIMEDOUT:
                                util_error("[ObjectControl] Connection timed out");
                                break;
                            default:
                                util_error("ERR: Failed to connect to control socket");
                                break;
                            }

                        }

                        bzero(pcRecvBuffer,RECV_MESSAGE_BUFFER);
                        //Have we received a command?
                        if(iCommRecv(&iCommand,pcRecvBuffer,RECV_MESSAGE_BUFFER,NULL))
                        {
                            if(iCommand == COMM_DISCONNECT)
                            {
                                DisconnectU8 = 1;
                                LOG_SEND(LogBuffer, "[ObjectControl] DISCONNECT received.");
                            }

                        }

                    } while(iResult < 0 && DisconnectU8 == 0);

                    if(iResult >= 0)
                    {
                        /* Send OSEM command in mq so that we get some information like GPSweek, origin (latitude,logitude,altitude in gps coordinates)*/
                        LogMessage(LOG_LEVEL_INFO,"Sending OSEM");
                        LOG_SEND(LogBuffer, "[ObjectControl] Sending OSEM.");
                      
                        ObjectControlOSEMtoASCII(&OSEMData, GPSWeek, OriginLatitude, OriginLongitude, OriginAltitude );
                        bzero(pcSendBuffer, sizeof(pcSendBuffer));
                        strcat(pcSendBuffer,GPSWeek);strcat(pcSendBuffer,";");
                        strcat(pcSendBuffer,OriginLongitude);strcat(pcSendBuffer,";");
                        strcat(pcSendBuffer,OriginLatitude);strcat(pcSendBuffer,";");
                        strcat(pcSendBuffer,OriginAltitude);

                        if(iCommSend(COMM_OSEM,pcSendBuffer,strlen(pcSendBuffer)+1) < 0)
                        {
                            LogMessage(LOG_LEVEL_ERROR,"Fatal communication fault when sending OSEM command - entering error state");
                            vSetState(OBC_STATE_ERROR, GSD);
                            ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
                        }
                      
                        UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);

                        /*Here we send DOTM, if the IP-address not is found*/
                        if(strstr(DTMReceivers, object_address_name[iIndex]) == NULL)
                        {

                            fd = fopen (object_traj_file[iIndex], "r");

                            RowCount = UtilCountFileRows(fd);
                            //printf("RowCount: %d\n", RowCount);
                            fclose (fd);

                            /*DOTM*/
                            MessageLength = ObjectControlBuildDOTMMessageHeader(TrajBuffer, RowCount-2, &HeaderData, &TRAJInfoData, 0);

                            /*Send DOTM header*/
                            UtilSendTCPData("Object Control", TrajBuffer, MessageLength, &socket_fd[iIndex], 0);

                            /*Send DOTM data*/
                            ObjectControlSendDOTMMEssage(object_traj_file[iIndex], &socket_fd[iIndex], RowCount-2, (char *)&object_address_name[iIndex], object_tcp_port[iIndex], &DOTMData, 0);
                        }


                        /* Adaptive Sync Points object configuration start...*/
                        if(TEST_SYNC_POINTS == 1) printf("Trajfile: %s\n", object_traj_file[iIndex]);
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
                                MessageLength =ObjectControlBuildSYPMMessage(MessageBuffer, &SYPMData, ASP[i].SlaveTrajSyncTime*1000, ASP[i].SlaveSyncStopTime*1000, 1);
                                UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 1);
                            }
                            else if(TEST_SYNC_POINTS == 0 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL)
                            {
                                /*Send SYPM to slave*/
                                MessageLength =ObjectControlBuildSYPMMessage(MessageBuffer, &SYPMData, ASP[i].SlaveTrajSyncTime*1000, ASP[i].SlaveSyncStopTime*1000, 1);
                                UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 1);
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
                                UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fd[iIndex], 0);
                            }
                        }
                        /* ...end*/
                    }

                }
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    if(USE_TEST_HOST == 0) vCreateSafetyChannel(object_address_name[iIndex], object_udp_port[iIndex], &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
                    else if(USE_TEST_HOST == 1) vCreateSafetyChannel(TESTSERVER_IP, object_udp_port[iIndex], &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
                }

                uiTimeCycle = 0;

                /* Execution mode*/
                ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

                /*Set server status*/
                ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK; //Set server to READY

                if(DisconnectU8 == 0) vSetState(OBC_STATE_CONNECTED, GSD);
                else if(DisconnectU8 == 1) vSetState(OBC_STATE_IDLE, GSD);
            }
            else if(iCommand == COMM_DATA_DICT)
            {
                
                LogMessage(LOG_LEVEL_INFO,"Updating variables from DataDictionary.");
               /*
                C8 *p = pcRecvBuffer, *pi = pcRecvBuffer;
                for(i = 0; i < 3; i ++)
                {
                   p = strstr(pi,";");
                   if(i == 0) strncpy(OriginLatitude, pi, p-pi);
                   else if(i == 1) strncpy(OriginLongitude, pi, p-pi);
                   else if(i == 2) strncpy(OriginAltitude, pi, p-pi);
                   pi = p + 1;
                }
                */                
                DataDictionaryGetOriginLatitudeC8(GSD, OriginLatitude, SMALL_BUFFER_SIZE_0);
                DataDictionaryGetOriginLongitudeC8(GSD, OriginLongitude, SMALL_BUFFER_SIZE_0);
                DataDictionaryGetOriginAltitudeC8(GSD, OriginAltitude, SMALL_BUFFER_SIZE_0);
                
                DataDictionaryGetOriginLatitudeDbl(GSD, &OriginLatitudeDbl);
                DataDictionaryGetOriginLongitudeDbl(GSD, &OriginLongitudeDbl);
                DataDictionaryGetOriginAltitudeDbl(GSD, &OriginAltitudeDbl);
                
                OriginLatitudeDbl = atof(OriginLatitude);
                OriginLongitudeDbl = atof(OriginLongitude);
                OriginAltitudeDbl = atof(OriginAltitude);
                OriginHeadingDbl = atof(OriginHeading);
                OriginPosition.Latitude = OriginLatitudeDbl;
                OriginPosition.Longitude = OriginLongitudeDbl;
                OriginPosition.Altitude = OriginAltitudeDbl;
                OriginPosition.Heading = OriginHeadingDbl;

                DataDictionaryGetForceToLocalhostU8(GSD, &iForceObjectToLocalhostU8);
                LogMessage(LOG_LEVEL_INFO,"ForceObjectToLocalhost = %d", iForceObjectToLocalhostU8);
                LOG_SEND(LogBuffer, "[ObjectControl] ForceObjectToLocalhost = %d", iForceObjectToLocalhostU8);

                DataDictionaryGetASPMaxTimeDiffDbl(GSD, &ASPMaxTimeDiffDbl);
                DataDictionaryGetASPMaxTrajDiffDbl(GSD, &ASPMaxTrajDiffDbl);
                DataDictionaryGetASPStepBackCountU32(GSD, &ASPStepBackCount);
                DataDictionaryGetASPFilterLevelDbl(GSD, &ASPFilterLevelDbl);
                DataDictionaryGetASPMaxDeltaTimeDbl(GSD, &ASPMaxDeltaTimeDbl);
                ASPDebugRate = 1;
                DataDictionaryGetVOILReceiversC8(GSD, VOILReceivers, SMALL_BUFFER_SIZE_254);
                DataDictionaryGetDTMReceiversC8(GSD, DTMReceivers, SMALL_BUFFER_SIZE_254);

            }
            else if(iCommand == COMM_DISCONNECT)
            {
                //#ifndef NOTCP
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    vDisconnectObject(&socket_fd[iIndex]);
                }
                //#endif //NOTCP

                LogMessage(LOG_LEVEL_INFO,"DISCONNECT received");
                LOG_SEND(LogBuffer, "[ObjectControl] DISCONNECT received.");
                /* Close safety socket */
                for(iIndex=0;iIndex<nbr_objects;++iIndex)
                {
                    vCloseSafetyChannel(&safety_socket_fd[iIndex]);
                }
                vSetState(OBC_STATE_IDLE, GSD);
            }
            else if(iCommand == COMM_EXIT)
            {
                iExit = 1;
                iCommClose();
            }
            else
            {
                LogMessage(LOG_LEVEL_WARNING,"Unhandled command in object control: %d",iCommand);
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

            // Periodically send state to signal aliveness
            TimeSetToCurrentSystemTime(&currentTime);
            if (timercmp(&currentTime, &nextStateReportTime, >))
            {
                timeradd(&nextStateReportTime, &stateReportInterval,
                         &nextStateReportTime);

                state = vGetState(GSD);
                bzero(Buffer2, sizeof(Buffer2));
                Buffer2[0] = (uint8_t)(DataDictionaryGetOBCStateU8(GSD));
                if (iCommSend(COMM_OBC_STATE, Buffer2, sizeof(Buffer2)) < 0)
                {
                    LogMessage(LOG_LEVEL_ERROR,"Fatal communication fault when sending OBC_STATE command - entering error state");
                    vSetState(OBC_STATE_ERROR, GSD);
                    ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
                }
            }

            (void)nanosleep(&sleep_time,&ref_time);
        }

    }

    LogMessage(LOG_LEVEL_INFO,"Object control exiting");
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/


I32 ObjectControlBuildVOILMessage(C8* MessageBuffer, VOILType *VOILData, C8* SimData, U8 debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;
    U16 U16Data = 0;
    I16 I16Data = 0;
    U32 U32Data = 0;
    I32 I32Data = 0;

    if(debug)
    {
        printf("Length: %d\n", *(SimData+3));
        for(i= 0; i < *(SimData+3)+4; i++) printf("%x-", *(SimData+i));
        printf("\n");
    }

    U16Data = (U16Data | *(SimData+5)) << 8;
    U16Data = (U16Data | *(SimData+4)) ;
    U16 MessageId = U16Data;
    //printf("MessageId = %x\n", MessageId);

    U32Data = (U32Data | *(SimData+6)) << 8;
    U32Data = (U32Data | *(SimData+7)) << 8;
    U32Data = (U32Data | *(SimData+8)) << 8;
    U32Data = (U32Data | *(SimData+9));
    U32 GPSSOW = U32Data;
    //printf("GPSSOW = %x\n", GPSSOW);
    U8 DynamicWorldState = *(SimData+10);
    U8 ObjectCount = *(SimData+11);
    //printf("ObjectCount = %d\n", ObjectCount);

    U8 ObjectId = *(SimData+12);
    U8 ObjectState = *(SimData+13);

    I32Data = (I32Data | *(SimData+14)) << 8;
    I32Data = (I32Data | *(SimData+15)) << 8;
    I32Data = (I32Data | *(SimData+16)) << 8;
    I32Data = (I32Data | *(SimData+17));
    I32 XPosition = I32Data;

    I32Data = 0;
    I32Data = (I32Data | *(SimData+18)) << 8;
    I32Data = (I32Data | *(SimData+19)) << 8;
    I32Data = (I32Data | *(SimData+20)) << 8;
    I32Data = (I32Data | *(SimData+21));
    I32 YPosition = I32Data;

    I32Data = 0;
    I32Data = (I32Data | *(SimData+22)) << 8;
    I32Data = (I32Data | *(SimData+23)) << 8;
    I32Data = (I32Data | *(SimData+24)) << 8;
    I32Data = (I32Data | *(SimData+25));
    I32 ZPosition = I32Data;

    U16Data = 0;
    U16Data = (U16Data | *(SimData+26)) << 8;
    U16Data = (U16Data | *(SimData+27));
    U16 Heading = U16Data;

    U16Data = 0;
    U16Data = (U16Data | *(SimData+28)) << 8;
    U16Data = (U16Data | *(SimData+29));
    U16 Pitch = U16Data;
    U16Data = 0;
    U16Data = (U16Data | *(SimData+30)) << 8;
    U16Data = (U16Data | *(SimData+31));
    U16 Roll = U16Data;

    //printf("Roll = %d\n", Roll);
    I16Data = 0;
    I16Data = (I16Data | *(SimData+32)) << 8;
    I16Data = (I16Data | *(SimData+33));
    I16 Speed = I16Data;
    //printf("Speed = %d\n", Speed);


    bzero(MessageBuffer, ObjectCount*sizeof(Sim1Type) + 6 + COMMAND_MESSAGE_FOOTER_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);


    VOILData->Header.SyncWordU16 = SYNC_WORD;
    VOILData->Header.TransmitterIdU8 = 0;
    VOILData->Header.MessageCounterU8 = 0;
    VOILData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    VOILData->Header.MessageIdU16 = COMMAND_VOIL_CODE;
    VOILData->Header.MessageLengthU32 = ObjectCount*sizeof(Sim1Type) + 6 - COMMAND_MESSAGE_HEADER_LENGTH;
    VOILData->GPSSOWU32 = GPSSOW;
    VOILData->WorldStateU8 = DynamicWorldState;
    VOILData->ObjectCountU8 = ObjectCount;
    VOILData->SimObjects[0].ObjectIdU8 = ObjectId;
    VOILData->SimObjects[0].ObjectStateU8 = ObjectState;
    VOILData->SimObjects[0].XPositionI32 = XPosition;
    VOILData->SimObjects[0].YPositionI32 = YPosition;
    VOILData->SimObjects[0].ZPositionI32 = ZPosition;
    VOILData->SimObjects[0].HeadingU16 = Heading;
    VOILData->SimObjects[0].PitchU16 = Pitch;
    VOILData->SimObjects[0].RollU16 = Roll;
    VOILData->SimObjects[0].SpeedI16 = Speed;


    p=(C8 *)VOILData;
    for(i=0; i < ObjectCount*sizeof(Sim1Type) + 6 + COMMAND_MESSAGE_HEADER_LENGTH; i++) *(MessageBuffer + i) = *p++;
    //Crc = crc_16((const C8*)MessageBuffer, sizeof(VOILData));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc);
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    MessageIndex = i;

    if(debug)
    {
        // TODO: use byte printout from logging when it has been implemented
        printf("VOILData total length = %d bytes (header+message+footer)\n", (int)(ObjectCount*sizeof(Sim1Type) + 6 + COMMAND_MESSAGE_FOOTER_LENGTH +COMMAND_MESSAGE_HEADER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(Sim1Type)*ObjectCount + 6 + COMMAND_MESSAGE_HEADER_LENGTH; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }


    return ObjectCount*sizeof(Sim1Type) + 6 + COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH; //Total number of bytes

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
        LogMessage(LOG_LEVEL_DEBUG,"TCM:");
        for(i = 0; i < MessageIndex; i ++) LogMessage(LOG_LEVEL_DEBUG,"[%d]= %x", i, (unsigned char)MessageBuffer[i]);
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
        LogMessage(LOG_LEVEL_DEBUG,"ACM:");
        for(i = 0; i < MessageIndex; i ++) LogMessage(LOG_LEVEL_DEBUG,"[%d]= %x", i, (unsigned char)MessageBuffer[i]);
    }

    return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}


I32 ObjectControlBuildMONRMessage(C8 *MonrData, MONRType *MONRData, U8 debug)
{
    I32 MessageIndex = 0, i = 0;
    dbl Data;
    U16 Crc = 0, U16Data = 0;
    I16 I16Data = 0;
    U32 U32Data = 0;
    I32 I32Data = 0;
    U64 U64Data = 0;
    C8 *p;
    U16Data = (U16Data | *(MonrData+1)) << 8;
    U16Data = U16Data | *MonrData;

    MONRData->Header.SyncWordU16 = U16Data;
    MONRData->Header.TransmitterIdU8 = *(MonrData+2);
    MONRData->Header.MessageCounterU8 = *(MonrData+3);
    MONRData->Header.AckReqProtVerU8 = *(MonrData+4);

    U16Data = 0;
    U16Data = (U16Data | *(MonrData+6)) << 8;
    U16Data = U16Data | *(MonrData+5);
    MONRData->Header.MessageIdU16 = U16Data;

    U32Data = (U32Data | *(MonrData+10)) << 8;
    U32Data = (U32Data | *(MonrData+9)) << 8;
    U32Data = (U32Data | *(MonrData+8)) << 8;
    U32Data = U32Data | *(MonrData+7);
    MONRData->Header.MessageLengthU32 = U32Data;

    U32Data = 0;
    U32Data = (U32Data | *(MonrData+14)) << 8;
    U32Data = (U32Data | *(MonrData+13)) << 8;
    U32Data = (U32Data | *(MonrData+12)) << 8;
    U32Data = U32Data | *(MonrData+11);
    MONRData->GPSSOWU32 = U32Data;

    I32Data = 0;
    I32Data = (I32Data | *(MonrData+18)) << 8;
    I32Data = (I32Data | *(MonrData+17)) << 8;
    I32Data = (I32Data | *(MonrData+16)) << 8;
    I32Data = I32Data | *(MonrData+15);
    MONRData->XPositionI32 = I32Data;


    I32Data = 0;
    I32Data = (I32Data | *(MonrData+22)) << 8;
    I32Data = (I32Data | *(MonrData+21)) << 8;
    I32Data = (I32Data | *(MonrData+20)) << 8;
    I32Data = I32Data | *(MonrData+19);
    MONRData->YPositionI32 = I32Data;


    I32Data = 0;
    I32Data = (I32Data | *(MonrData+26)) << 8;
    I32Data = (I32Data | *(MonrData+25)) << 8;
    I32Data = (I32Data | *(MonrData+24)) << 8;
    I32Data = I32Data | *(MonrData+23);
    MONRData->ZPositionI32 = I32Data;

    U16Data = 0;
    U16Data = (U16Data | *(MonrData+28)) << 8;
    U16Data = U16Data | *(MonrData+27);
    MONRData->HeadingU16 = U16Data;

    I16Data = 0;
    I16Data = (I16Data | *(MonrData+30)) << 8;
    I16Data = I16Data | *(MonrData+29);
    MONRData->LongitudinalSpeedI16 = I16Data;

    I16Data = 0;
    I16Data = (I16Data | *(MonrData+32)) << 8;
    I16Data = I16Data | *(MonrData+31);
    MONRData->LateralSpeedI16 = I16Data;

    I16Data = 0;
    I16Data = (I16Data | *(MonrData+34)) << 8;
    I16Data = I16Data | *(MonrData+33);
    MONRData->LongitudinalAccI16 = I16Data;

    I16Data = 0;
    I16Data = (I16Data | *(MonrData+36)) << 8;
    I16Data = I16Data | *(MonrData+35);
    MONRData->LateralAccI16 = I16Data;

    MONRData->DriveDirectionU8 = *(MonrData+37);
    MONRData->StateU8 = *(MonrData+38);
    MONRData->ReadyToArmU8 = *(MonrData+39);
    MONRData->ErrorStatusU8 = *(MonrData+40);



    if(debug == 1)
    {
        LogPrint("MONR:");
        LogPrint("SyncWord = %d", MONRData->Header.SyncWordU16);
        LogPrint("TransmitterId = %d", MONRData->Header.TransmitterIdU8);
        LogPrint("PackageCounter = %d", MONRData->Header.MessageCounterU8);
        LogPrint("AckReq = %d", MONRData->Header.AckReqProtVerU8);
        LogPrint("MessageLength = %d", MONRData->Header.MessageLengthU32);
        LogPrint("GPSSOW = %u",MONRData->GPSSOWU32);
    }

    return 0;
}


I32 ObjectControlMONRToASCII(MONRType *MONRData, GeoPosition *OriginPosition, I32 Idn, C8 *Id, C8 *Timestamp, C8 *XPosition, C8 *YPosition, C8 *ZPosition, C8 *LongitudinalSpeed, C8 *LateralSpeed, C8 *LongitudinalAcc, C8 *LateralAcc, C8 *Heading, C8 *DriveDirection, C8 *StatusFlag, C8 *StateFlag, C8 debug)
{
    char Buffer[6];
    long unsigned int MonrValueU64;
    unsigned int MonrValueU32;
    unsigned short MonrValueU16;
    unsigned char MonrValueU8;
    double iLlh[3] = {0, 0, 0};
    double xyz[3] = {0, 0, 0};
    double Llh[3] = {0, 0, 0};
    uint64_t ConvertGPStoUTC;

    bzero(Id, SMALL_BUFFER_SIZE_1);
    bzero(Timestamp, SMALL_BUFFER_SIZE_0);
    bzero(XPosition, SMALL_BUFFER_SIZE_0);
    bzero(YPosition, SMALL_BUFFER_SIZE_0);
    bzero(ZPosition, SMALL_BUFFER_SIZE_0);
    bzero(LongitudinalSpeed, SMALL_BUFFER_SIZE_0);
    bzero(LateralSpeed, SMALL_BUFFER_SIZE_0);
    bzero(LongitudinalAcc, SMALL_BUFFER_SIZE_0);
    bzero(LateralAcc, SMALL_BUFFER_SIZE_0);
    bzero(Heading, SMALL_BUFFER_SIZE_0);
    bzero(DriveDirection, SMALL_BUFFER_SIZE_1);
    bzero(StatusFlag, SMALL_BUFFER_SIZE_1);
    bzero(StateFlag, SMALL_BUFFER_SIZE_1);


    if(MONRData->Header.MessageIdU16 == COMMAND_MONR_CODE)
    {
        //Index
        sprintf(Id, "%" PRIu8, (C8)Idn);

        //Timestamp
        MonrValueU64 = 0;
        //for(i = 0; i <= 5; i++, j++) MonrValueU64 = *(MonrData+j) | (MonrValueU64 << 8);
        ConvertGPStoUTC =
        sprintf(Timestamp, "%" PRIu32, MONRData->GPSSOWU32);

        if(debug && MONRData->GPSSOWU32%400 == 0)
        {
            LogMessage(LOG_LEVEL_DEBUG,"MONR = %x-%x-%x-%x-%x-%x-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d",
                        MONRData->Header.MessageIdU16,
                        MONRData->Header.SyncWordU16,
                        MONRData->Header.TransmitterIdU8,
                        MONRData->Header.MessageCounterU8,
                        MONRData->Header.AckReqProtVerU8,
                        MONRData->Header.MessageLengthU32,
                        MONRData->GPSSOWU32,
                        MONRData->XPositionI32,
                        MONRData->YPositionI32,
                        MONRData->ZPositionI32,
                        MONRData->LongitudinalSpeedI16,
                        MONRData->HeadingU16,
                        MONRData->DriveDirectionU8,
                        MONRData->StateU8,
                        MONRData->ReadyToArmU8,
                        MONRData->ErrorStatusU8);
        }

        iLlh[0] = OriginPosition->Latitude;
        iLlh[1] = OriginPosition->Longitude;
        iLlh[2] = OriginPosition->Altitude;

        xyz[0] = ((dbl)MONRData->XPositionI32)/1000;
        xyz[1] = ((dbl)MONRData->YPositionI32)/1000;
        xyz[2] = ((dbl)MONRData->ZPositionI32)/1000;

        enuToLlh(iLlh, xyz, Llh);

        //XPosition
        //MonrValueU32 = 0;
        //for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
        //sprintf(Latitude, "%" PRIi32, (I32)(Llh[0]*1e7));
        sprintf(XPosition, "%" PRIi32, MONRData->XPositionI32);

        //YPosition
        //MonrValueU32 = 0;
        //for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
        //sprintf(Longitude, "%" PRIi32, (I32)(Llh[1]*1e7));
        sprintf(YPosition, "%" PRIi32, MONRData->YPositionI32);

        //ZPosition
        //MonrValueU32 = 0;
        //for(i = 0; i <= 3; i++, j++) MonrValueU32 = *(MonrData+j) | (MonrValueU32 << 8);
        //sprintf(Altitude, "%" PRIi32, (I32)(Llh[2]));
        sprintf(ZPosition, "%" PRIi32, MONRData->ZPositionI32);

        //Speed
        //MonrValueU16 = 0;
        //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(LongitudinalSpeed, "%" PRIi16, MONRData->LongitudinalSpeedI16);

        //LatSpeed
        //MonrValueU16 = 0;
        //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(LateralSpeed, "%" PRIi16, MONRData->LateralSpeedI16);

        //LongAcc
        //MonrValueU16 = 0;
        //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(LongitudinalAcc, "%" PRIi16, MONRData->LongitudinalAccI16);

        //LatAcc
        //MonrValueU16 = 0;
        //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(LateralAcc, "%" PRIi16, MONRData->LateralAccI16);

        //Heading
        //MonrValueU16 = 0;
        //for(i = 0; i <= 1; i++, j++) MonrValueU16 = *(MonrData+j) | (MonrValueU16 << 8);
        sprintf(Heading, "%" PRIu16, MONRData->HeadingU16);

        //Driving direction
        //MonrValueU8 = (unsigned char)*(MonrData+j);
        //printf("D: %d\n", MonrValueU8 );

        sprintf(DriveDirection, "%" PRIu8, MONRData->DriveDirectionU8);

        //State
        //MonrValueU8 = (unsigned char)*(MonrData+j);
        sprintf(StatusFlag, "%" PRIu8, MONRData->StateU8);

        //ReadyToArmU8
        //MonrValueU8 = (unsigned char)*(MonrData+j);
        sprintf(StateFlag, "%" PRIu8, MONRData->ReadyToArmU8);

        //ErrorStatusU8
        //MonrValueU8 = (unsigned char)*(MonrData+j);
        sprintf(StateFlag, "%" PRIu8, MONRData->ErrorStatusU8);

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


I32 ObjectControlBuildOSEMMessage(C8* MessageBuffer, OSEMType *OSEMData, TimeType *GPSTime, C8 *Latitude, C8 *Longitude, C8 *Altitude, U8 debug)
{
    I32 MessageIndex = 0, i = 0;
    dbl Data;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_OSEM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    OSEMData->Header.SyncWordU16 = SYNC_WORD;
    OSEMData->Header.TransmitterIdU8 = 0;
    OSEMData->Header.MessageCounterU8 = 0;
    OSEMData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    OSEMData->Header.MessageIdU16 = COMMAND_OSEM_CODE;
    OSEMData->Header.MessageLengthU32 = sizeof(OSEMType) - sizeof(HeaderType) - 4;
    OSEMData->LatitudeValueIdU16 = VALUE_ID_LATITUDE;
    OSEMData->LatitudeContentLengthU16 = 6;
    OSEMData->LatitudeI64 = (I64) ((atof((char *)Latitude) * 1e10));
    OSEMData->LongitudeValueIdU16 = VALUE_ID_LONGITUDE;
    OSEMData->LongitudeContentLengthU16 = 6;
    OSEMData->LongitudeI64 = (I64)((atof((char *)Longitude) * 1e10));
    OSEMData->AltitudeValueIdU16 = VALUE_ID_ALTITUDE;
    OSEMData->AltitudeContentLengthU16 = 4;
    OSEMData->AltitudeI32 = (I32)(atof((char *)Altitude) * 1e2);
    OSEMData->DateValueIdU16 = VALUE_ID_DATE_ISO8601;
    OSEMData->DateContentLengthU16 = 4;
    OSEMData->DateU32 = (U32)GPSTime->YearU16*10000+(U32)GPSTime->MonthU8*1000+(U32)GPSTime->DayU8;
    OSEMData->GPSWeekValueIdU16 = VALUE_ID_GPS_WEEK;
    OSEMData->GPSWeekContentLengthU16 = 2;
    OSEMData->GPSWeekU16 = GPSTime->GPSWeekU16;
    OSEMData->GPSSOWValueIdU16 = VALUE_ID_GPS_SECOND_OF_WEEK;
    OSEMData->GPSSOWContentLengthU16 = 4;
    OSEMData->GPSSOWU32 = ((GPSTime->GPSSecondsOfWeekU32*1000 + GPSTime->MillisecondU16) << 2) + GPSTime->MicroSecondU16;
    OSEMData->MaxWayDeviationValueIdU16 = VALUE_ID_MAX_WAY_DEVIATION;
    OSEMData->MaxWayDeviationContentLengthU16 = 2;
    OSEMData->MaxWayDeviationU16 = 65535;
    OSEMData->MaxLateralDeviationValueIdU16 = VALUE_ID_MAX_LATERAL_DEVIATION;
    OSEMData->MaxLateralDeviationContentLengthU16 = 2;
    OSEMData->MaxLateralDeviationU16 = 65535;
    OSEMData->MinPosAccuracyContentLengthU16 = 2;
    OSEMData->MinPosAccuracyValueIdU16 = VALUE_ID_MIN_POS_ACCURACY;
    OSEMData->MinPosAccuracyU16 = 65535;

    if (!GPSTime->isGPSenabled)
    {
        OSEMData->DateU32 = UtilgetIntDateFromMS(UtilgetCurrentUTCtimeMS());
        UtilgetCurrentGPStime(&OSEMData->GPSWeekU16,&OSEMData->GPSSOWU32);
    }

    p=(C8 *)OSEMData;
    for(i=0; i<21; i++) *(MessageBuffer + i) = *p++;
    *p++; *p++;
    for(; i<31; i++) *(MessageBuffer + i) = *p++;
    *p++; *p++;
    for(; i<sizeof(OSEMType)-4; i++) *(MessageBuffer + i) = *p++;

    Crc = crc_16((const C8*)MessageBuffer, sizeof(OSEMType)-4);
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc);
    *(MessageBuffer + i++) = (U8)(Crc >> 8);

    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
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

int ObjectControlOSEMtoASCII(OSEMType *OSEMData, char *GPSWeek, char *GPSLatitude, char *GPSLongitude, char *GPSAltitude)
{
    // what do i want? in my mq? gps week, origin in lat and long coordinates
    bzero(GPSWeek,SMALL_BUFFER_SIZE_0);
    bzero(GPSLatitude,SMALL_BUFFER_SIZE_0);
    bzero(GPSLongitude,SMALL_BUFFER_SIZE_0);
    bzero(GPSAltitude,SMALL_BUFFER_SIZE_0);

    if (OSEMData->Header.MessageIdU16 == COMMAND_OSEM_CODE)
    {
        sprintf(GPSWeek,"%" PRIu16 ,OSEMData->GPSWeekU16);

        sprintf(GPSLatitude,"%" PRIi64, OSEMData->LongitudeI64);

        sprintf(GPSLongitude,"%" PRIi64,OSEMData->LatitudeI64);

        sprintf(GPSAltitude,"%" PRIi32, OSEMData->AltitudeI32);
    }
    return 0;
}
int ObjectControlBuildSTRTMessage(C8* MessageBuffer, STRTType *STRTData, TimeType *GPSTime, U32 ScenarioStartTime, U32 DelayStart, U32 *OutgoingStartTime, U8 debug)
{
    I32 MessageIndex = 0, i = 0;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_STRT_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    STRTData->Header.SyncWordU16 = SYNC_WORD;
    STRTData->Header.TransmitterIdU8 = 0;
    STRTData->Header.MessageCounterU8 = 0;
    STRTData->Header.AckReqProtVerU8 = 0;
    STRTData->Header.MessageIdU16 = COMMAND_STRT_CODE;
    STRTData->Header.MessageLengthU32 = sizeof(STRTType) - sizeof(HeaderType);
    STRTData->StartTimeValueIdU16 = VALUE_ID_GPS_SECOND_OF_WEEK;
    STRTData->StartTimeContentLengthU16 = 4;
    STRTData->StartTimeU32 = ((GPSTime->GPSSecondsOfWeekU32*1000 + (U32)TimeControlGetMillisecond(GPSTime) + ScenarioStartTime) << 2) + GPSTime->MicroSecondU16;
    STRTData->DelayStartValueIdU16 = VALUE_ID_RELATIVE_TIME;
    STRTData->DelayStartContentLengthU16 = 4;
    STRTData->DelayStartU32 = DelayStart;
    *OutgoingStartTime = (STRTData->StartTimeU32) >> 2;

    if(!GPSTime->isGPSenabled)
    {
        UtilgetCurrentGPStime(NULL,&STRTData->StartTimeU32);
    }

    p=(char *)STRTData;
    for(i=0; i<sizeof(STRTType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const unsigned char *)MessageBuffer, sizeof(STRTType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc);
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("STRT total length = %d bytes (header+message+footer)\n", (int)(COMMAND_STRT_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(STRTType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}


I32 ObjectControlBuildOSTMMessage(C8* MessageBuffer, OSTMType *OSTMData, C8 CommandOption, U8 debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_OSTM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    OSTMData->Header.SyncWordU16 = SYNC_WORD;
    OSTMData->Header.TransmitterIdU8 = 0;
    OSTMData->Header.MessageCounterU8 = 0;
    OSTMData->Header.AckReqProtVerU8 = 0;
    OSTMData->Header.MessageIdU16 = COMMAND_OSTM_CODE;
    OSTMData->Header.MessageLengthU32 = sizeof(OSTMType) - sizeof(HeaderType);
    OSTMData->StateValueIdU16 = VALUE_ID_STATE_CHANGE_REQUEST;
    OSTMData->StateContentLengthU16 = 1;
    OSTMData->StateU8 = (U8)CommandOption;

    p=(C8 *)OSTMData;
    for(i=0; i<sizeof(OSTMType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8 *)MessageBuffer, sizeof(OSTMType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    *(MessageBuffer + i++) = (U8)(Crc);
    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("OSTM total length = %d bytes (header+message+footer)\n", (int)(COMMAND_OSTM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(OSTMType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}


I32 ObjectControlBuildHEABMessage(C8* MessageBuffer, HEABType *HEABData, TimeType *GPSTime, U8 CCStatus, U8 debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_HEAB_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    HEABData->Header.SyncWordU16 = SYNC_WORD;
    HEABData->Header.TransmitterIdU8 = 0;
    HEABData->Header.MessageCounterU8 = 0;
    HEABData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    HEABData->Header.MessageIdU16 = COMMAND_HEAB_CODE;
    HEABData->Header.MessageLengthU32 = sizeof(HEABType) - sizeof(HeaderType);
    //HEABData->HeabStructValueIdU16 = 0;
    //HEABData->HeabStructContentLengthU16 = sizeof(HEABType) - sizeof(HeaderType) - 4;
    HEABData->GPSSOWU32 = ((GPSTime->GPSSecondsOfWeekU32*1000 + (U32)TimeControlGetMillisecond(GPSTime)) << 2) + GPSTime->MicroSecondU16;
    HEABData->CCStatusU8 = CCStatus;

    if(!GPSTime->isGPSenabled){
        UtilgetCurrentGPStime(NULL,&HEABData->GPSSOWU32);
    }

    p=(C8 *)HEABData;
    for(i=0; i<sizeof(HEABType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8*)MessageBuffer, sizeof(HEABType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc);
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("HEAB total length = %d bytes (header+message+footer)\n", (int)(COMMAND_HEAB_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(HEABType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
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
        LogMessage(LOG_LEVEL_DEBUG,"LLCM:");
        for(i = 0; i < MessageIndex; i ++) LogMessage(LOG_LEVEL_DEBUG,"[%d]= %x", i, (unsigned char)MessageBuffer[i]);
    }

    return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}

I32 ObjectControlBuildSYPMMessage(C8* MessageBuffer, SYPMType *SYPMData, U32 SyncPoint, U32 StopTime, U8 debug)
{

    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_SYPM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    SYPMData->Header.SyncWordU16 = SYNC_WORD;
    SYPMData->Header.TransmitterIdU8 = 0;
    SYPMData->Header.MessageCounterU8 = 0;
    SYPMData->Header.AckReqProtVerU8 = 0;
    SYPMData->Header.MessageIdU16 = COMMAND_SYPM_CODE;
    SYPMData->Header.MessageLengthU32 = sizeof(SYPMType) - sizeof(HeaderType);
    SYPMData->SyncPointTimeValueIdU16 = 1;
    SYPMData->SyncPointTimeContentLengthU16 = 4;
    SYPMData->SyncPointTimeU32 = SyncPoint;
    SYPMData->FreezeTimeValueIdU16 = 2;
    SYPMData->FreezeTimeContentLengthU16 = 4;
    SYPMData->FreezeTimeU32 = StopTime;


    p=(C8 *)SYPMData;
    for(i=0; i<sizeof(SYPMType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8 *)MessageBuffer, sizeof(SYPMType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    *(MessageBuffer + i++) = (U8)(Crc);
    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("SYPM total length = %d bytes (header+message+footer)\n", (int)(COMMAND_SYPM_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(SYPMType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}

I32 ObjectControlBuildMTSPMessage(C8* MessageBuffer, MTSPType *MTSPData, U32 SyncTimestamp, U8 debug)
{

    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_MTSP_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH);

    MTSPData->Header.SyncWordU16 = SYNC_WORD;
    MTSPData->Header.TransmitterIdU8 = 0;
    MTSPData->Header.MessageCounterU8 = 0;
    MTSPData->Header.AckReqProtVerU8 = 0;
    MTSPData->Header.MessageIdU16 = COMMAND_MTSP_CODE;
    MTSPData->Header.MessageLengthU32 = sizeof(MTSPType) - sizeof(HeaderType);
    MTSPData->EstSyncPointTimeValueIdU16 = 1;
    MTSPData->EstSyncPointTimeContentLengthU16 = 4;
    MTSPData->EstSyncPointTimeU32 = SyncTimestamp;


    p=(C8 *)MTSPData;
    for(i=0; i<sizeof(MTSPType); i++) *(MessageBuffer + i) = *p++;
    Crc = crc_16((const C8 *)MessageBuffer, sizeof(MTSPType));
    Crc = 0;
    *(MessageBuffer + i++) = (U8)(Crc >> 8);
    *(MessageBuffer + i++) = (U8)(Crc);
    MessageIndex = i;

    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("MTSPData total length = %d bytes (header+message+footer)\n", (int)(COMMAND_MTSP_MESSAGE_LENGTH+COMMAND_MESSAGE_FOOTER_LENGTH));
        printf("----HEADER----\n");
        for(i = 0;i < sizeof(HeaderType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----MESSAGE----\n");
        for(;i < sizeof(MTSPType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n----FOOTER----\n");
        for(;i < MessageIndex; i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}


I32 ObjectControlBuildDOTMMessageHeader(C8* MessageBuffer, I32 RowCount, HeaderType *HeaderData, TRAJInfoType *TRAJInfoData, U8 debug)
{
    I32 MessageIndex = 0, i;
    U16 Crc = 0;
    C8 *p;

    bzero(MessageBuffer, COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH);

    HeaderData->SyncWordU16 = SYNC_WORD;
    HeaderData->TransmitterIdU8 = 0;
    HeaderData->MessageCounterU8 = 0;
    HeaderData->AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
    HeaderData->MessageIdU16 = COMMAND_DOTM_CODE;
    HeaderData->MessageLengthU32 = COMMAND_DOTM_ROW_MESSAGE_LENGTH*RowCount + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH;

    p=(C8 *)HeaderData;
    for(i=0; i< COMMAND_MESSAGE_HEADER_LENGTH; i++) *(MessageBuffer + i) = *p++;


    TRAJInfoData->TrajectoryIDValueIdU16 = VALUE_ID_TRAJECTORY_ID;
    TRAJInfoData->TrajectoryIDContentLengthU16 = 2;

    TRAJInfoData->TrajectoryNameValueIdU16 = VALUE_ID_TRAJECTORY_NAME;
    TRAJInfoData->TrajectoryNameContentLengthU16 = 64;

    TRAJInfoData->TrajectoryVersionValueIdU16 = VALUE_ID_TRAJECTORY_VERSION;
    TRAJInfoData->TrajectoryVersionContentLengthU16 = 2;

    TRAJInfoData->IpAddressValueIdU16 = 0xA000;
    TRAJInfoData->IpAddressContentLengthU16 = 4;

    p=(C8 *)TRAJInfoData;
    for(; i< COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH; i++) *(MessageBuffer + i) = *p++;

    MessageIndex = i;


    if(debug)
    {
        // TODO: Change to log printout when byte thingy has been implemented
        printf("Header + TRAJInfo total length = %d bytes\n", (int)(COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH));
        printf("----HEADER + TRAJInfo----\n");
        for(i = 0;i < sizeof(HeaderType) + sizeof(TRAJInfoType); i ++) printf("%x ", (unsigned char)MessageBuffer[i]);
        printf("\n");
        printf("DOTM message total length = %d bytes.\n", (int)HeaderData->MessageLengthU32);
        printf("TrajectoryID = %d\n", TRAJInfoData->TrajectoryIDU16);
        printf("TrajectoryName = %s\n", TRAJInfoData->TrajectoryNameC8);
        printf("TrajectoryVersion = %d\n", TRAJInfoData->TrajectoryVersionU16);
        printf("IpAddress = %d\n", TRAJInfoData->IpAddressU32);
        printf("\n----MESSAGE----\n");
    }

    return MessageIndex; //Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH
}



I32 ObjectControlSendDOTMMEssage(C8* Filename, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug)
{

    FILE *fd;
    fd = fopen (Filename, "r");
    UtilReadLineCntSpecChars(fd, TrajBuffer);//Read first line
    int Rest = 0, i = 0, MessageLength = 0, SumMessageLength = 0, Modulo = 0, Transmissions = 0;
    Transmissions = RowCount / COMMAND_DOTM_ROWS_IN_TRANSMISSION;
    Rest = RowCount % COMMAND_DOTM_ROWS_IN_TRANSMISSION;
    U16 CrcU16 = 0;

    for(i = 0; i < Transmissions; i ++)
    {
        MessageLength = ObjectControlBuildDOTMMessage(TrajBuffer, fd, COMMAND_DOTM_ROWS_IN_TRANSMISSION, DOTMData, debug);

        if(i == Transmissions && Rest == 0)
        {
            TrajBuffer[MessageLength] = (U8)(CrcU16);
            TrajBuffer[MessageLength+1] = (U8)(CrcU16 >> 8);
            MessageLength = MessageLength + 2;
            UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
            SumMessageLength = SumMessageLength + MessageLength;
        }
        else
        {
            UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
            SumMessageLength = SumMessageLength + MessageLength;
        }

        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Transmission %d: %d bytes sent", i, MessageLength);
    }

    if(Rest > 0)
    {
        MessageLength = ObjectControlBuildDOTMMessage(TrajBuffer, fd, Rest, DOTMData, debug);
        TrajBuffer[MessageLength] = (U8)(CrcU16);
        TrajBuffer[MessageLength+1] = (U8)(CrcU16 >> 8);
        MessageLength = MessageLength + 2;
        UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
        SumMessageLength = SumMessageLength + MessageLength;
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Transmission %d: %d bytes sent.\n", i, MessageLength);
    }

    LogMessage(LOG_LEVEL_INFO,"%d DOTM bytes sent to %s:%d", SumMessageLength, IP, Port);

    fclose (fd);

    return 0;
}

I32 ObjectControlBuildDOTMMessage(C8* MessageBuffer, FILE *fd, I32 RowCount, DOTMType *DOTMData, U8 debug)
{
    I32 MessageIndex = 0;
    C8 RowBuffer[100];
    C8 DataBuffer[20];
    dbl Data;
    C8 *src, *p;
    U16 Crc = 0;

    bzero(MessageBuffer, COMMAND_DOTM_ROW_MESSAGE_LENGTH*RowCount);

    I32 i = 0, j = 0, n = 0;
    for(i = 0; i < RowCount; i++)
    {
        bzero(RowBuffer, 100);
        UtilReadLineCntSpecChars(fd, RowBuffer);

        //Read to ';' in row = LINE;0.00;21.239000;39.045000;0.000000;-1.240001;0.029103;0.004005;0.000000;3;ENDLINE;
        //Time
        src = strchr(RowBuffer, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (U64)strchr(src+1, ';') - (U64)src - 1);
        Data = atof(DataBuffer)*1e3;
        DOTMData->RelativeTimeValueIdU16 = VALUE_ID_RELATIVE_TIME;
        DOTMData->RelativeTimeContentLengthU16 = 4;
        DOTMData->RelativeTimeU32 = (U32)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //x
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*1e3;
        DOTMData->XPositionValueIdU16 = VALUE_ID_X_POSITION;
        DOTMData->XPositionContentLengthU16 = 4;
        DOTMData->XPositionI32 = (I32)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //y
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*1e3;
        DOTMData->YPositionValueIdU16 = VALUE_ID_Y_POSITION;
        DOTMData->YPositionContentLengthU16 = 4;
        DOTMData->YPositionI32 = (I32)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //z
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*1e3;
        DOTMData->ZPositionValueIdU16 = VALUE_ID_Z_POSITION;
        DOTMData->ZPositionContentLengthU16 = 4;
        DOTMData->ZPositionI32 = (I32)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Heading
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = UtilRadToDeg(atof(DataBuffer));
        Data = 450 - Data; //Turn heading back pi/2
        while(Data<0) Data+=360.0;
        while(Data>360) Data-=360.0;
        DOTMData->HeadingValueIdU16 = VALUE_ID_HEADING;
        DOTMData->HeadingContentLengthU16 = 2;
        DOTMData->HeadingU16 = (U16)(Data*1e2);
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Longitudinal speed
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*1e2;
        DOTMData->LongitudinalSpeedValueIdU16 = VALUE_ID_LONGITUDINAL_SPEED;
        DOTMData->LongitudinalSpeedContentLengthU16 = 2;
        DOTMData->LongitudinalSpeedI16 = (I16)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Lateral speed
        //src = strchr(src + 1, ';');
        //bzero(DataBuffer, 20);
        //strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        //Data = atof(DataBuffer)*1e2;
        DOTMData->LateralSpeedValueIdU16 = VALUE_ID_LATERAL_SPEED;
        DOTMData->LateralSpeedContentLengthU16 = 2;
        DOTMData->LateralSpeedI16 = -32768;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Longitudinal acceleration
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*1e3;
        DOTMData->LongitudinalAccValueIdU16 = VALUE_ID_LONGITUDINAL_ACCELERATION;
        DOTMData->LongitudinalAccContentLengthU16 = 2;
        DOTMData->LongitudinalAccI16 = (I16)Data;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Lateral acceleration
        //src = strchr(src + 1, ';');
        //bzero(DataBuffer, 20);
        //strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        //Data = atof(DataBuffer)*1e3;
        DOTMData->LateralAccValueIdU16 = VALUE_ID_LATERAL_ACCELERATION;
        DOTMData->LateralAccContentLengthU16 = 2;
        DOTMData->LateralAccI16 = -32000;
        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        //Curvature
        src = strchr(src + 1, ';');
        bzero(DataBuffer, 20);
        strncpy(DataBuffer, src+1, (uint64_t)strchr(src+1, ';') - (uint64_t)src - 1);
        Data = atof(DataBuffer)*3e4;
        DOTMData->CurvatureValueIdU16 = VALUE_ID_CURVATURE;
        DOTMData->CurvatureContentLengthU16 = 4;
        DOTMData->CurvatureI32 = (I32)Data;

        //printf("DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

        p=(C8 *)DOTMData;
        for(j=0; j<sizeof(DOTMType); j++, n++) *(MessageBuffer + n) = *p++;
        MessageIndex = n;
    }


    if(debug)
    {
        int i = 0;
        for(i = 0; i < MessageIndex; i ++)
        {
            // TODO: Write to log when bytes thingy has been implemented
            if((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15) printf("0");
            printf("%x-", (unsigned char)MessageBuffer[i]);
        }
        printf("\n");
    }

    return MessageIndex; //Total number of bytes

}


I32 ObjectControlBuildASPMessage(C8* MessageBuffer, ASPType *ASPData, U8 debug)
{
    I32 MessageIndex = 0, i;
    C8 *p;

    bzero(MessageBuffer, ASP_MESSAGE_LENGTH);
    p=(C8 *)ASPData;
    for(i=0; i<sizeof(ASPType); i++) *(MessageBuffer + i) = *p++;
    MessageIndex = i;

    if(debug)
    {
        // TODO: Write to log when bytes thingy has been implemented
        printf("ASP total length = %d bytes \n", (int)(ASP_MESSAGE_LENGTH));
        printf("\n----MESSAGE----\n");
        for(i = 0;i < sizeof(ASPType); i ++) printf("%x ", (C8)MessageBuffer[i]);
        printf("\n");
    }

    return MessageIndex; //Total number of bytes
}



I32 ObjectControlSendDTMMessage(C8 *DTMData, I32 *Socket, I32 RowCount, C8 *IP, U32 Port, DOTMType *DOTMData, U8 debug)
{

    U32 Rest = 0, i = 0, MessageLength = 0, SumMessageLength = 0, Modulo = 0, Transmissions = 0;
    U16 CrcU16 = 0;

    MessageLength = ObjectControlBuildDTMMessage(TrajBuffer, DTMData, COMMAND_DOTM_ROWS_IN_TRANSMISSION, DOTMData, 0);

    if(debug) LogMessage(LOG_LEVEL_DEBUG,"Transmission %d: %d bytes sent", i, MessageLength);

    LogMessage(LOG_LEVEL_INFO,"%d DTM bytes sent to %s:%d", SumMessageLength, IP, Port);

    return 0;
}


I32 ObjectControlBuildDTMMessage(C8 *MessageBuffer, C8 *DTMData, I32 RowCount, DOTMType *DOTMData, U8 debug)
{
    I32 MessageIndex = 0;
    U32 Data;
    C8 *src, *p;
    U16 Crc = 0;

    bzero(MessageBuffer, COMMAND_DOTM_ROW_MESSAGE_LENGTH*RowCount);

    I32 i = 0, j = 0, n = 0;
    for(i = 0; i < RowCount; i++)
    {
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"DOTM row:");
        //Time
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 3);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 2);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 1);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 0);
        DOTMData->RelativeTimeValueIdU16 = VALUE_ID_RELATIVE_TIME;
        DOTMData->RelativeTimeContentLengthU16 = 4;
        DOTMData->RelativeTimeU32 = SwapU32((U32)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Time=%d", DOTMData->RelativeTimeU32);

        //x
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 7);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 6);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 5);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 4);
        DOTMData->XPositionValueIdU16 = VALUE_ID_X_POSITION;
        DOTMData->XPositionContentLengthU16 = 4;
        DOTMData->XPositionI32 = SwapI32((I32)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"X=%d", DOTMData->XPositionI32);

        //y
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 11);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 10);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 9);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 8);
        DOTMData->YPositionValueIdU16 = VALUE_ID_Y_POSITION;
        DOTMData->YPositionContentLengthU16 = 4;
        DOTMData->YPositionI32 = SwapI32((I32)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Y=%d", DOTMData->YPositionI32);

        //z
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 15);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 14);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 13);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 12);
        DOTMData->ZPositionValueIdU16 = VALUE_ID_Z_POSITION;
        DOTMData->ZPositionContentLengthU16 = 4;
        DOTMData->ZPositionI32 = SwapI32((I32)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Z=%d", DOTMData->ZPositionI32);

        //Heading
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 17);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 16);
        //Data = UtilRadToDeg(Data);
        //Data = 4500 - Data; //Turn heading back pi/2
        //while(Data<0) Data+=360.0;
        //while(Data>3600) Data-=360.0;
        DOTMData->HeadingValueIdU16 = VALUE_ID_HEADING;
        DOTMData->HeadingContentLengthU16 = 2;
        DOTMData->HeadingU16 = SwapU16((U16)(Data));
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"Heading=%d", DOTMData->HeadingU16);

        //Longitudinal speed
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 19);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 18);
        DOTMData->LongitudinalSpeedValueIdU16 = VALUE_ID_LONGITUDINAL_SPEED;
        DOTMData->LongitudinalSpeedContentLengthU16 = 2;
        DOTMData->LongitudinalSpeedI16 = SwapI16((I16)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"LongitudinalSpeedI16=%d", DOTMData->LongitudinalSpeedI16);

        //Lateral speed
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 21);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 20);
        DOTMData->LateralSpeedValueIdU16 = VALUE_ID_LATERAL_SPEED;
        DOTMData->LateralSpeedContentLengthU16 = 2;
        DOTMData->LateralSpeedI16 = SwapI16((I16)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"LateralSpeedI16=%d", DOTMData->LateralSpeedI16);

        //Longitudinal acceleration
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 23);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 22);
        DOTMData->LongitudinalAccValueIdU16 = VALUE_ID_LONGITUDINAL_ACCELERATION;
        DOTMData->LongitudinalAccContentLengthU16 = 2;
        DOTMData->LongitudinalAccI16 = SwapI16((I16)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"LongitudinalAccI16=%d", DOTMData->LongitudinalAccI16);

        //Lateral acceleration
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 25);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 24);
        DOTMData->LateralAccValueIdU16 = VALUE_ID_LATERAL_ACCELERATION;
        DOTMData->LateralAccContentLengthU16 = 2;
        DOTMData->LateralAccI16 = SwapI16((I16)Data);
        if(debug) LogMessage(LOG_LEVEL_DEBUG,"LateralAccI16=%d", DOTMData->LateralAccI16);

        //Curvature
        Data = 0;
        Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 29);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 28);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 27);
        Data = (Data<<8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW*i + 26);
        DOTMData->CurvatureValueIdU16 = VALUE_ID_CURVATURE;
        DOTMData->CurvatureContentLengthU16 = 4;
        DOTMData->CurvatureI32 = SwapI32((I32)Data);
        if(debug) printf("CurvatureI32=%d \n", DOTMData->CurvatureI32);

        p=(C8 *)DOTMData;
        for(j=0; j<sizeof(DOTMType); j++, n++) *(MessageBuffer + n) = *p++;
        MessageIndex = n;
    }


    Crc = crc_16((const C8*)MessageBuffer, sizeof(DOTMType));
    Crc = 0;
    *(MessageBuffer + MessageIndex++) = (U8)(Crc);
    *(MessageBuffer + MessageIndex++) = (U8)(Crc >> 8);


    if(debug)
    {
        int i = 0;
        for(i = 0; i < MessageIndex; i ++)
        {
            // TODO: Write to log when bytes thingy has been implemented
            if((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15) printf("0");
            printf("%x-", (unsigned char)MessageBuffer[i]);
        }
        printf("\n");
    }

    return MessageIndex; //Total number of bytes

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
        util_error("[ObjectControl] ERR: Failed to open control socket");
    }

    server = gethostbyname(name);
    if (server == NULL)
    {
        util_error("[ObjectControl] ERR: Unknown host ");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;

    bcopy((char *) server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(port);

    LogMessage(LOG_LEVEL_INFO,"Attempting to connect to socket: %s %i",name,port);

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

    LogMessage(LOG_LEVEL_INFO,"Connected to command socket: %s %i",name,port);
    // Enable polling of status to detect remote disconnect
    fcntl(*sockfd, F_SETFL, O_NONBLOCK);


    return iResult;
}

static void vDisconnectObject(int* sockfd)
{
    close(*sockfd);
}


static void vCreateSafetyChannel(const char* name, const uint32_t port, int* sockfd, struct sockaddr_in* addr)
{
    int result;
    struct hostent *object;

    /* Connect to object safety socket */
    LogMessage(LOG_LEVEL_DEBUG,"Creating safety socket");

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

    LogMessage(LOG_LEVEL_INFO,"Created socket and safety address: %s:%d",name,port);
}

static void vCloseSafetyChannel(int* sockfd)
{
    close(*sockfd);
}

static I32 vCheckRemoteDisconnected(int* sockfd)
{
    char dummy;
    ssize_t x = recv(*sockfd, &dummy, 1, MSG_PEEK);

    // Remote has disconnected: EOF => x=0
    if (x == 0)
    {
        return 1;
    }

    if (x == -1)
    {
        // Everything is normal - no communication has been received
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;

        // Other error occurred
        LogMessage(LOG_LEVEL_WARNING,"Error when checking connection status");
        return 1;
    }

    // Something has been received on socket
    if (x > 0)
    {
        LogMessage(LOG_LEVEL_INFO,"Received unexpected communication from object on command channel");
        return 0;
    }

    return 1;
}

int ObjectControlSendUDPData(int* sockfd, struct sockaddr_in* addr, char* SendData, int Length, char debug)
{
    ssize_t result;

    // TODO: Change to log write when bytes thingy has been implemented
    if(debug){ printf("Bytes sent: "); int i = 0; for(i = 0; i < Length; i++) printf("%x-", (unsigned char)*(SendData+i)); printf("\n");}

    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));

    if (result < 0)
    {
        util_error("ERR: Failed to send on monitor socket");
    }

    return 0;
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
                LogMessage(LOG_LEVEL_DEBUG,"No data received");
            }
        }
        else
        {
            *recievedNewData = 1;
            LogMessage(LOG_LEVEL_DEBUG,"Received: <%s>",buffer);
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



OBCState_t vGetState(GSDType *GSD)
{
    return DataDictionaryGetOBCStateU8(GSD);
}

StateTransitionResult vSetState(OBCState_t requestedState, GSDType *GSD)
{
    StateTransition transitionFunction;
    StateTransitionResult retval = TRANSITION_RESULT_UNDEFINED;
    OBCState_t currentState = DataDictionaryGetOBCStateU8(GSD);

    // Always allow transitions to these two states
    if (requestedState == OBC_STATE_ERROR || requestedState == OBC_STATE_UNDEFINED)
    {
        if (DataDictionarySetOBCStateU8(GSD, requestedState) == WRITE_OK)
            retval = TRANSITION_OK;
        else
            retval = TRANSITION_MEMORY_ERROR;
    }
    else if (requestedState == currentState)
    {
        retval = TRANSITION_OK;
    }
    else
    {
        transitionFunction = tGetTransition(currentState);
        retval = transitionFunction(&currentState, requestedState);
        if (retval != TRANSITION_INVALID)
        {
            if(DataDictionarySetOBCStateU8(GSD, currentState) == WRITE_OK)
                retval = TRANSITION_OK;
            else
                retval = TRANSITION_MEMORY_ERROR;
        }
    }

    if (retval == TRANSITION_INVALID)
    {
        LogMessage(LOG_LEVEL_WARNING,"Invalid transition requested: from %d to %d",currentState,requestedState);
    }
    else if (retval == TRANSITION_MEMORY_ERROR)
    {
        LogMessage(LOG_LEVEL_ERROR, "Unable to set state to %u in shared memory!!", requestedState);
    }
    return retval;
}


StateTransition tGetTransition(OBCState_t fromState)
{
    switch (fromState)
    {
    case OBC_STATE_IDLE:
        return &tFromIdle;
    case OBC_STATE_INITIALIZED:
        return &tFromInitialized;
    case OBC_STATE_CONNECTED:
        return &tFromConnected;
    case OBC_STATE_ARMED:
        return &tFromArmed;
    case OBC_STATE_RUNNING:
        return &tFromRunning;
    case OBC_STATE_ERROR:
        return &tFromError;
    case OBC_STATE_UNDEFINED:
        return &tFromUndefined;
    }
}

StateTransitionResult tFromIdle(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_INITIALIZED)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromInitialized(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_IDLE)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromConnected(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_ARMED || requestedState == OBC_STATE_IDLE)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromArmed(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_RUNNING || requestedState == OBC_STATE_IDLE)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromRunning(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_IDLE)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromError(OBCState_t *currentState, OBCState_t requestedState)
{
    if (requestedState == OBC_STATE_IDLE)
    {
        *currentState = requestedState;
        return TRANSITION_OK;
    }
    return TRANSITION_INVALID;
}

StateTransitionResult tFromUndefined(OBCState_t *currentState, OBCState_t requestedState)
{
    return TRANSITION_INVALID;
}


OBCState_t vInitializeState(OBCState_t firstState, GSDType *GSD){
    static int8_t isInitialized = 0;
    if (!isInitialized)
    {
        isInitialized = 1;
        if(DataDictionarySetOBCStateU8(GSD,firstState) != WRITE_OK)
            util_error("Unable to write object control state to shared memory");
    }
    else {
        LogMessage(LOG_LEVEL_WARNING,"Object control state already initialized");
    }
    return DataDictionaryGetOBCStateU8(GSD);
}
