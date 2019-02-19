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
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
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
#include <unistd.h>

#include "remotecontrol.h"
#include "util.h"
#include "systemcontrol.h"
#include "timecontrol.h"



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
#define SYSTEM_CONTROL_RX_PACKET_SIZE 1280
#define SYSTEM_CONTROL_TX_PACKET_SIZE SYSTEM_CONTROL_RX_PACKET_SIZE
#define SYSTEM_CONTROL_MAX_PATH_LENGTH 255
#define IPC_BUFFER_SIZE SYSTEM_CONTROL_RX_PACKET_SIZE
//#define IPC_BUFFER_SIZE   1024
#define SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE 64
#define SYSTEM_CONTROL_PROCESS_DATA_BUFFER	128

#define SYSTEM_CONTROL_ARG_CHAR_COUNT 		2
#define SYSTEM_CONTROL_COMMAND_MAX_LENGTH 	32
#define SYSTEM_CONTROL_ARG_MAX_COUNT	 	6
#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH	32
#define SYSTEM_CONTROL_TOTAL_COMMAND_MAX_LENGTH SYSTEM_CONTROL_ARG_CHAR_COUNT + SYSTEM_CONTROL_COMMAND_MAX_LENGTH + SYSTEM_CONTROL_ARG_MAX_COUNT*SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH 
//#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH	80

#define OSTM_OPT_SET_ARMED_STATE 2
#define OSTM_OPT_SET_DISARMED_STATE 3 
#define SC_RECV_MESSAGE_BUFFER 1024

#define SMALL_BUFFER_SIZE_1024 1024
#define SMALL_BUFFER_SIZE_128 128
#define SMALL_BUFFER_SIZE_64 64
#define SMALL_BUFFER_SIZE_16 16
#define SMALL_BUFFER_SIZE_20 20
#define SMALL_BUFFER_SIZE_6 6
#define SMALL_BUFFER_SIZE_3 3
#define SMALL_BUFFER_SIZE_2 2
#define SYSTEM_CONTROL_SEND_BUFFER_SIZE 1024

#define SYSTEM_CONTROL_CONF_FILE_PATH  "conf/test.conf"
#define SYSTEM_CONTROL_TEMP_CONF_FILE_PATH  "conf/temp.conf"

#define SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE 1024


#define SYSTEM_CONTROL_RESPONSE_CODE_OK 						0x0001
#define SYSTEM_CONTROL_RESPONSE_CODE_ERROR 						0x0F10
#define SYSTEM_CONTROL_RESPONSE_CODE_FUNCTION_NOT_AVAILABLE 	0x0F20  
#define SYSTEM_CONTROL_RESPONSE_CODE_INCORRECT_STATE  		 	0x0F25  

#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_LENGTH				0x0F30
#define SYSTEM_CONTROL_RESPONSE_CODE_BUSY						0x0F40
#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_SCRIPT				0x0F50
#define SYSTEM_CONTROL_RESPONSE_CODE_INVALID_ENCRYPTION_CODE	0x0F60
#define SYSTEM_CONTROL_RESPONSE_CODE_DECRYPTION_ERROR			0x0F61
#define SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA                    0x0F62

#define GetCurrentDir getcwd
#define REMOVE_FILE 1
#define KEEP_FILE 0



typedef enum {
    Idle_0, GetServerStatus_0, ArmScenario_0, DisarmScenario_0, StartScenario_1, stop_0, AbortScenario_0, InitializeScenario_0,
    ConnectObject_0, DisconnectObject_0, GetServerParameterList_0, SetServerParameter_2, GetServerParameter_1, DownloadFile_1, UploadFile_3, CheckFileDirectoryExist_1, GetDirectoryContent_1,
    DeleteFileDirectory_1, CreateDirectory_1, replay_1, control_0, Exit_0, start_ext_trigg_1, nocommand
} SystemControlCommand_t;
const char* SystemControlCommandsArr[] = 
{ 	
    "Idle_0", "GetServerStatus_0", "ArmScenario_0", "DisarmScenario_0", "StartScenario_1", "stop_0", "AbortScenario_0", "InitializeScenario_0",
    "ConnectObject_0", "DisconnectObject_0", "GetServerParameterList_0", "SetServerParameter_2", "GetServerParameter_1", "DownloadFile_1", "UploadFile_3", "CheckFileDirectoryExist_1", "GetDirectoryContent_1",
    "DeleteFileDirectory_1", "CreateDirectory_1", "replay_1", "control_0", "Exit_0", "start_ext_trigg_1"
};

const char* SystemControlStatesArr[] = { "UNDEFINED", "INITIALIZED", "IDLE", "READY", "RUNNING", "INWORK", "ERROR"};
const char* SystemControlOBCStatesArr[] = { "UNDEFINED", "IDLE", "INITIALIZED", "CONNECTED", "ARMED", "RUNNING", "ERROR"};


char SystemControlCommandArgCnt[SYSTEM_CONTROL_ARG_CHAR_COUNT];
char SystemControlStrippedCommand[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
char SystemControlArgument[SYSTEM_CONTROL_ARG_MAX_COUNT][SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH];
C8 *STR_SYSTEM_CONTROL_RX_PACKET_SIZE = "1280";
C8 *STR_SYSTEM_CONTROL_TX_PACKET_SIZE = "1200";

/*------------------------------------------------------------
  -- Function declarations.
  ------------------------------------------------------------*/
//SystemControlCommand_t SystemControlFindCommandOld(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *ArgCount);
SystemControlCommand_t SystemControlFindCommand(const char* CommandBuffer, SystemControlCommand_t *CurrentCommand, int *ArgCount);
static I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle, struct in_addr *ip_addr);
static I32 SystemControlConnectServer(int* sockfd, const char* name, const uint32_t port);
static void SystemControlSendBytes(const char* data, int length, int* sockfd, int debug);
void SystemControlSendControlResponse(U16 ResponseStatus, C8* ResponseString, C8* ResponseData, I32 ResponseDataLength, I32* Sockfd, U8 Debug);
void SystemControlSendLog(C8* LogString, I32* Sockfd, U8 Debug);
void SystemControlSendMONR(C8* LogString, I32* Sockfd, U8 Debug);
static void SystemControlCreateProcessChannel(const C8* name, const U32 port, I32 *sockfd, struct sockaddr_in* addr);
I32 SystemControlSendUDPData(I32 *sockfd, struct sockaddr_in* addr, C8 *SendData, I32 Length, U8 debug);
I32 SystemControlReadServerParameterList(C8 *ParameterList, U8 debug);
I32 SystemControlReadServerParameter(C8 *ParameterName, C8 *ReturnValue, U8 Debug);
I32 SystemControlWriteServerParameter(C8 *ParameterName, C8 *NewValue, U8 Debug);
I32 SystemControlCheckFileDirectoryExist(C8 *ParameterName, C8 *ReturnValue, U8 Debug);
I32 SystemControlUploadFile(C8 *Path, C8 *FileSize, C8 *PacketSize, C8 *ReturnValue, U8 Debug);
I32 SystemControlReceiveRxData(I32 *sockfd, C8 *Path, C8 *FileSize, C8 *PacketSize, C8 *ReturnValue, U8 Debug);
I32 SystemControlDeleteFileDirectory(C8 *Path, C8 *ReturnValue, U8 Debug);
I32 SystemControlBuildFileContentInfo(C8 *Path, C8 *ReturnValue, U8 Debug);
I32 SystemControlSendFileContent(I32 *sockfd, C8 *Path, C8 *PacketSize, C8 *ReturnValue, U8 Remove, U8 Debug);
I32 SystemControlCreateDirectory(C8 *Path, C8 *ReturnValue, U8 Debug);

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void systemcontrol_task(TimeType *GPSTime, GSDType *GSD)
{

    I32 ServerHandle;
    I32 ClientSocket;
    I32 ClientResult = 0;
    struct sockaddr_in ProcessChannelAddr;
    struct in_addr ip_addr;
    I32 ProcessChannelSocket;

    ServerState_t server_state = SERVER_STATE_UNDEFINED;
    SystemControlCommand_t SystemControlCommand = Idle_0;
    SystemControlCommand_t PreviousSystemControlCommand = Idle_0;

    int CommandArgCount=0, /*CurrentCommandArgCounter=0,*/ CurrentInputArgCount=0;
    C8 pcBuffer[IPC_BUFFER_SIZE];
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
    U64 uiTime;
    U32 DelayedStartU32;
    U8 ModeU8 = 0;
    C8 TextBufferC8[SMALL_BUFFER_SIZE_20];
    C8 ServerIPC8[SMALL_BUFFER_SIZE_20];
    C8 UsernameC8[SMALL_BUFFER_SIZE_20];
    C8 PasswordC8[SMALL_BUFFER_SIZE_20];
    U16 ServerPortU16;
    I32 ServerSocketI32=0;
    ServiceSessionType SessionData;
    C8 RemoteServerRxData[1024];
    struct timespec sleep_time, ref_time;
    struct timeval CurrentTimeStruct;
    U64 CurrentTimeU64 = 0;
    U64 TimeDiffU64 = 0;
    U64 OldTimeU64 = 0;
    U64 PollRateU64 = 0;
    C8 ControlResponseBuffer[SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE];
    U8 OBCStateU8;
    C8 UserControlIPC8[SMALL_BUFFER_SIZE_20];
    C8 ProcessControlData[SYSTEM_CONTROL_PROCESS_DATA_BUFFER];
    U32 ProcessControlSendCounterU32 = 0;
    U32	PCDMessageLengthU32;
    U16	PCDMessageCodeU16;
    struct timeval now;
    U16 MilliU16 = 0, NowU16 = 0;
    U64 GPSmsU64 = 0;
    C8 ParameterListC8[SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE];
    U32 LengthU32 = 0;
    C8 BinBuffer[SMALL_BUFFER_SIZE_1024];
 
    C8 TxBuffer[SYSTEM_CONTROL_TX_PACKET_SIZE];

    //C8 SIDSData[128][10000][8];

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

    //printf("[SystemControl] Mode: %d\n", ModeU8);
    //printf("[SystemControl] Remote server IP: %s\n", ServerIPC8);
    //printf("[SystemControl] Remote server port: %d\n", ServerPortU16);
    //printf("[SystemControl] Username: %s\n", UsernameC8);
    //printf("[SystemControl] Password: %s\n", PasswordC8);

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

                if(USE_LOCAL_USER_CONTROL == 0)
                {

                    ClientResult = SystemControlInitServer(&ClientSocket, &ServerHandle, &ip_addr);
                    bzero(UserControlIPC8, SMALL_BUFFER_SIZE_20);
                    sprintf(UserControlIPC8, "%s", inet_ntoa(ip_addr));
                    printf("[SystemControl] UserControl IP address is %s\n", inet_ntoa(ip_addr));
                    SystemControlCreateProcessChannel(UserControlIPC8, SYSTEM_CONTROL_PROCESS_PORT, &ProcessChannelSocket, &ProcessChannelAddr);

                }
                if(USE_LOCAL_USER_CONTROL == 1)
                {
                    ClientResult = SystemControlConnectServer(&ClientSocket, LOCAL_USER_CONTROL_IP, LOCAL_USER_CONTROL_PORT);
                    SystemControlCreateProcessChannel(LOCAL_USER_CONTROL_IP, SYSTEM_CONTROL_PROCESS_PORT, &ProcessChannelSocket, &ProcessChannelAddr);
                }

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
            else if(ClientResult > 0 && ClientResult < SYSTEM_CONTROL_TOTAL_COMMAND_MAX_LENGTH)
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
        {   /* use util.c function to call time
            gettimeofday(&CurrentTimeStruct, NULL);
            CurrentTimeU64 = (uint64_t)CurrentTimeStruct.tv_sec*1000 + (uint64_t)CurrentTimeStruct.tv_usec/1000;
            */
            CurrentTimeU64 = UtilgetCurrentUTCtimeMS();
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

        else if(iCommand == COMM_MONI){
          //printf("Monr sys %s\n", pcRecvBuffer);
          C8 Data[strlen(pcRecvBuffer) + 11];
          bzero(Data,strlen(Data));

          Data[3] = strlen(pcRecvBuffer);
          Data[5] = 2;

          strcat((Data + 11), pcRecvBuffer);

          UtilSendUDPData("SystemControl", &ProcessChannelSocket, &ProcessChannelAddr, &Data, sizeof(Data), 0);
        }

        ++ProcessControlSendCounterU32;
        if(ProcessChannelSocket != 0 && ProcessControlSendCounterU32 == 1)
        {
            ProcessControlSendCounterU32 = 0;
            bzero(ProcessControlData, SYSTEM_CONTROL_PROCESS_DATA_BUFFER);
            PCDMessageLengthU32 = 14;
            PCDMessageCodeU16 = 1;

            GPSmsU64 = GPSTime->GPSMillisecondsU64 + (U64)TimeControlGetMillisecond(GPSTime);
            ProcessControlData[0] =	(U8)(PCDMessageLengthU32 >> 24);
            ProcessControlData[1] =	(U8)(PCDMessageLengthU32 >> 16);
            ProcessControlData[2] =	(U8)(PCDMessageLengthU32 >> 8);
            ProcessControlData[3] =	(U8) PCDMessageLengthU32;
            ProcessControlData[4] =	(U8)(PCDMessageCodeU16 >> 8);
            ProcessControlData[5] =	(U8) PCDMessageCodeU16;
            ProcessControlData[6] =	(U8)(GPSmsU64 >> 56);
            ProcessControlData[7] =	(U8)(GPSmsU64 >> 48);
            ProcessControlData[8] =	(U8)(GPSmsU64 >> 40);
            ProcessControlData[9] =	(U8)(GPSmsU64 >> 32);
            ProcessControlData[10] =	(U8)(GPSmsU64 >> 24);
            ProcessControlData[11] =	(U8)(GPSmsU64 >> 16);
            ProcessControlData[12] =	(U8)(GPSmsU64 >> 8);
            ProcessControlData[13] =	(U8)(GPSmsU64);
            ProcessControlData[14] = server_state;
            ProcessControlData[15] = OBCStateU8;
            ProcessControlData[16] = (U8)(GSD->TimeControlExecTimeU16 >> 8);
            ProcessControlData[17] = (U8) GSD->TimeControlExecTimeU16;
            ProcessControlData[18] = (U8) GPSTime->FixQualityU8;
            ProcessControlData[19] = (U8) GPSTime->NSatellitesU8;

            SystemControlSendUDPData(&ProcessChannelSocket, &ProcessChannelAddr, ProcessControlData, PCDMessageLengthU32 + 6, 1);
        }



        switch(SystemControlCommand)
        {
        // can you access GetServerParameterList_0, GetServerParameter_1, SetServerParameter_2 and DISarmScenario and Exit from the GUI
        case Idle_0:
            /*bzero(pcRecvBuffer,SC_RECV_MESSAGE_BUFFER);
                iCommRecv(&iCommand,pcRecvBuffer,SC_RECV_MESSAGE_BUFFER,NULL);

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
            DEBUG_LPRINT(DEBUG_LEVEL_HIGH,"[SystemControl] State: %s, OBCState: %s\n", SystemControlStatesArr[server_state], SystemControlOBCStatesArr[OBCStateU8]);
            SystemControlCommand = Idle_0;
            bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
            ControlResponseBuffer[0] = server_state;
            ControlResponseBuffer[1] = OBCStateU8;
            DEBUG_LPRINT(DEBUG_LEVEL_LOW,"GPSMillisecondsU64: %ld\n", GPSTime->GPSMillisecondsU64); // GPSTime just ticks from 0 up shouldent it be in the global GPStime?
            SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetServerStatus:", ControlResponseBuffer, 2, &ClientSocket, 0);
            break;
        case GetServerParameterList_0:
            SystemControlCommand = Idle_0;
            bzero(ParameterListC8, SYSTEM_CONTROL_SERVER_PARAMETER_LIST_SIZE);
            SystemControlReadServerParameterList(ParameterListC8, 0);
            SystemControlSendControlResponse(strlen(ParameterListC8) > 0 ? SYSTEM_CONTROL_RESPONSE_CODE_OK: SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA , "GetServerParameterList:", ParameterListC8, strlen(ParameterListC8), &ClientSocket, 0);
            break;
        case GetServerParameter_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlReadServerParameter(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(strlen(ControlResponseBuffer) > 0 ? SYSTEM_CONTROL_RESPONSE_CODE_OK: SYSTEM_CONTROL_RESPONSE_CODE_NO_DATA , "GetServerParameter:", ControlResponseBuffer, strlen(ControlResponseBuffer), &ClientSocket, 0);
            } else { printf("[SystemControl] Err: Wrong parameter count in GetServerParameter(Name)!\n"); SystemControlCommand = Idle_0;}
            break;
        case SetServerParameter_2:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlWriteServerParameter(SystemControlArgument[0], SystemControlArgument[1], 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "SetServerParameter:", ControlResponseBuffer, 0, &ClientSocket, 0);

            } else { printf("[SystemControl] Err: Wrong parameter count in SetServerParameter(Name, Value)!\n"); SystemControlCommand = Idle_0;}
        break;
        case CheckFileDirectoryExist_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "CheckFileDirectoryExist:", ControlResponseBuffer, 1, &ClientSocket, 0);

            } else { printf("[SystemControl] Err: Wrong parameter count in CheckFFExist(path)!\n"); SystemControlCommand = Idle_0;}
        break;
        case DeleteFileDirectory_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlDeleteFileDirectory(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DeleteFileDirectory:", ControlResponseBuffer, 1, &ClientSocket, 0);

            } else { printf("[SystemControl] Err: Wrong parameter count in DeleteFileDirectory(path)!\n"); SystemControlCommand = Idle_0;}
        break;
        case CreateDirectory_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlCreateDirectory(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "CreateDirectory:", ControlResponseBuffer, 1, &ClientSocket, 0);

            } else { printf("[SystemControl] Err: Wrong parameter count in CreateDirectory(path)!\n"); SystemControlCommand = Idle_0;}
        break;
        case GetDirectoryContent_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetDirectoryContent:", ControlResponseBuffer, 1, &ClientSocket, 0);
                if(ControlResponseBuffer[0] == FOLDER_EXIST)
                {
                    UtilCreateDirContent(SystemControlArgument[0], "/dir.info");
                    bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                    SystemControlBuildFileContentInfo("/dir.info", ControlResponseBuffer, 0);
                    SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "GetDirectoryContent:", ControlResponseBuffer, 4, &ClientSocket, 0);
                    SystemControlSendFileContent(&ClientSocket, "/dir.info", STR_SYSTEM_CONTROL_TX_PACKET_SIZE, ControlResponseBuffer, REMOVE_FILE, 0);
                }

            } else { printf("[SystemControl] Err: Wrong parameter count in GetDirectoryContent(path)!\n"); SystemControlCommand = Idle_0;}
        break;
        case DownloadFile_1:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlCheckFileDirectoryExist(SystemControlArgument[0], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DownloadFile:", ControlResponseBuffer, 1, &ClientSocket, 0);
                if(ControlResponseBuffer[0] == FILE_EXIST)
                {
                    UtilCreateDirContent(SystemControlArgument[0], SystemControlArgument[0]);
                    bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                    SystemControlBuildFileContentInfo(SystemControlArgument[0], ControlResponseBuffer, 0);
                    SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "DownloadFile:", ControlResponseBuffer, 4, &ClientSocket, 0);
                    SystemControlSendFileContent(&ClientSocket, SystemControlArgument[0], STR_SYSTEM_CONTROL_TX_PACKET_SIZE, ControlResponseBuffer, KEEP_FILE, 0);
                }

            } else { printf("[SystemControl] Err: Wrong parameter count in GetDirectoryContent(path)!\n"); SystemControlCommand = Idle_0;}
        break;
        case UploadFile_3:
            if(CurrentInputArgCount == CommandArgCount)
            {
                SystemControlCommand = Idle_0;
                bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                SystemControlUploadFile(SystemControlArgument[0], SystemControlArgument[1], SystemControlArgument[2], ControlResponseBuffer, 0);
                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK , "UploadFile:", ControlResponseBuffer, 1, &ClientSocket, 0);
                
                if(ControlResponseBuffer[0] == SERVER_PREPARED_BIG_PACKET_SIZE) //Server is ready to receive data
                {
                    printf("[SystemControl] Receiving file: %s\n", SystemControlArgument[0]);
                    SystemControlReceiveRxData(&ClientSocket, SystemControlArgument[0], SystemControlArgument[1], STR_SYSTEM_CONTROL_RX_PACKET_SIZE, ControlResponseBuffer, 0);
                } 
                else if (ControlResponseBuffer[0] == PATH_INVALID_MISSING)
                { 
                    printf("[SystemControl] Failed receiving file: %s\n", SystemControlArgument[0]);
                    SystemControlReceiveRxData(&ClientSocket, "/file.tmp", SystemControlArgument[1], STR_SYSTEM_CONTROL_RX_PACKET_SIZE, ControlResponseBuffer, 0);
                    SystemControlDeleteFileDirectory("/file.tmp", ControlResponseBuffer, 0);
                    ControlResponseBuffer[0] = PATH_INVALID_MISSING;
                }
                else
                {
                    printf("[SystemControl] Receiving file: %s\n", SystemControlArgument[0]);
                    SystemControlReceiveRxData(&ClientSocket, SystemControlArgument[0], SystemControlArgument[1], SystemControlArgument[2], ControlResponseBuffer, 0);  
                }  

                SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "UploadFile:", ControlResponseBuffer, 1, &ClientSocket, 0);

            } else { printf("[SystemControl] Err: Wrong parameter count in PrepFileRx(path, filesize, packetsize)!\n"); SystemControlCommand = Idle_0;}
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
                SystemControlSendLog("[SystemControl] Simulate that all objects becomes disarmed.\n", &ClientSocket, 0);
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
                if(server_state == SERVER_STATE_IDLE && strstr(SystemControlOBCStatesArr[OBCStateU8], "ARMED") != NULL) //Temporary!
                {
                    bzero(pcBuffer, IPC_BUFFER_SIZE);
                    /* Lest use UTC time everywhere instead of etsi and gps time
                    gettimeofday(&tvTime, NULL);
                    uiTime = (uint64_t)tvTime.tv_sec*1000 + (uint64_t)tvTime.tv_usec/1000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
                    */
                    uiTime = UtilgetCurrentUTCtimeMS();
                    if(TIME_COMPENSATE_LAGING_VM) uiTime = uiTime - TIME_COMPENSATE_LAGING_VM_VAL;

                    printf("[SystemControl] Current timestamp (gtd): %lu\n",uiTime );

                    //clock_gettime(CLOCK_MONOTONIC_COARSE, &tTime);
                    //clock_gettime(CLOCK_REALTIME, &tTime);
                    //uiTime = (uint64_t)tTime.tv_sec*1000 + (uint64_t)tTime.tv_nsec/1000000 - MS_FROM_1970_TO_2004_NO_LEAP_SECS + DIFF_LEAP_SECONDS_UTC_ETSI*1000;
                    //printf("[SystemControl] Current timestamp (cgt): %lu\n",uiTime );
                    //printf("[SystemControl] Current timestamp: %lu\n",uiTime );

                    //uiTime += atoi(SystemControlArgument[0]);
                    uiTime = atoi(SystemControlArgument[0]);
                    DelayedStartU32 = atoi(SystemControlArgument[1]);
                    sprintf ( pcBuffer,"%" PRIu8 ";%" PRIu64 ";%" PRIu32 ";", 0, uiTime, DelayedStartU32);
                    printf("[SystemControl] Sending START <%s> (delayed +%s ms)\n",pcBuffer, SystemControlArgument[1]);
                    fflush(stdout);

                    (void)iCommSend(COMM_STRT,pcBuffer);
                    bzero(ControlResponseBuffer,SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE);
                    SystemControlSendControlResponse(SYSTEM_CONTROL_RESPONSE_CODE_OK, "StartScenario:", ControlResponseBuffer, 0, &ClientSocket, 0);
                    server_state = SERVER_STATE_INWORK; 
                    //server_state = SERVER_STATE_IDLE; //Temporary!
                    //SystemControlCommand = Idle_0; //Temporary!
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

            } else printf("[SystemControl] START command parameter count error.\n");
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
            GSD->ExitU8 = 1;
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


        sleep_time.tv_sec = 0;
        sleep_time.tv_nsec = 10000000;
        nanosleep(&sleep_time,&ref_time);


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

void SystemControlSendMONR(C8* MONRStr, I32* Sockfd, U8 Debug){
  int i, n, j, t;
  C8 Length[4];
  C8 Header[2] = {0 ,2};
  C8 Data[SYSTEM_CONTROL_SEND_BUFFER_SIZE];

  bzero(Data, SYSTEM_CONTROL_SEND_BUFFER_SIZE);
  n = 2 + strlen(MONRStr);
  Length[0] = (C8)(n >> 24); Length[1] = (C8)(n >> 16); Length[2] = (C8)(n >> 8); Length[3] = (C8)n;


  if(n + 4 < SYSTEM_CONTROL_SEND_BUFFER_SIZE)
  {
      for(i = 0, j = 0; i < 4; i++, j++) Data[j] = Length[i];
      for(i = 0; i < 2; i++, j++) Data[j] = Header[i];
      t = strlen(MONRStr);
      for(i = 0; i < t; i++, j++) Data[j] = *(MONRStr+i);
      SystemControlSendBytes(Data, n + 4, Sockfd, 0);
  } else printf("[SystemControl] MONR string longer then %d bytes!\n", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
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

        if(Debug) { for(i = 0; i < n + 4; i++) printf("%x-", Data[i]); printf("\n"); }
 
        SystemControlSendBytes(Data, n + 4, Sockfd, 0);
    } else printf("[SystemControl] Response data more then %d bytes!\n", SYSTEM_CONTROL_SEND_BUFFER_SIZE);
}


static void SystemControlSendBytes(const char* data, int length, int* sockfd, int debug)
{
    int i, n;
    
    if(debug == 1){ printf("Bytes sent: "); int i = 0; for(i = 0; i < length; i++) printf("%d ", (C8)*(data+i)); printf("\n");}

    n = write(*sockfd, data, length);
    if (n < 0)
    {
        util_error("[SystemControl] ERR: Failed to send on control socket");
    }
}



static I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle, struct in_addr *ip_addr)
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

    printf("[SystemControl] Connection received: %x, %i\n", htons(cli_addr.sin_addr.s_addr), htons(command_server_addr.sin_port));

    ip_addr->s_addr = cli_addr.sin_addr.s_addr; //Set IP-address of Usercontrol

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


    DEBUG_LPRINT(DEBUG_LEVEL_HIGH,"[SystemControl] Try to connect to control socket: %s %i\n",name,port);

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

    DEBUG_LPRINT(DEBUG_LEVEL_HIGH,"[SystemControl] Maestro connected to UserControl: %s %i\n",name,port);
    return iResult;

}


static void SystemControlCreateProcessChannel(const C8* name, const U32 port, I32 *sockfd, struct sockaddr_in* addr)
{
    int result;
    struct hostent *object;

    /* Connect to object safety socket */

    DEBUG_LPRINT(DEBUG_LEVEL_LOW,"%s","[SystemControl] Creating CPC socket\n");

    *sockfd= socket(AF_INET, SOCK_DGRAM, 0);
    if (*sockfd < 0)
    {
        util_error("[SystemControl] ERR: Failed to connect to CPC socket");
    }

    /* Set address to object */
    object = gethostbyname(name);

    if (object==0)
    {
        util_error("[SystemControl] ERR: Unknown host");
    }

    bcopy((char *) object->h_addr, (char *)&addr->sin_addr.s_addr, object->h_length);
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);

    /* set socket to non-blocking */
    result = fcntl(*sockfd, F_SETFL,
                   fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
    if (result < 0)
    {
        util_error("[SystemControl] ERR: calling fcntl");
    }

    DEBUG_LPRINT(DEBUG_LEVEL_HIGH,"[SystemControl] Created CPC socket and address: %s %d\n",name,port);


}

I32 SystemControlSendUDPData(I32 *sockfd, struct sockaddr_in* addr, C8 *SendData, I32 Length, U8 debug)
{
    I32 result;

    result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *) addr, sizeof(struct sockaddr_in));

    if (result < 0)
    {
        util_error("[SystemControl] Failed to send on process control socket.");
    }

    return 0;
}



I32 SystemControlWriteServerParameter(C8 *ParameterName, C8 *NewValue, U8 Debug)
{

    I32 RowCount, i;
    C8 Parameter[SMALL_BUFFER_SIZE_64];
    C8 Row[SMALL_BUFFER_SIZE_128];
    C8 NewRow[SMALL_BUFFER_SIZE_128];
    FILE *fd, *TempFd;
    C8 *ptr1, *ptr2;
    U8 ParameterFound = 0;
    bzero(Parameter, SMALL_BUFFER_SIZE_64);

    strcat(Parameter, ParameterName);
    strcat(Parameter, "=");

    //Remove temporary file
    remove(SYSTEM_CONTROL_TEMP_CONF_FILE_PATH);

    //Create temporary file
    TempFd = fopen (SYSTEM_CONTROL_TEMP_CONF_FILE_PATH, "w+");
    
    //Open configuration file
    fd = fopen (SYSTEM_CONTROL_CONF_FILE_PATH, "r");
    

    if(fd > 0)
    {
        RowCount = UtilCountFileRows(fd);
        fclose(fd);
        fd = fopen (SYSTEM_CONTROL_CONF_FILE_PATH, "r");
    
        for(i = 0; i < RowCount; i++)
        {
            bzero(Row, SMALL_BUFFER_SIZE_128);
            UtilReadLine(fd, Row);
            
            ptr1 = strstr(Row, Parameter);
            ptr2 = strstr(Row, "//");
            if (ptr2 == NULL) ptr2 = ptr1; //No comment found 
            if(ptr1 != NULL && (U64)ptr2 >= (U64)ptr1 && ParameterFound == 0)
            {
                ParameterFound = 1;
                bzero(NewRow, SMALL_BUFFER_SIZE_128);
                strncpy(NewRow, Row, (U64)ptr1 - (U64)Row + strlen(Parameter));
                strcat(NewRow, NewValue);
                if((U64)ptr2 > (U64)ptr1)
                { 
                    strcat(NewRow, " "); // Add space
                    strcat(NewRow, ptr2); // Add the comment
                }

                if(Debug)
                {
                    printf("[SystemControl] Changed parameter: %s\n", NewRow);
                }

                strcat(NewRow, "\n");
                (void)fwrite(NewRow,1,strlen(NewRow),TempFd);

            } 
            else
            {
                strcat(Row, "\n");
                (void)fwrite(Row,1,strlen(Row),TempFd);
            }
        }
    
        fclose(TempFd);
        fclose(fd);
    
        //Remove test.conf
        remove(SYSTEM_CONTROL_CONF_FILE_PATH);

        //Rename temp.conf to test.conf
        rename(SYSTEM_CONTROL_TEMP_CONF_FILE_PATH, SYSTEM_CONTROL_CONF_FILE_PATH);

        //Remove temporary file
        remove(SYSTEM_CONTROL_TEMP_CONF_FILE_PATH);

    }


    return 0;
}



I32 SystemControlReadServerParameter(C8 *ParameterName, C8 *ReturnValue, U8 Debug)
{

    I32 RowCount, i;
    C8 TextBuffer[SMALL_BUFFER_SIZE_128];

    bzero(TextBuffer, SMALL_BUFFER_SIZE_128);

    strcat(TextBuffer, ParameterName);
    strcat(TextBuffer, "=");

    UtilSearchTextFile(SYSTEM_CONTROL_CONF_FILE_PATH, TextBuffer, "", ReturnValue);

    if(Debug)
    {
        printf("[SystemControl] %s = %s\n", ParameterName, ReturnValue);
    }

    return strlen(ReturnValue);
}


I32 SystemControlReadServerParameterList(C8 *ParameterList, U8 Debug)
{

    I32 RowCount, i;
    C8 TextBuffer[SMALL_BUFFER_SIZE_128];
    FILE *fd;

    fd = fopen (SYSTEM_CONTROL_CONF_FILE_PATH, "r");
    if(fd > 0)
    {
        RowCount = UtilCountFileRows(fd);
        fclose(fd);
        fd = fopen (SYSTEM_CONTROL_CONF_FILE_PATH, "r");
    
        for(i = 0; i < RowCount; i++)
        {
            bzero(TextBuffer, SMALL_BUFFER_SIZE_128);
            UtilReadLineCntSpecChars(fd, TextBuffer);
            if(strlen(TextBuffer) > 0)
            {
                strcat(ParameterList, TextBuffer);
                strcat(ParameterList, ";");        
            }
        }
    
        fclose(fd);
    }

    if(Debug)
    {
        printf("[SystemControl] ParameterList = %s\n", ParameterList);
    }

    return strlen(ParameterList);
}

I32 SystemControlBuildFileContentInfo(C8 *Path, C8 *ReturnValue, U8 Debug)
{
                
    struct stat st;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);
     
    stat(CompletePath, &st);
    *(ReturnValue+0) = (U8)(st.st_size >> 24);
    *(ReturnValue+1) = (U8)(st.st_size >> 16);
    *(ReturnValue+2) = (U8)(st.st_size >> 8);
    *(ReturnValue+3) = (U8)st.st_size;
    
    if(Debug) printf("Filesize %d of %s\n", (I32)st.st_size, CompletePath);

    return 0;
}

I32 SystemControlCheckFileDirectoryExist(C8 *ParameterName, C8 *ReturnValue, U8 Debug)
{
                
    DIR *pDir;
    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, ParameterName);
     
    *ReturnValue = PATH_INVALID_MISSING; 

    pDir = opendir(CompletePath);
    if(pDir == NULL)
    {
        fd = fopen(CompletePath, "r");
        if(fd != NULL)
        {
         *ReturnValue = FILE_EXIST; //File exist
         fclose(fd);
        }
    }
    else 
    {
        *ReturnValue = FOLDER_EXIST; //Directory exist
        closedir(pDir);
    }
    
    if(Debug) printf("%d %s\n", *ReturnValue, CompletePath);

    return 0;
}


I32 SystemControlDeleteFileDirectory(C8 *Path, C8 *ReturnValue, U8 Debug)
{
                
    DIR *pDir;
    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);
     
    *ReturnValue = PATH_INVALID_MISSING; 
    
    pDir = opendir(CompletePath);
    if(pDir == NULL)
    {
        fd = fopen(CompletePath, "r");
        if(fd == NULL)
        {
            *ReturnValue = PATH_INVALID_MISSING; //Missing file
        }
        else
        {
            if(0 == remove(CompletePath)) //Delete file
            {    
                *ReturnValue = SUCCEDED_DELETE; 
            }
            else
            {
                *ReturnValue = FAILED_DELETE;
            }
        }
    }
    else 
    {
            if(0 == remove(CompletePath)) //Delete directory
            {    
                *ReturnValue = SUCCEDED_DELETE; 
            }
            else
            {
                *ReturnValue = FAILED_DELETE;
            }
    }
    
    if(*ReturnValue == SUCCEDED_DELETE) printf("[SystemControl] Deleted: %s\n", CompletePath);
    else if(*ReturnValue == FAILED_DELETE) printf("[SystemControl] Failed to delete: %s\n", CompletePath);

    return 0;
}


I32 SystemControlCreateDirectory(C8 *Path, C8 *ReturnValue, U8 Debug)
{
                
    DIR *pDir;
    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);
     
    *ReturnValue = PATH_INVALID_MISSING; 

    pDir = opendir(CompletePath);
    if(pDir == NULL)
    {
        fd = fopen(CompletePath, "r");
        if(fd != NULL)
        {
            *ReturnValue = FILE_EXIST; //This is a file!
            fclose(fd);
        }
        else
        {
            if(0 == mkdir(CompletePath, S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH)) //Make the new directory
            {    
                *ReturnValue = SUCCEDED_CREATE_FOLDER; 
            }
            else
            {
                *ReturnValue = FAILED_CREATE_FOLDER;
            }
        }
    }
    else 
    {
        *ReturnValue = FOLDER_EXIST; //Directory exist
        closedir(pDir);
    }
    
    if(Debug) printf("%d %s\n", *(ReturnValue), CompletePath);

    if(*ReturnValue == SUCCEDED_CREATE_FOLDER) printf("[SystemControl] Folder created: %s\n", CompletePath);
    
    return 0;
}




I32 SystemControlUploadFile(C8 *Path, C8 *FileSize, C8 *PacketSize, C8 *ReturnValue, U8 Debug)
{

    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);

    if(Debug)
    {
        printf("%s\n", Path);
        printf("%s\n", FileSize);
        printf("%s\n", PacketSize);
        printf("%s\n", CompletePath);
    }
   
    if(atoi(PacketSize) > SYSTEM_CONTROL_RX_PACKET_SIZE) //Check packet size
    {
        *ReturnValue = SERVER_PREPARED_BIG_PACKET_SIZE;
        return 0;
    }

    fd = fopen(CompletePath, "r");
    if(fd != NULL)
    {
        fclose(fd);
        remove(CompletePath); //Remove file if exist
    }

    fd = fopen(CompletePath, "w+"); //Create the file
    if(fd != NULL)
    {
        *ReturnValue = SERVER_PREPARED;//Server prepared
        fclose(fd);
        return 0;
    }
    else
    {
        //ok, path invalid create temporary file
        bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
        GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
        strcat(CompletePath, "/file.tmp");
        fd = fopen(CompletePath, "r");
        if(fd != NULL)
        {
            fclose(fd);
            remove(CompletePath); //Remove file if exist
        }
        fd = fopen(CompletePath, "w+"); //Create the temporary file

        *ReturnValue = PATH_INVALID_MISSING; 

        return 0;
    } 

    return 0;
}

I32 SystemControlReceiveRxData(I32 *sockfd, C8 *Path, C8 *FileSize, C8 *PacketSize, C8 *ReturnValue, U8 Debug)
{

    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);
    U32 FileSizeU32 = atoi(FileSize);
    U16 PacketSizeU16 = atoi(PacketSize);
    I32 ClientStatus = 0, Time1 = 0, Time2 = 0, TimeDiff = 0, i = 0, j = 0;
    C8 RxBuffer[SYSTEM_CONTROL_RX_PACKET_SIZE];
    U32 TotalRxCount = 0, TransmissionCount = 0, RestCount = 0;
    struct timeval CurTime;


    if(Debug)
    {
        printf("%s\n", Path);
        printf("%s\n", FileSize);
        printf("%s\n", PacketSize);
        printf("%s\n", CompletePath);
    }



    fd = fopen(CompletePath, "w+");
    if(fd != NULL)
    {

        gettimeofday(&CurTime, NULL);
        Time1 = CurTime.tv_sec*1000 + CurTime.tv_usec/1000;
           
        while (TotalRxCount < FileSizeU32 && TimeDiff < 3000)
        {
            gettimeofday(&CurTime, NULL);
            Time2 = CurTime.tv_sec*1000 + CurTime.tv_usec/1000; 

            bzero(RxBuffer,PacketSizeU16);
            ClientStatus = recv(*sockfd, RxBuffer, PacketSizeU16, MSG_WAITALL);
            
            if (ClientStatus > 0)
            {
                i ++;
                fwrite(RxBuffer, 1, ClientStatus, fd);
                fflush(fd);
                if(Debug)
                {
                    printf("%d, %d, %d, %d :", i, ClientStatus, TotalRxCount, TimeDiff);
                    for(j = 0; j < 10; j ++ ) printf("%x-", RxBuffer[j]);
                    printf("...\n");
                }
                TotalRxCount = TotalRxCount + ClientStatus;
                gettimeofday(&CurTime, NULL);
                Time1 = CurTime.tv_sec*1000 + CurTime.tv_usec/1000; 
            }

            
            TimeDiff = abs(Time1 - Time2);
        }

        fclose(fd);

        if(TotalRxCount == FileSizeU32)
        { 
            *ReturnValue = FILE_UPLOADED;
        }
        else if(TotalRxCount > FileSizeU32)
        {
            *ReturnValue = FILE_TO_MUCH_DATA;
        }
        else
        {
            *ReturnValue = TIME_OUT;
        } 
        
        printf("[SystemControl] Rec count = %d, Req count = %d\n", TotalRxCount, FileSizeU32);

    }
 
    return 0;
}


I32 SystemControlSendFileContent(I32 *sockfd, C8 *Path, C8 *PacketSize, C8 *ReturnValue, U8 Remove, U8 Debug)
{

    FILE *fd;
    C8 CompletePath[SYSTEM_CONTROL_MAX_PATH_LENGTH];
    bzero(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    GetCurrentDir(CompletePath, SYSTEM_CONTROL_MAX_PATH_LENGTH);
    strcat(CompletePath, Path);
    U32 FileSizeU32 = 0;
    U16 PacketSizeU16 = atoi(PacketSize);
    I32 ClientStatus = 0, Time1 = 0, Time2 = 0, TimeDiff = 0, i = 0, j = 0;
    C8 TxBuffer[SYSTEM_CONTROL_TX_PACKET_SIZE];
    U32 TotalRxCount = 0, TransmissionCount = 0, RestCount = 0;
    struct timeval CurTime;
    struct stat st;
     
    stat(CompletePath, &st);
    TransmissionCount = (U32)(st.st_size) / PacketSizeU16;
    RestCount = (U32)(st.st_size) % PacketSizeU16;

    if(Debug)
    {
        printf("%s\n", Path);
        //printf("%s\n", FileSize);
        printf("%s\n", PacketSize);
        printf("%s\n", CompletePath);
    }

    fd = fopen(CompletePath, "r");
   
    if(fd != NULL)
    {

        for(i = 0; i < TransmissionCount; i++)
        {
            bzero(TxBuffer, PacketSizeU16);
            fread(TxBuffer,1,PacketSizeU16,fd);
            SystemControlSendBytes(TxBuffer, PacketSizeU16, sockfd, 0); //Send a packet
        }

        if(RestCount > 0)
        {
            bzero(TxBuffer, PacketSizeU16);
            fread(TxBuffer,1,RestCount,fd);
            SystemControlSendBytes(TxBuffer, RestCount, sockfd, 0); //Send the rest
        }
               
        fclose(fd);

        if(Remove) remove(CompletePath);
        
        printf("[SystemControl] Sent file: %s, total size = %d, transmissions = %d\n", CompletePath, (PacketSizeU16*TransmissionCount+RestCount), i + 1);

    }
 
    return 0;
}
