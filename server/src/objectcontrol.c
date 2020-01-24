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
#include <signal.h>
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
#include "iso22133.h"



/*------------------------------------------------------------
  -- Defines
  ------------------------------------------------------------*/

// Macro for determining individual struct member sizes
#define member_sizeof(type, member) sizeof(((type *)0)->member)

#define LOCALHOST "127.0.0.1"

#define RECV_MESSAGE_BUFFER 6200
#define BUFFER_SIZE_3100 3100
#define TRAJ_FILE_HEADER_ROW 256
#define OBJECT_MESS_BUFFER_SIZE 1024

#define OC_SLEEP_TIME_EMPTY_MQ_S 0
#define OC_SLEEP_TIME_EMPTY_MQ_NS 1000000
#define OC_SLEEP_TIME_NONEMPTY_MQ_S 0
#define OC_SLEEP_TIME_NONEMPTY_MQ_NS 0


#define TASK_PERIOD_MS 1
#define HEARTBEAT_TIME_MS 10
#define OBJECT_CONTROL_CONTROL_MODE 0
#define OBJECT_CONTROL_REPLAY_MODE 1
#define OBJECT_CONTROL_ABORT_MODE 1

#define OC_STATE_REPORT_PERIOD_S 1
#define OC_STATE_REPORT_PERIOD_US 0

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



#define ASP_MESSAGE_LENGTH sizeof(ASPType)

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
typedef StateTransitionResult(*StateTransition) (OBCState_t * currentState, OBCState_t requestedState);

C8 TrajBuffer[COMMAND_DOTM_ROWS_IN_TRANSMISSION * COMMAND_DOTM_ROW_MESSAGE_LENGTH +
			  COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH];


/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static I32 vConnectObject(int *sockfd, const char *name, const uint32_t port, U8 * Disconnect);
static void vDisconnectObject(int *sockfd);
static I32 vCheckRemoteDisconnected(int *sockfd);

static void vCreateSafetyChannel(const char *name, const uint32_t port, int *sockfd,
								 struct sockaddr_in *addr);
static void vCloseSafetyChannel(int *sockfd);
I32 ObjectControlBuildOSEMMessage(C8 * MessageBuffer, OSEMType * OSEMData, TimeType * GPSTime, C8 * Latitude,
								  C8 * Longitude, C8 * Altitude, U8 debug);
static size_t uiRecvMonitor(int *sockfd, char *buffer, size_t length);
static int iGetObjectIndexFromObjectIP(in_addr_t ipAddr, in_addr_t objectIPs[], unsigned int numberOfObjects);
static void signalHandler(int signo);
I32 ObjectControlBuildOSTMMessage(C8 * MessageBuffer, OSTMType * OSTMData, C8 CommandOption, U8 debug);
I32 ObjectControlBuildHEABMessage(C8 * MessageBuffer, HEABType * HEABData, TimeType * GPSTime, U8 CCStatus,
								  U8 debug);
int ObjectControlBuildLLCMMessage(char *MessageBuffer, unsigned short Speed, unsigned short Curvature,
								  unsigned char Mode, char debug);
I32 ObjectControlBuildSYPMMessage(C8 * MessageBuffer, SYPMType * SYPMData, U32 SyncPoint, U32 StopTime,
								  U8 debug);
I32 ObjectControlBuildMTSPMessage(C8 * MessageBuffer, MTSPType * MTSPData, U32 SyncTimestamp, U8 debug);
I32 ObjectControlBuildTRAJMessageHeader(C8 * MessageBuffer, I32 * RowCount, HeaderType * HeaderData,
										TRAJInfoType * TRAJInfoData, C8 * TrajFileHeader, U8 debug);
I32 ObjectControlBuildTRAJMessage(C8 * MessageBuffer, FILE * fd, I32 RowCount, DOTMType * DOTMData, U8 debug);
I32 ObjectControlSendTRAJMessage(C8 * Filename, I32 * Socket, I32 RowCount, C8 * IP, U32 Port,
								 DOTMType * DOTMData, U8 debug);
int ObjectControlSendUDPData(int *sockfd, struct sockaddr_in *addr, char *SendData, int Length, char debug);
I32 ObjectControlBuildVOILMessage(C8 * MessageBuffer, VOILType * VOILData, C8 * SimData, U8 debug);
I32 ObjectControlSendDTMMessage(C8 * DTMData, I32 * Socket, I32 RowCount, C8 * IP, U32 Port,
								DOTMType * DOTMData, U8 debug);
I32 ObjectControlBuildDTMMessage(C8 * MessageBuffer, C8 * DTMData, I32 RowCount, DOTMType * DOTMData,
								 U8 debug);
I32 ObjectControlBuildASPMessage(C8 * MessageBuffer, ASPType * ASPData, U8 debug);
I32 ObjectControlBuildACCMMessage(ACCMData * mqACCMData, ACCMType * ACCM, U8 debug);
I32 ObjectControlBuildEXACMessage(EXACData * mqEXACData, EXACType * EXAC, U8 debug);
I32 ObjectControlBuildTRCMMessage(TRCMData * mqTRCMData, TRCMType * TRCM, U8 debug);
I32 ObjectControlSendACCMMessage(ACCMData * ACCM, I32 * socket, U8 debug);
I32 ObjectControlSendTRCMMessage(TRCMData * TRCM, I32 * socket, U8 debug);
I32 ObjectControlSendEXACMessage(EXACData * EXAC, I32 * socket, U8 debug);

static int iFindObjectsInfo(C8 object_traj_file[MAX_OBJECTS][MAX_FILE_PATH],
							C8 object_address_name[MAX_OBJECTS][MAX_FILE_PATH],
							in_addr_t objectIPs[MAX_OBJECTS], I32 * nbr_objects);

OBCState_t vInitializeState(OBCState_t firstState, GSDType * GSD);
inline OBCState_t vGetState(GSDType * GSD);
StateTransitionResult vSetState(OBCState_t requestedState, GSDType * GSD);
StateTransition tGetTransition(OBCState_t fromState);
StateTransitionResult tFromIdle(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromInitialized(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromConnected(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromArmed(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromRunning(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromError(OBCState_t * currentState, OBCState_t requestedState);
StateTransitionResult tFromUndefined(OBCState_t * currentState, OBCState_t requestedState);

/*------------------------------------------------------------
-- Private variables
------------------------------------------------------------*/

#define MODULE_NAME "ObjectControl"
static volatile int iExit = 0;

/*------------------------------------------------------------
  -- Public functions
  ------------------------------------------------------------*/

void objectcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel) {
	I32 safety_socket_fd[MAX_OBJECTS];
	struct sockaddr_in safety_object_addr[MAX_OBJECTS];
	I32 socket_fds[MAX_OBJECTS];
	I32 socket_fd = 0;
	C8 object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
	C8 object_address_name[MAX_OBJECTS][MAX_FILE_PATH];
	in_addr_t objectIPs[MAX_OBJECTS];
	U32 object_udp_port[MAX_OBJECTS];
	U32 object_tcp_port[MAX_OBJECTS];
	I32 nbr_objects = 0;
	enum COMMAND iCommand;
	U8 pcRecvBuffer[RECV_MESSAGE_BUFFER];
	C8 pcTempBuffer[512];
	C8 MessageBuffer[BUFFER_SIZE_3100];
	I32 iIndex = 0, i = 0;
	struct timespec sleep_time, ref_time;

	/*! Timers for reporting state over message bus */
	const struct timespec mqEmptyPollPeriod = { OC_SLEEP_TIME_EMPTY_MQ_S, OC_SLEEP_TIME_EMPTY_MQ_NS };
	const struct timespec mqNonEmptyPollPeriod =
		{ OC_SLEEP_TIME_NONEMPTY_MQ_S, OC_SLEEP_TIME_NONEMPTY_MQ_NS };
	const struct timeval stateReportPeriod = { OC_STATE_REPORT_PERIOD_S, OC_STATE_REPORT_PERIOD_US };
	struct timeval currentTime, nextStateReportTime;
	U8 iForceObjectToLocalhostU8 = 0;

	FILE *fd;
	C8 Timestamp[SMALL_BUFFER_SIZE_0];
	C8 GPSWeek[SMALL_BUFFER_SIZE_0];
	I32 MessageLength;
	C8 *MiscPtr;
	C8 MiscText[SMALL_BUFFER_SIZE_0];
	U32 StartTimeU32 = 0;
	U32 CurrentTimeU32 = 0;
	U32 OldTimeU32 = 0;
	U64 TimeCap1, TimeCap2;
	struct timeval CurrentTimeStruct;
	I32 HeartbeatMessageCounter = 0;

	ObjectPosition OP[MAX_OBJECTS];
	flt SpaceArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
	flt TimeArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
	SpaceTime SpaceTimeArr[MAX_OBJECTS][TRAJECTORY_FILE_MAX_ROWS];
	C8 OriginLatitude[SMALL_BUFFER_SIZE_0], OriginLongitude[SMALL_BUFFER_SIZE_0],
		OriginAltitude[SMALL_BUFFER_SIZE_0], OriginHeading[SMALL_BUFFER_SIZE_0];
	C8 TextBuffer[SMALL_BUFFER_SIZE_0];
	dbl OriginLatitudeDbl;
	dbl OriginLongitudeDbl;
	dbl OriginAltitudeDbl;
	dbl OriginHeadingDbl;
	C8 pcSendBuffer[MBUS_MAX_DATALEN];
	C8 ObjectPort[SMALL_BUFFER_SIZE_0];
	HeaderType HeaderData;
	OSEMType OSEMData;
	OSTMType OSTMData;
	HEABType HEABData;
	MONRType MONRData;
	DOTMType DOTMData;
	TRAJInfoType TRAJInfoData;
	VOILType VOILData;
	SYPMType SYPMData;
	MTSPType MTSPData;
	ACCMData mqACCMData;
	EXACData mqEXACData;
	GeoPosition OriginPosition;
	ASPType ASPData;

	ASPData.MTSPU32 = 0;
	ASPData.TimeToSyncPointDbl = 0;
	ASPData.PrevTimeToSyncPointDbl = 0;
	ASPData.CurrentTimeDbl = 0;
	AdaptiveSyncPoint ASP[MAX_ADAPTIVE_SYNC_POINTS];
	I32 SyncPointCount = 0;
	I32 SearchStartIndex = 0;
	dbl ASPMaxTimeDiffDbl = 0;
	dbl ASPMaxTrajDiffDbl = 0;
	dbl ASPFilterLevelDbl = 0;
	dbl ASPMaxDeltaTimeDbl = 0;
	I32 ASPDebugRate = 1;
	I32 ASPStepBackCount = 0;
	char confDirectoryPath[MAX_FILE_PATH];

	U8 ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_BOOTING;

	vInitializeState(OBC_STATE_IDLE, GSD);
	U8 uiTimeCycle = 0;
	I32 ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

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

	U8 STRTSentU8 = 0;
	C8 FileHeaderBufferC8[TRAJ_FILE_HEADER_ROW];


	// Create log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Object control task running with PID: %i", getpid());


	// Set up signal handlers
	if (signal(SIGINT, signalHandler) == SIG_ERR)
		util_error("Unable to initialize signal handler");

	// Set up message bus connection
	if (iCommInit())
		util_error("Unable to connect to message queue bus");

	// Initialize timer for sending state
	TimeSetToCurrentSystemTime(&currentTime);
	nextStateReportTime = currentTime;

	// Initialize timer for sending state
	TimeSetToCurrentSystemTime(&currentTime);
	nextStateReportTime = currentTime;

	while (!iExit) {

		if (vGetState(GSD) == OBC_STATE_ERROR) {
			ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
			MessageLength =
				ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, GPSTime, ObjectControlServerStatus,
											  0);
			UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex],
							MessageBuffer, MessageLength, 0);
		}

		if (vGetState(GSD) == OBC_STATE_RUNNING || vGetState(GSD) == OBC_STATE_ARMED
			|| vGetState(GSD) == OBC_STATE_CONNECTED) {
			 /*HEAB*/ for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				if (uiTimeCycle == 0) {
					//HeartbeatMessageCounter ++;
					MessageLength =
						ObjectControlBuildHEABMessage(MessageBuffer, &HEABData, GPSTime,
													  ObjectControlServerStatus, 0);
					//ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 1);
					UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex],
									MessageBuffer, MessageLength, 0);
				}
			}


			// Check if any object has disconnected - if so, disconnect all objects and return to idle
			DisconnectU8 = 0;
			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				DisconnectU8 |= vCheckRemoteDisconnected(&socket_fds[iIndex]);
				if (DisconnectU8) {
					LogMessage(LOG_LEVEL_WARNING, "Lost connection to IP %s - returning to IDLE",
							   object_address_name[iIndex]);

					for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
						vDisconnectObject(&socket_fds[iIndex]);
					}

					/* Close safety socket */
					for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
						vCloseSafetyChannel(&safety_socket_fd[iIndex]);
					}
					vSetState(OBC_STATE_IDLE, GSD);
					break;
				}
			}
		}

		if (vGetState(GSD) == OBC_STATE_RUNNING || vGetState(GSD) == OBC_STATE_CONNECTED
			|| vGetState(GSD) == OBC_STATE_ARMED) {
			char buffer[RECV_MESSAGE_BUFFER];
			size_t receivedMONRData = 0;

			// this is etsi time lets remov it ans use utc instead
			//gettimeofday(&CurrentTimeStruct, NULL);

			CurrentTimeU32 =
				((GPSTime->GPSSecondsOfWeekU32 * 1000 + (U32) TimeControlGetMillisecond(GPSTime)) << 2) +
				GPSTime->MicroSecondU16;


			 /*MTSP*/ if (HeartbeatMessageCounter == 0) {
				HeartbeatMessageCounter = 0;
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					for (i = 0; i < SyncPointCount; i++) {
						if (TEST_SYNC_POINTS == 0
							&& strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL
							&& ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1) {
							/*Send Master time to adaptive sync point */
							MessageLength =
								ObjectControlBuildMTSPMessage(MessageBuffer, &MTSPData, ASPData.MTSPU32, 0);
							//ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
							UtilSendUDPData("Object Control", &safety_socket_fd[iIndex],
											&safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
						}
						/*else if(TEST_SYNC_POINTS == 1 && iIndex == 1 && ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1 )
						   {
						   Send Master time to adaptive sync point
						   MessageLength =ObjectControlBuildMTSPMessage(MessageBuffer, &MTSPData, (U32)ASPData.MTSPU32, 0);
						   ObjectControlSendUDPData(&safety_socket_fd[iIndex], &safety_object_addr[iIndex], MessageBuffer, MessageLength, 0);
						   } */
					}
				}
			}


			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				memset(buffer, 0, sizeof (buffer));
				receivedMONRData = uiRecvMonitor(&safety_socket_fd[iIndex], buffer, sizeof (buffer));

				if (receivedMONRData > 0) {
					LogMessage(LOG_LEVEL_DEBUG, "Recieved new data from %s %d %d: %s",
							   object_address_name[iIndex], object_udp_port[iIndex], receivedMONRData,
							   buffer);

					if (decodeMONRMessage(buffer, receivedMONRData, &MONRData, 0) != MESSAGE_OK) {
						LogMessage(LOG_LEVEL_INFO, "Error decoding MONR from %s: disconnecting object",
								   object_address_name[iIndex]);
						vDisconnectObject(&safety_socket_fd[iIndex]);
						// TODO smarter way of handling?
						continue;
					}

					if (ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) {
						// Append IP to buffer
						memcpy(&buffer[receivedMONRData], &safety_object_addr[iIndex].sin_addr.s_addr,
							   sizeof (in_addr_t));
						// Send MONR message as bytes

						if (iCommSend(COMM_MONR, buffer, (size_t) (receivedMONRData) + sizeof (in_addr_t)) <
							0) {
							LogMessage(LOG_LEVEL_ERROR,
									   "Fatal communication fault when sending MONR command - entering error state");
							vSetState(OBC_STATE_ERROR, GSD);
							ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
						}
					}


					//Store MONR in GSD
					//UtilSendUDPData("ObjectControl", &ObjectControlUDPSocketfdI32, &simulator_addr, &MONRData, sizeof(MONRData), 0);
					for (i = 0;
						 i <
						 (MONRData.header.MessageLengthU32 + sizeof (MONRData.header) +
						  sizeof (MONRData.footer)); i++)
						GSD->MONRData[i] = buffer[i];
					GSD->MONRSizeU8 =
						MONRData.header.MessageLengthU32 + sizeof (MONRData.header) +
						sizeof (MONRData.footer);
					memset(buffer, 0, sizeof (buffer));
					memcpy(buffer, object_address_name[iIndex], strlen(object_address_name[iIndex]));
					strcat(buffer, ";0;");
					MONRToASCII(&MONRData, buffer + strlen(buffer), sizeof (buffer) - strlen(buffer), 0);


					if (ASPData.MTSPU32 != 0) {
						//Add MTSP to MONR if not 0
						sprintf(buffer + strlen(buffer), "%" PRIu32 ";", ASPData.MTSPU32);
					}

					//Ok, let's do the ASP
					for (i = 0; i < SyncPointCount; i++) {
						if (TEST_SYNC_POINTS == 0
							&& strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL
							&& CurrentTimeU32 > StartTimeU32 && StartTimeU32 > 0
							&& ASPData.TimeToSyncPointDbl > -1
							/*|| TEST_SYNC_POINTS == 1 && ASP[0].TestPort == object_udp_port[iIndex] && StartTimeU32 > 0 && iIndex == 0 && TimeToSyncPoint > -1 */
							) {
							// Use the util.c function for time here but it soent mather
							gettimeofday(&CurrentTimeStruct, NULL);	//Capture time

							TimeCap1 = (uint64_t) CurrentTimeStruct.tv_sec * 1000 + (uint64_t) CurrentTimeStruct.tv_usec / 1000;	//Calculate initial timestamp

							OP[iIndex].x = ((double)MONRData.xPosition) / 1000;	//Set x and y on OP (ObjectPosition)
							OP[iIndex].y = ((double)MONRData.yPosition) / 1000;

							//OP[iIndex].OrigoDistance = sqrt(pow(OP[iIndex].x,2) + pow(OP[iIndex].y,2)); //Calculate hypotenuse

							// TODO: check use of this function since it should take two lat/long points but is here used with x/y
							UtilCalcPositionDelta(OriginLatitudeDbl, OriginLongitudeDbl,
												  MONRData.xPosition / 1e7, MONRData.yPosition / 1e7,
												  &OP[iIndex]);

							if (OP[iIndex].BestFoundTrajectoryIndex <= OP[iIndex].SyncIndex) {
								ASPData.CurrentTimeU32 = CurrentTimeU32;
								ASPData.CurrentTimeDbl =
									(((double)CurrentTimeU32 - (double)StartTimeU32) / 1000);
								SearchStartIndex = OP[iIndex].BestFoundTrajectoryIndex - ASPStepBackCount;
								UtilFindCurrentTrajectoryPosition(&OP[iIndex], SearchStartIndex,
																  ASPData.CurrentTimeDbl, ASPMaxTrajDiffDbl,
																  ASPMaxTimeDiffDbl, 0);
								ASPData.BestFoundIndexI32 = OP[iIndex].BestFoundTrajectoryIndex;

								if (OP[iIndex].BestFoundTrajectoryIndex != TRAJ_POSITION_NOT_FOUND) {
									ASPData.TimeToSyncPointDbl = UtilCalculateTimeToSync(&OP[iIndex]);
									if (ASPData.TimeToSyncPointDbl > 0) {
										if (ASPData.PrevTimeToSyncPointDbl != 0 && ASPFilterLevelDbl > 0) {
											if (ASPData.TimeToSyncPointDbl / ASPData.PrevTimeToSyncPointDbl >
												(1 + ASPFilterLevelDbl / 100))
												ASPData.TimeToSyncPointDbl = ASPData.PrevTimeToSyncPointDbl + ASPMaxDeltaTimeDbl;	//TimeToSyncPoint*ASPFilterLevelDbl/500;
											else if (ASPData.TimeToSyncPointDbl /
													 ASPData.PrevTimeToSyncPointDbl <
													 (1 - ASPFilterLevelDbl / 100))
												ASPData.TimeToSyncPointDbl = ASPData.PrevTimeToSyncPointDbl - ASPMaxDeltaTimeDbl;	//TimeToSyncPoint*ASPFilterLevelDbl/500;
										}
										ASPData.MTSPU32 =
											CurrentTimeU32 + (U32) (ASPData.TimeToSyncPointDbl * 4000);

										ASPData.PrevTimeToSyncPointDbl = ASPData.TimeToSyncPointDbl;
										OldTimeU32 = CurrentTimeU32;
									}
									else {
										CurrentTimeU32 = 0;
										ASPData.TimeToSyncPointDbl = -1;
									}

								}

								gettimeofday(&CurrentTimeStruct, NULL);
								TimeCap2 =
									(uint64_t) CurrentTimeStruct.tv_sec * 1000 +
									(uint64_t) CurrentTimeStruct.tv_usec / 1000;

								ASPData.SyncPointIndexI32 = OP[iIndex].SyncIndex;
								ASPData.IterationTimeU16 = (U16) (TimeCap2 - TimeCap1);
								//Build ASP debug data and set to GSD
								//bzero(buffer,OBJECT_MESS_BUFFER_SIZE);
								//ObjectControlBuildASPMessage(buffer, &ASPData, 0);
								DataDictionarySetRVSSAsp(GSD, &ASPData);

								if (MONRData.gpsQmsOfWeek % ASPDebugRate == 0) {
									printf("%d, %d, %3.3f, %s, %s\n", CurrentTimeU32, StartTimeU32,
										   ASPData.TimeToSyncPointDbl, object_address_name[iIndex],
										   ASP[i].MasterIP);
									printf
										("TtS=%3.3f, BestIndex=%d, MTSP=%d, iIndex=%d, IterationTime=%3.0f ms\n",
										 ASPData.TimeToSyncPointDbl, OP[iIndex].BestFoundTrajectoryIndex,
										 ASPData.MTSPU32, iIndex, ((double)(TimeCap2) - (double)TimeCap1));
									printf("CurrentTime=%3.3f, x=%3.3f mm, y=%3.3f\n\n",
										   ASPData.CurrentTimeDbl, OP[iIndex].x, OP[iIndex].y);

									//Build and send ASP on message queue
									//(void)iCommSend(COMM_ASP,buffer);
								}
							}
						}
					}
					OP[iIndex].Speed =
						sqrt(pow(MONRData.lateralSpeed, 2) + pow(MONRData.longitudinalSpeed, 2));
				}
				else if (receivedMONRData > 0)
					LogMessage(LOG_LEVEL_INFO, "MONR length error (should be %d but is %ld) from %s.",
							   sizeof (MONRType), object_address_name[iIndex]);
			}
		}


		bzero(pcRecvBuffer, RECV_MESSAGE_BUFFER);
		//Have we recieved a command?
		if (iCommRecv(&iCommand, pcRecvBuffer, RECV_MESSAGE_BUFFER, NULL)) {
			LogMessage(LOG_LEVEL_INFO, "Received command %d", iCommand);


			if (iCommand == COMM_ARM && vGetState(GSD) == OBC_STATE_CONNECTED) {

				LogMessage(LOG_LEVEL_INFO, "Sending ARM");
				LOG_SEND(LogBuffer, "[ObjectControl] Sending ARM");
				vSetState(OBC_STATE_ARMED, GSD);
				MessageLength =
					ObjectControlBuildOSTMMessage(MessageBuffer, &OSTMData, COMMAND_OSTM_OPT_SET_ARMED_STATE,
												  0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					UtilSendTCPData("[Object Control]", MessageBuffer, MessageLength, &socket_fds[iIndex], 0);
				}

				ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;	//Set server to READY
			}
			else if (iCommand == COMM_DISARM && vGetState(GSD) == OBC_STATE_ARMED) {

				LogMessage(LOG_LEVEL_INFO, "Sending DISARM");
				LOG_SEND(LogBuffer, "[ObjectControl] Sending DISARM");
				vSetState(OBC_STATE_CONNECTED, GSD);

				MessageLength =
					ObjectControlBuildOSTMMessage(MessageBuffer, &OSTMData,
												  COMMAND_OSTM_OPT_SET_DISARMED_STATE, 0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					UtilSendTCPData("[" MODULE_NAME "]", MessageBuffer, MessageLength, &socket_fds[iIndex],
									0);
				}

				ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;	//Set server to READY
			}
			else if (iCommand == COMM_STRT && (vGetState(GSD) == OBC_STATE_ARMED) /*|| OBC_STATE_INITIALIZED) */ )	//OBC_STATE_INITIALIZED is temporary!
			{
				struct timeval startTime, startDelay;

				MiscPtr = pcRecvBuffer;
				TimeSetToUTCms(&startTime, (int64_t) strtoul(MiscPtr, &MiscPtr, 10));
				TimeSetToUTCms(&startDelay, (int64_t) strtoul(MiscPtr + 1, NULL, 10));
				timeradd(&startTime, &startDelay, &startTime);
				MessageLength =
					(int)encodeSTRTMessage(TimeGetAsGPSqmsOfWeek(&startTime), TimeGetAsGPSweek(&startTime),
										   MessageBuffer, sizeof (MessageBuffer), 0);

				ASPData.MTSPU32 = 0;
				ASPData.TimeToSyncPointDbl = 0;
				SearchStartIndex = -1;
				ASPData.PrevTimeToSyncPointDbl = 0;
				OldTimeU32 = CurrentTimeU32;
				ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;	//Set server to READY

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fds[iIndex], 0);
				}
				vSetState(OBC_STATE_RUNNING, GSD);

				//Store STRT in GSD
				if (STRTSentU8 == 0) {
					//for(i = 0; i < MessageLength; i++) GSD->STRTData[i] = MessageBuffer[i];
					//GSD->STRTSizeU8 = (U8)MessageLength;
					STRTSentU8 = 1;
				}
				//OBCState = OBC_STATE_INITIALIZED; //This is temporary!
				//printf("OutgoingStartTimeU32 = %d\n", OutgoingStartTimeU32);
				GSD->ScenarioStartTimeU32 = TimeGetAsGPSqmsOfWeek(&startTime) >> 2;
				bzero(MiscText, SMALL_BUFFER_SIZE_0);
				sprintf(MiscText, "%" PRIu32, GSD->ScenarioStartTimeU32 << 2);
				LOG_SEND(LogBuffer, "[ObjectControl] START received <%s>, GPS time <%s>", pcRecvBuffer,
						 MiscText);
			}
			else if (iCommand == COMM_REPLAY) {
				ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
				LogMessage(LOG_LEVEL_INFO, "Entering REPLAY mode <%s>", pcRecvBuffer);
			}
			else if (iCommand == COMM_ABORT && vGetState(GSD) == OBC_STATE_RUNNING) {
				vSetState(OBC_STATE_CONNECTED, GSD);
				ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;	//Set server to ABORT
				LogMessage(LOG_LEVEL_WARNING, "ABORT received");
				LOG_SEND(LogBuffer, "[ObjectControl] ABORT received.");
			}
			else if (iCommand == COMM_CONTROL) {
				ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
				printf("[ObjectControl] Object control in CONTROL mode\n");
			}
			else if (iCommand == COMM_INIT) {
				LogMessage(LOG_LEVEL_INFO, "INIT received");
				LOG_SEND(LogBuffer, "[ObjectControl] INIT received.");
				nbr_objects = 0;
				if (iFindObjectsInfo(object_traj_file, object_address_name, objectIPs, &nbr_objects) == 0) {
					// Get objects; name and drive file
					DataDictionaryGetForceToLocalhostU8(GSD, &iForceObjectToLocalhostU8);

					for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
						if (0 == iForceObjectToLocalhostU8) {
							object_udp_port[iIndex] = SAFETY_CHANNEL_PORT;
							object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT;
						}
						else {
							object_udp_port[iIndex] = SAFETY_CHANNEL_PORT + iIndex * 2;
							object_tcp_port[iIndex] = CONTROL_CHANNEL_PORT + iIndex * 2;
						}
					}

					/*Setup Adaptive Sync Points (ASP) */
					UtilGetConfDirectoryPath(confDirectoryPath, sizeof (confDirectoryPath));
					strcat(confDirectoryPath, ADAPTIVE_SYNC_FILE_NAME);
					fd = fopen(confDirectoryPath, "r");
					if (fd) {
						SyncPointCount = UtilCountFileRows(fd) - 1;
						fclose(fd);
						fd = fopen(confDirectoryPath, "r");
						UtilReadLineCntSpecChars(fd, pcTempBuffer);	//Read header

						for (i = 0; i < SyncPointCount; i++) {
							UtilSetAdaptiveSyncPoint(&ASP[i], fd, 0);
							if (TEST_SYNC_POINTS == 1)
								ASP[i].TestPort = SAFETY_CHANNEL_PORT;
						}
						fclose(fd);
					}

					vSetState(OBC_STATE_INITIALIZED, GSD);
					LogMessage(LOG_LEVEL_INFO, "ObjectControl is initialized");
					LOG_SEND(LogBuffer, "[ObjectControl] ObjectControl is initialized.");

					//Remove temporary file
					remove(TEMP_LOG_FILE);
					if (USE_TEMP_LOGFILE) {
						//Create temporary file
						TempFd = fopen(TEMP_LOG_FILE, "w+");
					}

					//OSEMSentU8 = 0;
					STRTSentU8 = 0;
				}
				else {
					LogMessage(LOG_LEVEL_INFO,
							   "Could not initialize: object info was not processed successfully");
					pcSendBuffer[0] = (uint8_t) iCommand;
					iCommSend(COMM_FAILURE, pcSendBuffer, sizeof (iCommand));
				}
			}
			else if (iCommand == COMM_ACCM && vGetState(GSD) == OBC_STATE_CONNECTED) {
				UtilPopulateACCMDataStructFromMQ(pcRecvBuffer, sizeof (pcRecvBuffer), &mqACCMData);
				iIndex =
					iGetObjectIndexFromObjectIP(mqACCMData.ip, objectIPs,
												sizeof (objectIPs) / sizeof (objectIPs[0]));
				if (iIndex != -1) {
					ObjectControlSendACCMMessage(&mqACCMData, &(socket_fds[iIndex]), 0);
				}
				else
					LogMessage(LOG_LEVEL_WARNING, "Unable to send ACCM: no valid socket found");
			}
			else if (iCommand == COMM_EXAC && vGetState(GSD) == OBC_STATE_RUNNING) {
				UtilPopulateEXACDataStructFromMQ(pcRecvBuffer, sizeof (pcRecvBuffer), &mqEXACData);
				iIndex =
					iGetObjectIndexFromObjectIP(mqEXACData.ip, objectIPs,
												sizeof (objectIPs) / sizeof (objectIPs[0]));
				if (iIndex != -1)
					ObjectControlSendEXACMessage(&mqEXACData, &(socket_fds[iIndex]), 0);
				else
					LogMessage(LOG_LEVEL_WARNING, "Unable to send EXAC: no valid socket found");
			}
			else if (iCommand == COMM_CONNECT && vGetState(GSD) == OBC_STATE_INITIALIZED) {
				LogMessage(LOG_LEVEL_INFO, "CONNECT received");
				LOG_SEND(LogBuffer, "[ObjectControl] CONNECT received.");

				/* Connect and send drive files */
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {

					UtilSetObjectPositionIP(&OP[iIndex], object_address_name[iIndex]);

					MessageLength = ObjectControlBuildOSEMMessage(MessageBuffer, &OSEMData, GPSTime,
																  OriginLatitude, OriginLongitude,
																  OriginAltitude, 0);

					DisconnectU8 = 0;

					do {

						iResult =
							vConnectObject(&socket_fds[iIndex], object_address_name[iIndex],
										   object_tcp_port[iIndex], &DisconnectU8);

						if (iResult < 0) {
							switch (errno) {
							case ECONNREFUSED:
								LogMessage(LOG_LEVEL_INFO,
										   "Unable to connect to object %s:%d, retry in %d sec...",
										   object_address_name[iIndex], object_tcp_port[iIndex],
										   (!(1 & DisconnectU8)) * 3);
								LOG_SEND(LogBuffer,
										 "[ObjectControl] Was not able to connect to object, [IP: %s] [PORT: %d], retry in %d sec...",
										 object_address_name[iIndex], object_tcp_port[iIndex],
										 (!(1 & DisconnectU8)) * 3);
								(void)sleep(3);	// TODO: Move this to the rest of the sleep operations? Also, remove the hardcoded 3
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

						bzero(pcRecvBuffer, RECV_MESSAGE_BUFFER);
						//Have we received a command?
						if (iCommRecv(&iCommand, pcRecvBuffer, RECV_MESSAGE_BUFFER, NULL)) {
							if (iCommand == COMM_DISCONNECT) {
								DisconnectU8 = 1;
								LOG_SEND(LogBuffer, "[ObjectControl] DISCONNECT received.");
							}

						}

					} while (iResult < 0 && DisconnectU8 == 0);

					if (iResult >= 0) {
						/* Send OSEM command in mq so that we get some information like GPSweek, origin (latitude,logitude,altitude in gps coordinates) */
						LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
						LOG_SEND(LogBuffer, "[ObjectControl] Sending OSEM.");

						ObjectControlOSEMtoASCII(&OSEMData, GPSWeek, OriginLatitude, OriginLongitude,
												 OriginAltitude);
						bzero(pcSendBuffer, sizeof (pcSendBuffer));
						strcat(pcSendBuffer, GPSWeek);
						strcat(pcSendBuffer, ";");
						strcat(pcSendBuffer, OriginLatitude);
						strcat(pcSendBuffer, ";");
						strcat(pcSendBuffer, OriginLongitude);
						strcat(pcSendBuffer, ";");
						strcat(pcSendBuffer, OriginAltitude);

						//Restore the buffers
						DataDictionaryGetOriginLatitudeC8(GSD, OriginLatitude, SMALL_BUFFER_SIZE_0);
						DataDictionaryGetOriginLongitudeC8(GSD, OriginLongitude, SMALL_BUFFER_SIZE_0);
						DataDictionaryGetOriginAltitudeC8(GSD, OriginAltitude, SMALL_BUFFER_SIZE_0);

						if (iCommSend(COMM_OSEM, pcSendBuffer, strlen(pcSendBuffer) + 1) < 0) {
							LogMessage(LOG_LEVEL_ERROR,
									   "Fatal communication fault when sending OSEM command - entering error state");
							vSetState(OBC_STATE_ERROR, GSD);
							ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
						}
						UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fds[iIndex],
										0);

						/*Here we send TRAJ, if the IP-address not is found */
						if (strstr(DTMReceivers, object_address_name[iIndex]) == NULL) {

							fd = fopen(object_traj_file[iIndex], "r");

							if (fd != NULL) {
								//RowCount = UtilCountFileRows(fd);
								//printf("RowCount: %d\n", RowCount);
								//fclose(fd);

								//fd = fopen(object_traj_file[iIndex], "r");
								//printf("Open file: %s\n", object_traj_file[iIndex]);
								UtilReadLineCntSpecChars(fd, FileHeaderBufferC8);
								fclose(fd);

								 /*TRAJ*/
									MessageLength = ObjectControlBuildTRAJMessageHeader(TrajBuffer,
																						&RowCount,
																						&HeaderData,
																						&TRAJInfoData,
																						FileHeaderBufferC8,
																						0);

								//printf("RowCount: %d\n", RowCount);

								/*Send TRAJ header */
								UtilSendTCPData("Object Control", TrajBuffer, MessageLength,
												&socket_fds[iIndex], 0);

								/*Send TRAJ data */
								ObjectControlSendTRAJMessage(object_traj_file[iIndex], &socket_fds[iIndex],
															 RowCount,
															 (char *)&object_address_name[iIndex],
															 object_tcp_port[iIndex], &DOTMData, 0);

							}
							else
								LogMessage(LOG_LEVEL_WARNING, "Could not open file <%s>",
										   object_traj_file[iIndex]);
						}


						/* Adaptive Sync Points object configuration start... */
						if (TEST_SYNC_POINTS == 1)
							printf("Trajfile: %s\n", object_traj_file[iIndex]);
						OP[iIndex].TrajectoryPositionCount = RowCount;
						OP[iIndex].SpaceArr = SpaceArr[iIndex];
						OP[iIndex].TimeArr = TimeArr[iIndex];
						OP[iIndex].SpaceTimeArr = SpaceTimeArr[iIndex];
						UtilPopulateSpaceTimeArr(&OP[iIndex], object_traj_file[iIndex]);

						LogMessage(LOG_LEVEL_INFO, "Sync point counts: %d", SyncPointCount);
						for (i = 0; i < SyncPointCount; i++) {
							if (TEST_SYNC_POINTS == 1 && iIndex == 1) {
								/*Send SYPM to slave */
								MessageLength =
									ObjectControlBuildSYPMMessage(MessageBuffer, &SYPMData,
																  ASP[i].SlaveTrajSyncTime * 1000,
																  ASP[i].SlaveSyncStopTime * 1000, 1);
								UtilSendTCPData("Object Control", MessageBuffer, MessageLength,
												&socket_fds[iIndex], 0);
							}
							else if (TEST_SYNC_POINTS == 0
									 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL) {
								/*Send SYPM to slave */
								MessageLength =
									ObjectControlBuildSYPMMessage(MessageBuffer, &SYPMData,
																  ASP[i].SlaveTrajSyncTime * 1000,
																  ASP[i].SlaveSyncStopTime * 1000, 1);
								UtilSendTCPData("Object Control", MessageBuffer, MessageLength,
												&socket_fds[iIndex], 0);
							}
						}

						/*Set Sync point in OP */
						for (i = 0; i < SyncPointCount; i++) {
							if (TEST_SYNC_POINTS == 1 && iIndex == 0)
								UtilSetSyncPoint(&OP[iIndex], 0, 0, 0, ASP[i].MasterTrajSyncTime);
							else if (TEST_SYNC_POINTS == 0
									 && strstr(object_address_name[iIndex], ASP[i].MasterIP) != NULL)
								UtilSetSyncPoint(&OP[iIndex], 0, 0, 0, ASP[i].MasterTrajSyncTime);
						}
						/* ...end */
					}

				}
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					if (USE_TEST_HOST == 0)
						vCreateSafetyChannel(object_address_name[iIndex], object_udp_port[iIndex],
											 &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
					else if (USE_TEST_HOST == 1)
						vCreateSafetyChannel(TESTSERVER_IP, object_udp_port[iIndex],
											 &safety_socket_fd[iIndex], &safety_object_addr[iIndex]);
				}

				uiTimeCycle = 0;

				/* Execution mode */
				ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

				/*Set server status */
				ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_OK;	//Set server to READY

				if (DisconnectU8 == 0) {
					vSetState(OBC_STATE_CONNECTED, GSD);
					iCommSend(COMM_OBJECTS_CONNECTED, NULL, 0);
				}
				else if (DisconnectU8 == 1)
					vSetState(OBC_STATE_IDLE, GSD);
			}
			else if (iCommand == COMM_DATA_DICT) {

				LogMessage(LOG_LEVEL_INFO, "Updating variables from DataDictionary.");
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
				LogMessage(LOG_LEVEL_INFO, "ForceObjectToLocalhost = %d", iForceObjectToLocalhostU8);
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
			else if (iCommand == COMM_DISCONNECT) {
				//#ifndef NOTCP
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					vDisconnectObject(&socket_fds[iIndex]);
				}
				//#endif //NOTCP

				LogMessage(LOG_LEVEL_INFO, "DISCONNECT received");
				LOG_SEND(LogBuffer, "[ObjectControl] DISCONNECT received.");
				/* Close safety socket */
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					vCloseSafetyChannel(&safety_socket_fd[iIndex]);
				}
				vSetState(OBC_STATE_IDLE, GSD);
			}
			else if (iCommand == COMM_EXIT) {
				iExit = 1;
				iCommClose();
			}
			else {
				LogMessage(LOG_LEVEL_WARNING, "Unhandled command in object control: %d", iCommand);
			}
		}

		if (!iExit) {
			/* Make call periodic */
			sleep_time = iCommand == COMM_INV ? mqEmptyPollPeriod : mqNonEmptyPollPeriod;

			++uiTimeCycle;
			if (uiTimeCycle >= HEARTBEAT_TIME_MS / TASK_PERIOD_MS) {
				uiTimeCycle = 0;
			}

			// Periodically send state to signal aliveness
			TimeSetToCurrentSystemTime(&currentTime);
			if (timercmp(&currentTime, &nextStateReportTime, >)) {
				timeradd(&nextStateReportTime, &stateReportPeriod, &nextStateReportTime);

				bzero(Buffer2, sizeof (Buffer2));
				Buffer2[0] = (uint8_t) (DataDictionaryGetOBCStateU8(GSD));
				if (iCommSend(COMM_OBC_STATE, Buffer2, sizeof (Buffer2)) < 0) {
					LogMessage(LOG_LEVEL_ERROR,
							   "Fatal communication fault when sending OBC_STATE command - entering error state");
					vSetState(OBC_STATE_ERROR, GSD);
					ObjectControlServerStatus = COMMAND_HEAB_OPT_SERVER_STATUS_ABORT;
				}
			}

			(void)nanosleep(&sleep_time, &ref_time);
		}
	}

	LogMessage(LOG_LEVEL_INFO, "Object control exiting");
}


/*------------------------------------------------------------
  -- Private functions
  ------------------------------------------------------------*/

void signalHandler(int signo) {
	if (signo == SIGINT) {
		LogMessage(LOG_LEVEL_WARNING, "Caught keyboard interrupt");
		iExit = 1;
	}
	else {
		LogMessage(LOG_LEVEL_ERROR, "Caught unhandled signal");
	}
}

I32 ObjectControlBuildVOILMessage(C8 * MessageBuffer, VOILType * VOILData, C8 * SimData, U8 debug) {
	I32 MessageIndex = 0, i;
	U16 Crc = 0;
	C8 *p;
	U16 U16Data = 0;
	I16 I16Data = 0;
	U32 U32Data = 0;
	I32 I32Data = 0;

	if (debug) {
		printf("Length: %d\n", *(SimData + 3));
		for (i = 0; i < *(SimData + 3) + 4; i++)
			printf("%x-", *(SimData + i));
		printf("\n");
	}

	U16Data = (U16Data | *(SimData + 5)) << 8;
	U16Data = (U16Data | *(SimData + 4));
	U16 MessageId = U16Data;

	//printf("MessageId = %x\n", MessageId);

	U32Data = (U32Data | *(SimData + 6)) << 8;
	U32Data = (U32Data | *(SimData + 7)) << 8;
	U32Data = (U32Data | *(SimData + 8)) << 8;
	U32Data = (U32Data | *(SimData + 9));
	U32 GPSSOW = U32Data;

	//printf("GPSSOW = %x\n", GPSSOW);
	U8 DynamicWorldState = *(SimData + 10);
	U8 ObjectCount = *(SimData + 11);

	//printf("ObjectCount = %d\n", ObjectCount);

	U8 ObjectId = *(SimData + 12);
	U8 ObjectState = *(SimData + 13);

	I32Data = (I32Data | *(SimData + 14)) << 8;
	I32Data = (I32Data | *(SimData + 15)) << 8;
	I32Data = (I32Data | *(SimData + 16)) << 8;
	I32Data = (I32Data | *(SimData + 17));
	I32 XPosition = I32Data;

	I32Data = 0;
	I32Data = (I32Data | *(SimData + 18)) << 8;
	I32Data = (I32Data | *(SimData + 19)) << 8;
	I32Data = (I32Data | *(SimData + 20)) << 8;
	I32Data = (I32Data | *(SimData + 21));
	I32 YPosition = I32Data;

	I32Data = 0;
	I32Data = (I32Data | *(SimData + 22)) << 8;
	I32Data = (I32Data | *(SimData + 23)) << 8;
	I32Data = (I32Data | *(SimData + 24)) << 8;
	I32Data = (I32Data | *(SimData + 25));
	I32 ZPosition = I32Data;

	U16Data = 0;
	U16Data = (U16Data | *(SimData + 26)) << 8;
	U16Data = (U16Data | *(SimData + 27));
	U16 Heading = U16Data;

	U16Data = 0;
	U16Data = (U16Data | *(SimData + 28)) << 8;
	U16Data = (U16Data | *(SimData + 29));
	U16 Pitch = U16Data;

	U16Data = 0;
	U16Data = (U16Data | *(SimData + 30)) << 8;
	U16Data = (U16Data | *(SimData + 31));
	U16 Roll = U16Data;

	//printf("Roll = %d\n", Roll);
	I16Data = 0;
	I16Data = (I16Data | *(SimData + 32)) << 8;
	I16Data = (I16Data | *(SimData + 33));
	I16 Speed = I16Data;

	//printf("Speed = %d\n", Speed);


	bzero(MessageBuffer,
		  ObjectCount * sizeof (Sim1Type) + 6 + COMMAND_MESSAGE_FOOTER_LENGTH +
		  COMMAND_MESSAGE_HEADER_LENGTH);


	VOILData->Header.SyncWordU16 = ISO_SYNC_WORD;
	VOILData->Header.TransmitterIdU8 = 0;
	VOILData->Header.MessageCounterU8 = 0;
	VOILData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	VOILData->Header.MessageIdU16 = COMMAND_VOIL_CODE;
	VOILData->Header.MessageLengthU32 = ObjectCount * sizeof (Sim1Type) + 6 - COMMAND_MESSAGE_HEADER_LENGTH;
	VOILData->GPSQmsOfWeekU32 = GPSSOW;
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


	p = (C8 *) VOILData;
	for (i = 0; i < ObjectCount * sizeof (Sim1Type) + 6 + COMMAND_MESSAGE_HEADER_LENGTH; i++)
		*(MessageBuffer + i) = *p++;
	//Crc = crc_16((const C8*)MessageBuffer, sizeof(VOILData));
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc);
	*(MessageBuffer + i++) = (U8) (Crc >> 8);
	MessageIndex = i;

	if (debug) {
		// TODO: use byte printout from logging when it has been implemented
		printf("VOILData total length = %d bytes (header+message+footer)\n",
			   (int)(ObjectCount * sizeof (Sim1Type) + 6 + COMMAND_MESSAGE_FOOTER_LENGTH +
					 COMMAND_MESSAGE_HEADER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (Sim1Type) * ObjectCount + 6 + COMMAND_MESSAGE_HEADER_LENGTH; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
	}


	return ObjectCount * sizeof (Sim1Type) + 6 + COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH;	//Total number of bytes

}

I32 ObjectControlBuildOSEMMessage(C8 * MessageBuffer, OSEMType * OSEMData, TimeType * GPSTime, C8 * Latitude,
								  C8 * Longitude, C8 * Altitude, U8 debug) {
	I32 MessageIndex = 0, i = 0;
	dbl Data;
	U16 Crc = 0;
	C8 *p;
	U32 ISODate = 0;

	bzero(MessageBuffer, COMMAND_OSEM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH);

	OSEMData->Header.SyncWordU16 = ISO_SYNC_WORD;
	OSEMData->Header.TransmitterIdU8 = 0;
	OSEMData->Header.MessageCounterU8 = 0;
	OSEMData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	OSEMData->Header.MessageIdU16 = COMMAND_OSEM_CODE;
	OSEMData->Header.MessageLengthU32 = sizeof (OSEMType) - sizeof (HeaderType) - 4;
	OSEMData->LatitudeValueIdU16 = VALUE_ID_LATITUDE;
	OSEMData->LatitudeContentLengthU16 = 6;
	OSEMData->LatitudeI64 = (I64) ((atof((const char *)Latitude) * 1e10));
	OSEMData->LongitudeValueIdU16 = VALUE_ID_LONGITUDE;
	OSEMData->LongitudeContentLengthU16 = 6;
	OSEMData->LongitudeI64 = (I64) ((atof((const char *)Longitude) * 1e10));
	OSEMData->AltitudeValueIdU16 = VALUE_ID_ALTITUDE;
	OSEMData->AltitudeContentLengthU16 = 4;
	OSEMData->AltitudeI32 = (I32) (atof((char *)Altitude) * 1e2);
	OSEMData->DateValueIdU16 = VALUE_ID_DATE_ISO8601;
	OSEMData->DateContentLengthU16 = 4;
	OSEMData->DateU32 =
		((U32) GPSTime->YearU16 * 10000) + ((U32) GPSTime->MonthU8 * 100) + ((U32) GPSTime->DayU8);
	OSEMData->GPSWeekValueIdU16 = VALUE_ID_GPS_WEEK;
	OSEMData->GPSWeekContentLengthU16 = 2;
	OSEMData->GPSWeekU16 = GPSTime->GPSWeekU16;
	OSEMData->GPSSOWValueIdU16 = VALUE_ID_GPS_SECOND_OF_WEEK;
	OSEMData->GPSSOWContentLengthU16 = 4;
	OSEMData->GPSQmsOfWeekU32 =
		((GPSTime->GPSSecondsOfWeekU32 * 1000 + GPSTime->MillisecondU16) << 2) + GPSTime->MicroSecondU16;
	OSEMData->MaxWayDeviationValueIdU16 = VALUE_ID_MAX_WAY_DEVIATION;
	OSEMData->MaxWayDeviationContentLengthU16 = 2;
	OSEMData->MaxWayDeviationU16 = 65535;
	OSEMData->MaxLateralDeviationValueIdU16 = VALUE_ID_MAX_LATERAL_DEVIATION;
	OSEMData->MaxLateralDeviationContentLengthU16 = 2;
	OSEMData->MaxLateralDeviationU16 = 65535;
	OSEMData->MinPosAccuracyContentLengthU16 = 2;
	OSEMData->MinPosAccuracyValueIdU16 = VALUE_ID_MIN_POS_ACCURACY;
	OSEMData->MinPosAccuracyU16 = 65535;

	if (!GPSTime->isGPSenabled) {
		OSEMData->DateU32 = UtilgetIntDateFromMS(UtilgetCurrentUTCtimeMS());
		UtilgetCurrentGPStime(&OSEMData->GPSWeekU16, &OSEMData->GPSQmsOfWeekU32);
	}

	p = (C8 *) OSEMData;
	for (i = 0; i < 21; i++)
		*(MessageBuffer + i) = *p++;
	*p++;
	*p++;
	for (; i < 31; i++)
		*(MessageBuffer + i) = *p++;
	*p++;
	*p++;
	for (; i < sizeof (OSEMType) - 4; i++)
		*(MessageBuffer + i) = *p++;

	Crc = crc_16((const C8 *)MessageBuffer, sizeof (OSEMType) - 4);
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc);
	*(MessageBuffer + i++) = (U8) (Crc >> 8);

	MessageIndex = i;

	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("OSEM total length = %d bytes (header+message+footer)\n",
			   (int)(COMMAND_OSEM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (OSEMType) - 4; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
		printf("Latitude = %ld\n", OSEMData->LatitudeI64);
		printf("Longitude = %ld\n", OSEMData->LongitudeI64);
		printf("ISODate = %d\n", OSEMData->DateU32);
	}
	return MessageIndex;		//Total number of bytes
}

int ObjectControlOSEMtoASCII(OSEMType * OSEMData, char *GPSWeek, char *GPSLatitude, char *GPSLongitude,
							 char *GPSAltitude) {
	// what do i want? in my mq? gps week, origin in lat and long coordinates
	bzero(GPSWeek, SMALL_BUFFER_SIZE_0);
	bzero(GPSLatitude, SMALL_BUFFER_SIZE_0);
	bzero(GPSLongitude, SMALL_BUFFER_SIZE_0);
	bzero(GPSAltitude, SMALL_BUFFER_SIZE_0);

	if (OSEMData->Header.MessageIdU16 == COMMAND_OSEM_CODE) {
		sprintf(GPSWeek, "%" PRIu16, OSEMData->GPSWeekU16);

		sprintf(GPSLatitude, "%" PRIi64, OSEMData->LatitudeI64);

		sprintf(GPSLongitude, "%" PRIi64, OSEMData->LongitudeI64);

		sprintf(GPSAltitude, "%" PRIi32, OSEMData->AltitudeI32);
	}
	return 0;
}

I32 ObjectControlBuildOSTMMessage(C8 * MessageBuffer, OSTMType * OSTMData, C8 CommandOption, U8 debug) {
	I32 MessageIndex = 0, i;
	U16 Crc = 0;
	C8 *p;

	bzero(MessageBuffer, COMMAND_OSTM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH);

	OSTMData->Header.SyncWordU16 = ISO_SYNC_WORD;
	OSTMData->Header.TransmitterIdU8 = 0;
	OSTMData->Header.MessageCounterU8 = 0;
	OSTMData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	OSTMData->Header.MessageIdU16 = COMMAND_OSTM_CODE;
	OSTMData->Header.MessageLengthU32 = sizeof (OSTMType) - sizeof (HeaderType);
	OSTMData->StateValueIdU16 = VALUE_ID_STATE_CHANGE_REQUEST;
	OSTMData->StateContentLengthU16 = sizeof (OSTMData->StateU8);
	OSTMData->StateU8 = (U8) CommandOption;

	p = (C8 *) OSTMData;
	for (i = 0; i < sizeof (OSTMType); i++)
		*(MessageBuffer + i) = *p++;
	Crc = crc_16((const C8 *)MessageBuffer, sizeof (OSTMType));
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc >> 8);
	*(MessageBuffer + i++) = (U8) (Crc);
	MessageIndex = i;

	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("OSTM total length = %d bytes (header+message+footer)\n",
			   (int)(COMMAND_OSTM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (OSTMType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes
}


I32 ObjectControlBuildHEABMessage(C8 * MessageBuffer, HEABType * HEABData, TimeType * GPSTime, U8 CCStatus,
								  U8 debug) {
	I32 MessageIndex = 0, i;
	U16 Crc = 0;
	C8 *p;

	bzero(MessageBuffer, COMMAND_HEAB_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH);

	HEABData->Header.SyncWordU16 = ISO_SYNC_WORD;
	HEABData->Header.TransmitterIdU8 = 0;
	HEABData->Header.MessageCounterU8 = 0;
	HEABData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	HEABData->Header.MessageIdU16 = COMMAND_HEAB_CODE;
	HEABData->Header.MessageLengthU32 = sizeof (HEABType) - sizeof (HeaderType);
	HEABData->HeabStructValueIdU16 = VALUE_ID_HEAB_STRUCT;
	HEABData->HeabStructContentLengthU16 = sizeof (HEABType) - sizeof (HeaderType)
		- sizeof (HEABData->HeabStructValueIdU16) - sizeof (HEABData->HeabStructContentLengthU16);
	HEABData->GPSQmsOfWeekU32 =
		((GPSTime->GPSSecondsOfWeekU32 * 1000 + (U32) TimeControlGetMillisecond(GPSTime)) << 2) +
		GPSTime->MicroSecondU16;
	HEABData->CCStatusU8 = CCStatus;

	if (!GPSTime->isGPSenabled) {
		UtilgetCurrentGPStime(NULL, &HEABData->GPSQmsOfWeekU32);
	}

	p = (C8 *) HEABData;
	for (i = 0; i < sizeof (HEABType); i++)
		*(MessageBuffer + i) = *p++;
	Crc = crc_16((const C8 *)MessageBuffer, sizeof (HEABType));
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc);
	*(MessageBuffer + i++) = (U8) (Crc >> 8);
	MessageIndex = i;

	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("HEAB total length = %d bytes (header+message+footer)\n",
			   (int)(COMMAND_HEAB_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (HEABType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes

}


int ObjectControlBuildLLCMMessage(char *MessageBuffer, unsigned short Speed, unsigned short Curvature,
								  unsigned char Mode, char debug) {
	int MessageIndex = 0;

	bzero(MessageBuffer, COMMAND_LLCM_MESSAGE_LENGTH + COMMAND_MESSAGE_HEADER_LENGTH);

	UtilAddOneByteMessageData(MessageBuffer, COMMAND_CODE_INDEX, COMMAND_LLCM_CODE);

	MessageIndex =
		UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex + COMMAND_MESSAGE_HEADER_LENGTH, Speed);

	MessageIndex = UtilAddTwoBytesMessageData(MessageBuffer, MessageIndex, Curvature);

	MessageIndex = UtilAddOneByteMessageData(MessageBuffer, MessageIndex, Mode);

	UtilAddFourBytesMessageData(MessageBuffer, COMMAND_MESSAGE_LENGTH_INDEX,
								(unsigned int)MessageIndex - COMMAND_MESSAGE_HEADER_LENGTH);

	if (debug) {
		int i = 0;

		LogMessage(LOG_LEVEL_DEBUG, "LLCM:");
		for (i = 0; i < MessageIndex; i++)
			LogMessage(LOG_LEVEL_DEBUG, "[%d]= %x", i, (unsigned char)MessageBuffer[i]);
	}

	return MessageIndex;		//Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH + message data count
}

I32 ObjectControlBuildSYPMMessage(C8 * MessageBuffer, SYPMType * SYPMData, U32 SyncPoint, U32 StopTime,
								  U8 debug) {

	I32 MessageIndex = 0, i;
	U16 Crc = 0;
	C8 *p;

	bzero(MessageBuffer, COMMAND_SYPM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH);

	SYPMData->Header.SyncWordU16 = ISO_SYNC_WORD;
	SYPMData->Header.TransmitterIdU8 = 0;
	SYPMData->Header.MessageCounterU8 = 0;
	SYPMData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	SYPMData->Header.MessageIdU16 = COMMAND_SYPM_CODE;
	SYPMData->Header.MessageLengthU32 = sizeof (SYPMType) - sizeof (HeaderType);
	SYPMData->SyncPointTimeValueIdU16 = 1;
	SYPMData->SyncPointTimeContentLengthU16 = 4;
	SYPMData->SyncPointTimeU32 = SyncPoint;
	SYPMData->FreezeTimeValueIdU16 = 2;
	SYPMData->FreezeTimeContentLengthU16 = 4;
	SYPMData->FreezeTimeU32 = StopTime;


	p = (C8 *) SYPMData;
	for (i = 0; i < sizeof (SYPMType); i++)
		*(MessageBuffer + i) = *p++;
	Crc = crc_16((const C8 *)MessageBuffer, sizeof (SYPMType));
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc >> 8);
	*(MessageBuffer + i++) = (U8) (Crc);
	MessageIndex = i;

	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("SYPM total length = %d bytes (header+message+footer)\n",
			   (int)(COMMAND_SYPM_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (SYPMType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes
}

I32 ObjectControlBuildMTSPMessage(C8 * MessageBuffer, MTSPType * MTSPData, U32 SyncTimestamp, U8 debug) {

	I32 MessageIndex = 0, i;
	U16 Crc = 0;
	C8 *p;

	bzero(MessageBuffer, COMMAND_MTSP_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH);

	MTSPData->Header.SyncWordU16 = ISO_SYNC_WORD;
	MTSPData->Header.TransmitterIdU8 = 0;
	MTSPData->Header.MessageCounterU8 = 0;
	MTSPData->Header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	MTSPData->Header.MessageIdU16 = COMMAND_MTSP_CODE;
	MTSPData->Header.MessageLengthU32 = sizeof (MTSPType) - sizeof (HeaderType);
	MTSPData->EstSyncPointTimeValueIdU16 = 1;
	MTSPData->EstSyncPointTimeContentLengthU16 = 4;
	MTSPData->EstSyncPointTimeU32 = SyncTimestamp;


	p = (C8 *) MTSPData;
	for (i = 0; i < sizeof (MTSPType); i++)
		*(MessageBuffer + i) = *p++;
	Crc = crc_16((const C8 *)MessageBuffer, sizeof (MTSPType));
	Crc = 0;
	*(MessageBuffer + i++) = (U8) (Crc >> 8);
	*(MessageBuffer + i++) = (U8) (Crc);
	MessageIndex = i;

	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("MTSPData total length = %d bytes (header+message+footer)\n",
			   (int)(COMMAND_MTSP_MESSAGE_LENGTH + COMMAND_MESSAGE_FOOTER_LENGTH));
		printf("----HEADER----\n");
		for (i = 0; i < sizeof (HeaderType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----MESSAGE----\n");
		for (; i < sizeof (MTSPType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n----FOOTER----\n");
		for (; i < MessageIndex; i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes
}


I32 ObjectControlBuildTRAJMessageHeader(C8 * MessageBuffer, I32 * RowCount, HeaderType * HeaderData,
										TRAJInfoType * TRAJInfoData, C8 * TrajFileHeader, U8 debug) {
	I32 MessageIndex = 0, i, j;
	U16 Crc = 0;
	C8 *p;
	C8 *token;


	if (strlen(TrajFileHeader) >= 1) {
		j = 0;
		token = strtok(TrajFileHeader, ";");
		while (token != NULL) {
			if (j == 1) {
				TRAJInfoData->TrajectoryIDValueIdU16 = VALUE_ID_TRAJECTORY_ID;
				TRAJInfoData->TrajectoryIDContentLengthU16 = 2;
				TRAJInfoData->TrajectoryIDU16 = atoi(token);
			}
			else if (j == 2) {
				TRAJInfoData->TrajectoryNameValueIdU16 = VALUE_ID_TRAJECTORY_NAME;
				TRAJInfoData->TrajectoryNameContentLengthU16 = 64;
				bzero(TRAJInfoData->TrajectoryNameC8, 64);
				strncpy(TRAJInfoData->TrajectoryNameC8, token, strlen(token));
			}
			else if (j == 3) {
				TRAJInfoData->TrajectoryVersionValueIdU16 = VALUE_ID_TRAJECTORY_VERSION;
				TRAJInfoData->TrajectoryVersionContentLengthU16 = 2;
				TRAJInfoData->TrajectoryVersionU16 = atoi(token);
			}
			else if (j == 4) {
				*RowCount = atoi(token);
			}

			j++;
			token = strtok(NULL, ";");
		}
	}

	TRAJInfoData->IpAddressValueIdU16 = 0xA000;
	TRAJInfoData->IpAddressContentLengthU16 = 4;
	TRAJInfoData->IpAddressU32 = 0;

	bzero(MessageBuffer, COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH);

	HeaderData->SyncWordU16 = ISO_SYNC_WORD;
	HeaderData->TransmitterIdU8 = 0;
	HeaderData->MessageCounterU8 = 0;
	HeaderData->AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	HeaderData->MessageIdU16 = COMMAND_DOTM_CODE;
	HeaderData->MessageLengthU32 =
		*RowCount * COMMAND_DOTM_ROW_MESSAGE_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH;

	p = (C8 *) HeaderData;
	for (i = 0; i < COMMAND_MESSAGE_HEADER_LENGTH; i++)
		*(MessageBuffer + i) = *p++;

	p = (C8 *) TRAJInfoData;
	for (; i < COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH; i++)
		*(MessageBuffer + i) = *p++;

	MessageIndex = i;


	if (debug) {
		// TODO: Change to log printout when byte thingy has been implemented
		printf("Header + TRAJInfo total length = %d bytes\n",
			   (int)(COMMAND_MESSAGE_HEADER_LENGTH + COMMAND_TRAJ_INFO_ROW_MESSAGE_LENGTH));
		printf("----HEADER + TRAJInfo----\n");
		for (i = 0; i < sizeof (HeaderType) + sizeof (TRAJInfoType); i++)
			printf("%x ", (unsigned char)MessageBuffer[i]);
		printf("\n");
		printf("DOTM message total length = %d bytes.\n", (int)HeaderData->MessageLengthU32);
		printf("Traj file header = %s\n", TrajFileHeader);
		printf("TrajectoryID = %d\n", TRAJInfoData->TrajectoryIDU16);
		printf("TrajectoryName = %s\n", TRAJInfoData->TrajectoryNameC8);
		printf("TrajectoryVersion = %d\n", TRAJInfoData->TrajectoryVersionU16);
		printf("RowCount = %d\n", *RowCount);
		printf("IpAddress = %d\n", TRAJInfoData->IpAddressU32);

		printf("\n----MESSAGE----\n");
	}

	return MessageIndex;		//Total number of bytes = COMMAND_MESSAGE_HEADER_LENGTH
}



I32 ObjectControlSendTRAJMessage(C8 * Filename, I32 * Socket, I32 RowCount, C8 * IP, U32 Port,
								 DOTMType * DOTMData, U8 debug) {
	FILE *fd;

	// Save socket settings and set it to blocking
	int retval = fcntl(*Socket, F_GETFL);

	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error getting socket options with fcntl");
		return -1;
	}
	int socketOptions = retval;

	retval = fcntl(*Socket, F_SETFL, socketOptions & ~O_NONBLOCK);
	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error setting socket options with fcntl");
		return -1;
	}


	fd = fopen(Filename, "r");
	if (fd == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", Filename);
		return -1;
	}

	UtilReadLineCntSpecChars(fd, TrajBuffer);	//Read first line
	int Rest = 0, i = 0, MessageLength = 0, SumMessageLength = 0, Modulo = 0, Transmissions = 0;

	Transmissions = RowCount / COMMAND_DOTM_ROWS_IN_TRANSMISSION;
	Rest = RowCount % COMMAND_DOTM_ROWS_IN_TRANSMISSION;
	U16 CrcU16 = 0;


	for (i = 0; i < Transmissions; i++) {
		MessageLength =
			ObjectControlBuildTRAJMessage(TrajBuffer, fd, COMMAND_DOTM_ROWS_IN_TRANSMISSION, DOTMData, debug);

		if (i == Transmissions && Rest == 0) {
			TrajBuffer[MessageLength] = (U8) (CrcU16);
			TrajBuffer[MessageLength + 1] = (U8) (CrcU16 >> 8);
			MessageLength = MessageLength + 2;
			UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
			SumMessageLength = SumMessageLength + MessageLength;
		}
		else {
			UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
			SumMessageLength = SumMessageLength + MessageLength;
		}

		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Transmission %d: %d bytes sent", i, MessageLength);
	}

	if (Rest > 0) {
		MessageLength = ObjectControlBuildTRAJMessage(TrajBuffer, fd, Rest, DOTMData, debug);
		TrajBuffer[MessageLength] = (U8) (CrcU16);
		TrajBuffer[MessageLength + 1] = (U8) (CrcU16 >> 8);
		MessageLength = MessageLength + 2;
		UtilSendTCPData("Object Control", TrajBuffer, MessageLength, Socket, 0);
		SumMessageLength = SumMessageLength + MessageLength;
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Transmission %d: %d bytes sent.\n", i, MessageLength);
	}

	LogMessage(LOG_LEVEL_INFO, "%d DOTM bytes sent to %s:%d", SumMessageLength, IP, Port);
	fclose(fd);

	// Reset socket settings
	retval = fcntl(*Socket, F_SETFL, socketOptions);
	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error setting socket options with fcntl");
		return -1;
	}


	return 0;
}

I32 ObjectControlBuildTRAJMessage(C8 * MessageBuffer, FILE * fd, I32 RowCount, DOTMType * DOTMData, U8 debug) {
	I32 MessageIndex = 0;
	C8 RowBuffer[100];
	C8 DataBuffer[20];
	dbl Data;
	C8 *src, *p;
	U16 Crc = 0;
	flt curv = 0;
	C8 *pc;

	bzero(MessageBuffer, COMMAND_DOTM_ROW_MESSAGE_LENGTH * RowCount);

	I32 i = 0, j = 0, n = 0;

	for (i = 0; i < RowCount; i++) {
		bzero(RowBuffer, 100);
		UtilReadLineCntSpecChars(fd, RowBuffer);

		//Read to ';' in row = LINE;0.00;21.239000;39.045000;0.000000;-1.240001;0.029103;0.004005;0.000000;3;ENDLINE;
		//Time
		src = strchr(RowBuffer, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (U64) strchr(src + 1, ';') - (U64) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->RelativeTimeValueIdU16 = VALUE_ID_RELATIVE_TIME;
		DOTMData->RelativeTimeContentLengthU16 = 4;
		DOTMData->RelativeTimeU32 = (U32) Data;
		if (debug)
			printf("Time DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//x
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->XPositionValueIdU16 = VALUE_ID_X_POSITION;
		DOTMData->XPositionContentLengthU16 = 4;
		DOTMData->XPositionI32 = (I32) Data;
		if (debug)
			printf("X DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//y
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->YPositionValueIdU16 = VALUE_ID_Y_POSITION;
		DOTMData->YPositionContentLengthU16 = 4;
		DOTMData->YPositionI32 = (I32) Data;
		if (debug)
			printf("Y DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//z
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->ZPositionValueIdU16 = VALUE_ID_Z_POSITION;
		DOTMData->ZPositionContentLengthU16 = 4;
		DOTMData->ZPositionI32 = (I32) Data;
		if (debug)
			printf("Z DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Heading
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = UtilRadToDeg(atof(DataBuffer));
		Data = 450 - Data;		//Turn heading back pi/2
		while (Data < 0)
			Data += 360.0;
		while (Data > 360)
			Data -= 360.0;
		DOTMData->HeadingValueIdU16 = VALUE_ID_HEADING;
		DOTMData->HeadingContentLengthU16 = 2;
		DOTMData->HeadingU16 = (U16) (Data * 1e2);
		if (debug)
			printf("Heading DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Longitudinal speed
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e2;
		DOTMData->LongitudinalSpeedValueIdU16 = VALUE_ID_LONGITUDINAL_SPEED;
		DOTMData->LongitudinalSpeedContentLengthU16 = 2;
		DOTMData->LongitudinalSpeedI16 = (I16) Data;
		if (debug)
			printf("Long speed DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Lateral speed
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e2;
		DOTMData->LateralSpeedValueIdU16 = VALUE_ID_LATERAL_SPEED;
		DOTMData->LateralSpeedContentLengthU16 = 2;
		DOTMData->LateralSpeedI16 = (I16) Data;
		if (debug)
			printf("Lat speed DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Longitudinal acceleration
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->LongitudinalAccValueIdU16 = VALUE_ID_LONGITUDINAL_ACCELERATION;
		DOTMData->LongitudinalAccContentLengthU16 = 2;
		DOTMData->LongitudinalAccI16 = (I16) Data;
		if (debug)
			printf("Long acc DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Lateral acceleration
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		Data = atof(DataBuffer) * 1e3;
		DOTMData->LateralAccValueIdU16 = VALUE_ID_LATERAL_ACCELERATION;
		DOTMData->LateralAccContentLengthU16 = 2;
		DOTMData->LateralAccI16 = (I16) Data;
		if (debug)
			printf("Lat accDataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		//Curvature
		src = strchr(src + 1, ';');
		bzero(DataBuffer, 20);
		strncpy(DataBuffer, src + 1, (uint64_t) strchr(src + 1, ';') - (uint64_t) src - 1);
		//Data = atof(DataBuffer) * 3e4;
		curv = atof(DataBuffer);
		pc = (C8 *) & curv;
		DOTMData->CurvatureValueIdU16 = VALUE_ID_CURVATURE;
		DOTMData->CurvatureContentLengthU16 = 4;
		//DOTMData->CurvatureI32 = (I32) Data;
		DOTMData->CurvatureI32 = pc[0];
		DOTMData->CurvatureI32 = DOTMData->CurvatureI32 | ((I32) pc[1]) << 8;
		DOTMData->CurvatureI32 = DOTMData->CurvatureI32 | ((I32) pc[2]) << 16;
		DOTMData->CurvatureI32 = DOTMData->CurvatureI32 | ((I32) pc[3]) << 24;

		if (debug)
			printf("Curv DataBuffer=%s  float=%3.6f\n", DataBuffer, Data);

		p = (C8 *) DOTMData;
		for (j = 0; j < sizeof (DOTMType); j++, n++)
			*(MessageBuffer + n) = *p++;
		MessageIndex = n;
	}


	if (debug) {
		int i = 0;

		for (i = 0; i < MessageIndex; i++) {
			// TODO: Write to log when bytes thingy has been implemented
			if ((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15)
				printf("0");
			printf("%x-", (unsigned char)MessageBuffer[i]);
		}
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes

}


I32 ObjectControlBuildASPMessage(C8 * MessageBuffer, ASPType * ASPData, U8 debug) {
	I32 MessageIndex = 0, i;
	C8 *p;

	bzero(MessageBuffer, ASP_MESSAGE_LENGTH);
	p = (C8 *) ASPData;
	for (i = 0; i < sizeof (ASPType); i++)
		*(MessageBuffer + i) = *p++;
	MessageIndex = i;

	if (debug) {
		// TODO: Write to log when bytes thingy has been implemented
		printf("ASP total length = %d bytes \n", (int)(ASP_MESSAGE_LENGTH));
		printf("\n----MESSAGE----\n");
		for (i = 0; i < sizeof (ASPType); i++)
			printf("%x ", (C8) MessageBuffer[i]);
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes
}


/*!
 * \brief ObjectControlSendACCMMessage Sends ACCM data, reformatted to an ISO compliant message, to specified TCP socket
 * \param ACCM ACCM data from message bus
 * \param socket Socket where to send ACCM
 * \param debug Debug flag
 * \return Length of sent message
 */
I32 ObjectControlSendACCMMessage(ACCMData * ACCM, I32 * socket, U8 debug) {
	ACCMType isoACCM;
	C8 messageBuffer[sizeof (isoACCM)];
	C8 *ptr = messageBuffer;
	U32 messageSize = 0;

	ObjectControlBuildACCMMessage(ACCM, &isoACCM, debug);

	// Copy ACCM header to send buffer
	memcpy(ptr, &isoACCM.header.SyncWordU16, sizeof (isoACCM.header.SyncWordU16));
	ptr += sizeof (isoACCM.header.SyncWordU16);

	memcpy(ptr, &isoACCM.header.TransmitterIdU8, sizeof (isoACCM.header.TransmitterIdU8));
	ptr += sizeof (isoACCM.header.TransmitterIdU8);

	memcpy(ptr, &isoACCM.header.MessageCounterU8, sizeof (isoACCM.header.MessageCounterU8));
	ptr += sizeof (isoACCM.header.MessageCounterU8);

	memcpy(ptr, &isoACCM.header.AckReqProtVerU8, sizeof (isoACCM.header.AckReqProtVerU8));
	ptr += sizeof (isoACCM.header.AckReqProtVerU8);

	memcpy(ptr, &isoACCM.header.MessageIdU16, sizeof (isoACCM.header.MessageIdU16));
	ptr += sizeof (isoACCM.header.MessageIdU16);

	memcpy(ptr, &isoACCM.header.MessageLengthU32, sizeof (isoACCM.header.MessageLengthU32));
	ptr += sizeof (isoACCM.header.MessageLengthU32);


	// Copy ACCM action ID to send buffer
	memcpy(ptr, &isoACCM.actionIDValueID, sizeof (isoACCM.actionIDValueID));
	ptr += sizeof (isoACCM.actionIDValueID);

	memcpy(ptr, &isoACCM.actionIDContentLength, sizeof (isoACCM.actionIDContentLength));
	ptr += sizeof (isoACCM.actionIDContentLength);

	memcpy(ptr, &isoACCM.actionID, sizeof (isoACCM.actionID));
	ptr += sizeof (isoACCM.actionID);

	// Copy ACCM action type to send buffer
	memcpy(ptr, &isoACCM.actionTypeValueID, sizeof (isoACCM.actionTypeValueID));
	ptr += sizeof (isoACCM.actionTypeValueID);

	memcpy(ptr, &isoACCM.actionTypeContentLength, sizeof (isoACCM.actionTypeContentLength));
	ptr += sizeof (isoACCM.actionTypeContentLength);

	memcpy(ptr, &isoACCM.actionType, sizeof (isoACCM.actionType));
	ptr += sizeof (isoACCM.actionType);

	// Copy ACCM action parameter 1 to send buffer
	memcpy(ptr, &isoACCM.actionTypeParameter1ValueID, sizeof (isoACCM.actionTypeParameter1ValueID));
	ptr += sizeof (isoACCM.actionTypeParameter1ValueID);

	memcpy(ptr, &isoACCM.actionTypeParameter1ContentLength,
		   sizeof (isoACCM.actionTypeParameter1ContentLength));
	ptr += sizeof (isoACCM.actionTypeParameter1ContentLength);

	memcpy(ptr, &isoACCM.actionTypeParameter1, sizeof (isoACCM.actionTypeParameter1));
	ptr += sizeof (isoACCM.actionTypeParameter1);

	// Copy ACCM action parameter 2 to send buffer
	memcpy(ptr, &isoACCM.actionTypeParameter2ValueID, sizeof (isoACCM.actionTypeParameter2ValueID));
	ptr += sizeof (isoACCM.actionTypeParameter2ValueID);

	memcpy(ptr, &isoACCM.actionTypeParameter2ContentLength,
		   sizeof (isoACCM.actionTypeParameter2ContentLength));
	ptr += sizeof (isoACCM.actionTypeParameter2ContentLength);

	memcpy(ptr, &isoACCM.actionTypeParameter2, sizeof (isoACCM.actionTypeParameter2));
	ptr += sizeof (isoACCM.actionTypeParameter2);

	// Copy ACCM action parameter 3 to send buffer
	memcpy(ptr, &isoACCM.actionTypeParameter3ValueID, sizeof (isoACCM.actionTypeParameter3ValueID));
	ptr += sizeof (isoACCM.actionTypeParameter3ValueID);

	memcpy(ptr, &isoACCM.actionTypeParameter3ContentLength,
		   sizeof (isoACCM.actionTypeParameter3ContentLength));
	ptr += sizeof (isoACCM.actionTypeParameter3ContentLength);

	memcpy(ptr, &isoACCM.actionTypeParameter3, sizeof (isoACCM.actionTypeParameter3));
	ptr += sizeof (isoACCM.actionTypeParameter3);


	// Copy ACCM footer to send buffer
	memcpy(ptr, &isoACCM.footer.Crc, sizeof (isoACCM.footer.Crc));
	ptr += sizeof (isoACCM.footer.Crc);

	if (ptr > messageBuffer)
		messageSize = (U32) (ptr - messageBuffer);

	if (messageSize - sizeof (isoACCM.header) - sizeof (isoACCM.footer) != isoACCM.header.MessageLengthU32)
		LogMessage(LOG_LEVEL_WARNING, "ACCM message sent with invalid message length");

	UtilSendTCPData(MODULE_NAME, messageBuffer, (I32) messageSize, socket, debug);

	return (I32) messageSize;
}

/*!
 * \brief ObjectControlSendTRCMMessage Sends TRCM data, reformatted to an ISO compliant message, to specified TCP socket
 * \param ACCM TRCM data from message bus
 * \param socket Socket where to send TRCM
 * \param debug Debug flag
 * \return Length of sent message
 */
I32 ObjectControlSendTRCMMessage(TRCMData * TRCM, I32 * socket, U8 debug) {
	TRCMType isoTRCM;
	C8 messageBuffer[sizeof (isoTRCM)];
	C8 *ptr = messageBuffer;
	U32 messageSize = 0;

	ObjectControlBuildTRCMMessage(TRCM, &isoTRCM, debug);

	// Copy TRCM header to send buffer
	memcpy(ptr, &isoTRCM.header.SyncWordU16, sizeof (isoTRCM.header.SyncWordU16));
	ptr += sizeof (isoTRCM.header.SyncWordU16);

	memcpy(ptr, &isoTRCM.header.TransmitterIdU8, sizeof (isoTRCM.header.TransmitterIdU8));
	ptr += sizeof (isoTRCM.header.TransmitterIdU8);

	memcpy(ptr, &isoTRCM.header.MessageCounterU8, sizeof (isoTRCM.header.MessageCounterU8));
	ptr += sizeof (isoTRCM.header.MessageCounterU8);

	memcpy(ptr, &isoTRCM.header.AckReqProtVerU8, sizeof (isoTRCM.header.AckReqProtVerU8));
	ptr += sizeof (isoTRCM.header.AckReqProtVerU8);

	memcpy(ptr, &isoTRCM.header.MessageIdU16, sizeof (isoTRCM.header.MessageIdU16));
	ptr += sizeof (isoTRCM.header.MessageIdU16);

	memcpy(ptr, &isoTRCM.header.MessageLengthU32, sizeof (isoTRCM.header.MessageLengthU32));
	ptr += sizeof (isoTRCM.header.MessageLengthU32);


	// Copy TRCM trigger ID to send buffer
	memcpy(ptr, &isoTRCM.triggerIDValueID, sizeof (isoTRCM.triggerIDValueID));
	ptr += sizeof (isoTRCM.triggerIDValueID);

	memcpy(ptr, &isoTRCM.triggerIDContentLength, sizeof (isoTRCM.triggerIDContentLength));
	ptr += sizeof (isoTRCM.triggerIDContentLength);

	memcpy(ptr, &isoTRCM.triggerID, sizeof (isoTRCM.triggerID));
	ptr += sizeof (isoTRCM.triggerID);

	// Copy TRCM trigger type to send buffer
	memcpy(ptr, &isoTRCM.triggerTypeValueID, sizeof (isoTRCM.triggerTypeValueID));
	ptr += sizeof (isoTRCM.triggerTypeValueID);

	memcpy(ptr, &isoTRCM.triggerTypeContentLength, sizeof (isoTRCM.triggerTypeContentLength));
	ptr += sizeof (isoTRCM.triggerTypeContentLength);

	memcpy(ptr, &isoTRCM.triggerType, sizeof (isoTRCM.triggerType));
	ptr += sizeof (isoTRCM.triggerType);

	// Copy TRCM trigger parameter 1 to send buffer
	memcpy(ptr, &isoTRCM.triggerTypeParameter1ValueID, sizeof (isoTRCM.triggerTypeParameter1ValueID));
	ptr += sizeof (isoTRCM.triggerTypeParameter1ValueID);

	memcpy(ptr, &isoTRCM.triggerTypeParameter1ContentLength,
		   sizeof (isoTRCM.triggerTypeParameter1ContentLength));
	ptr += sizeof (isoTRCM.triggerTypeParameter1ContentLength);

	memcpy(ptr, &isoTRCM.triggerTypeParameter1, sizeof (isoTRCM.triggerTypeParameter1));
	ptr += sizeof (isoTRCM.triggerTypeParameter1);

	// Copy TRCM trigger parameter 2 to send buffer
	memcpy(ptr, &isoTRCM.triggerTypeParameter2ValueID, sizeof (isoTRCM.triggerTypeParameter2ValueID));
	ptr += sizeof (isoTRCM.triggerTypeParameter2ValueID);

	memcpy(ptr, &isoTRCM.triggerTypeParameter2ContentLength,
		   sizeof (isoTRCM.triggerTypeParameter2ContentLength));
	ptr += sizeof (isoTRCM.triggerTypeParameter2ContentLength);

	memcpy(ptr, &isoTRCM.triggerTypeParameter2, sizeof (isoTRCM.triggerTypeParameter2));
	ptr += sizeof (isoTRCM.triggerTypeParameter2);

	// Copy TRCM trigger parameter 3 to send buffer
	memcpy(ptr, &isoTRCM.triggerTypeParameter3ValueID, sizeof (isoTRCM.triggerTypeParameter3ValueID));
	ptr += sizeof (isoTRCM.triggerTypeParameter3ValueID);

	memcpy(ptr, &isoTRCM.triggerTypeParameter3ContentLength,
		   sizeof (isoTRCM.triggerTypeParameter3ContentLength));
	ptr += sizeof (isoTRCM.triggerTypeParameter3ContentLength);

	memcpy(ptr, &isoTRCM.triggerTypeParameter3, sizeof (isoTRCM.triggerTypeParameter3));
	ptr += sizeof (isoTRCM.triggerTypeParameter3);


	// Copy TRCM footer to send buffer
	memcpy(ptr, &isoTRCM.footer.Crc, sizeof (isoTRCM.footer.Crc));
	ptr += sizeof (isoTRCM.footer.Crc);

	if (ptr > messageBuffer)
		messageSize = (U32) (ptr - messageBuffer);

	if (messageSize - sizeof (isoTRCM.header) - sizeof (isoTRCM.footer) != isoTRCM.header.MessageLengthU32)
		LogMessage(LOG_LEVEL_WARNING, "TRCM message sent with invalid message length");

	UtilSendTCPData(MODULE_NAME, messageBuffer, (I32) messageSize, socket, 0);

	return (I32) messageSize;
}

/*!
 * \brief ObjectControlSendEXACMessage Sends EXAC data, reformatted to an ISO compliant message, to specified TCP socket
 * \param ACCM EXAC data from message bus
 * \param socket Socket where to send EXAC
 * \param debug Debug flag
 * \return Length of sent message
 */
I32 ObjectControlSendEXACMessage(EXACData * EXAC, I32 * socket, U8 debug) {
	EXACType isoEXAC;
	C8 messageBuffer[sizeof (isoEXAC)];
	C8 *ptr = messageBuffer;
	U32 messageSize = 0;

	ObjectControlBuildEXACMessage(EXAC, &isoEXAC, debug);

	// Copy EXAC header to send buffer
	memcpy(ptr, &isoEXAC.header.SyncWordU16, sizeof (isoEXAC.header.SyncWordU16));
	ptr += sizeof (isoEXAC.header.SyncWordU16);

	memcpy(ptr, &isoEXAC.header.TransmitterIdU8, sizeof (isoEXAC.header.TransmitterIdU8));
	ptr += sizeof (isoEXAC.header.TransmitterIdU8);

	memcpy(ptr, &isoEXAC.header.MessageCounterU8, sizeof (isoEXAC.header.MessageCounterU8));
	ptr += sizeof (isoEXAC.header.MessageCounterU8);

	memcpy(ptr, &isoEXAC.header.AckReqProtVerU8, sizeof (isoEXAC.header.AckReqProtVerU8));
	ptr += sizeof (isoEXAC.header.AckReqProtVerU8);

	memcpy(ptr, &isoEXAC.header.MessageIdU16, sizeof (isoEXAC.header.MessageIdU16));
	ptr += sizeof (isoEXAC.header.MessageIdU16);

	memcpy(ptr, &isoEXAC.header.MessageLengthU32, sizeof (isoEXAC.header.MessageLengthU32));
	ptr += sizeof (isoEXAC.header.MessageLengthU32);


	// Copy EXAC action ID to send buffer
	memcpy(ptr, &isoEXAC.actionIDValueID, sizeof (isoEXAC.actionIDValueID));
	ptr += sizeof (isoEXAC.actionIDValueID);

	memcpy(ptr, &isoEXAC.actionIDContentLength, sizeof (isoEXAC.actionIDContentLength));
	ptr += sizeof (isoEXAC.actionIDContentLength);

	memcpy(ptr, &isoEXAC.actionID, sizeof (isoEXAC.actionID));
	ptr += sizeof (isoEXAC.actionID);

	// Copy EXAC action execution time to send buffer
	memcpy(ptr, &isoEXAC.executionTime_qmsoWValueID, sizeof (isoEXAC.executionTime_qmsoWValueID));
	ptr += sizeof (isoEXAC.executionTime_qmsoWValueID);

	memcpy(ptr, &isoEXAC.executionTime_qmsoWContentLength, sizeof (isoEXAC.executionTime_qmsoWContentLength));
	ptr += sizeof (isoEXAC.executionTime_qmsoWContentLength);

	memcpy(ptr, &isoEXAC.executionTime_qmsoW, sizeof (isoEXAC.executionTime_qmsoW));
	ptr += sizeof (isoEXAC.executionTime_qmsoW);


	// Copy EXAC footer to send buffer
	memcpy(ptr, &isoEXAC.footer.Crc, sizeof (isoEXAC.footer.Crc));
	ptr += sizeof (isoEXAC.footer.Crc);

	if (ptr > messageBuffer)
		messageSize = (U32) (ptr - messageBuffer);

	if (messageSize - sizeof (isoEXAC.header) - sizeof (isoEXAC.footer) != isoEXAC.header.MessageLengthU32)
		LogMessage(LOG_LEVEL_WARNING, "EXAC message sent with invalid message length");

	UtilSendTCPData(MODULE_NAME, messageBuffer, (I32) messageSize, socket, 0);

	return (I32) messageSize;
}


/*!
 * \brief ObjectControlBuildACCMMessage Fills an ISO ACCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param mqACCMData Data which is to fill ACCM struct
 * \param ACCM Output ACCM struct
 * \param debug Debug flag
 * \return Byte size of ACCM struct
 */
I32 ObjectControlBuildACCMMessage(ACCMData * mqACCMData, ACCMType * ACCM, U8 debug) {
	// Header
	ACCM->header.SyncWordU16 = ISO_SYNC_WORD;
	ACCM->header.TransmitterIdU8 = 0;
	ACCM->header.MessageCounterU8 = 0;
	ACCM->header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	ACCM->header.MessageIdU16 = COMMAND_ACCM_CODE;
	ACCM->header.MessageLengthU32 = sizeof (ACCMType) - sizeof (HeaderType) - sizeof (FooterType);

	// Data fields
	ACCM->actionID = mqACCMData->actionID;
	ACCM->actionType = mqACCMData->actionType;
	ACCM->actionTypeParameter1 = mqACCMData->actionTypeParameter1;
	ACCM->actionTypeParameter2 = mqACCMData->actionTypeParameter2;
	ACCM->actionTypeParameter3 = mqACCMData->actionTypeParameter3;

	// Value ID fields
	ACCM->actionIDValueID = VALUE_ID_ACTION_ID;
	ACCM->actionTypeValueID = VALUE_ID_ACTION_TYPE;
	ACCM->actionTypeParameter1ValueID = VALUE_ID_ACTION_TYPE_PARAM1;
	ACCM->actionTypeParameter2ValueID = VALUE_ID_ACTION_TYPE_PARAM2;
	ACCM->actionTypeParameter3ValueID = VALUE_ID_ACTION_TYPE_PARAM3;

	// Content length fields
	ACCM->actionIDContentLength = sizeof (ACCM->actionID);
	ACCM->actionTypeContentLength = sizeof (ACCM->actionType);
	ACCM->actionTypeParameter1ContentLength = sizeof (ACCM->actionTypeParameter1);
	ACCM->actionTypeParameter2ContentLength = sizeof (ACCM->actionTypeParameter2);
	ACCM->actionTypeParameter3ContentLength = sizeof (ACCM->actionTypeParameter3);

	// Header content length
	ACCM->header.MessageLengthU32 = sizeof (ACCM->actionID) + sizeof (ACCM->actionType)
		+ sizeof (ACCM->actionTypeParameter1) + sizeof (ACCM->actionTypeParameter2) +
		sizeof (ACCM->actionTypeParameter3)
		+ sizeof (ACCM->actionIDValueID) + sizeof (ACCM->actionTypeValueID)
		+ sizeof (ACCM->actionTypeParameter1ValueID) + sizeof (ACCM->actionTypeParameter1ValueID) +
		sizeof (ACCM->actionTypeParameter3ValueID)
		+ sizeof (ACCM->actionIDContentLength) + sizeof (ACCM->actionTypeContentLength)
		+ sizeof (ACCM->actionTypeParameter1ContentLength) +
		sizeof (ACCM->actionTypeParameter1ContentLength) + sizeof (ACCM->actionTypeParameter3ContentLength);


	// Footer (TODO)
	ACCM->footer.Crc = 0;

	U32 messageLen =
		ACCM->header.MessageLengthU32 + sizeof (ACCM->footer.Crc) + sizeof (ACCM->header.SyncWordU16) +
		sizeof (ACCM->header.MessageIdU16) + sizeof (ACCM->header.AckReqProtVerU8) +
		sizeof (ACCM->header.TransmitterIdU8) + sizeof (ACCM->header.MessageCounterU8) +
		sizeof (ACCM->header.MessageLengthU32);

	if (debug) {
		LogPrint
			("ACCM (%u bytes):\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x",
			 messageLen, ACCM->actionIDValueID, ACCM->actionIDContentLength, ACCM->actionID,
			 ACCM->actionTypeValueID, ACCM->actionTypeContentLength, ACCM->actionType,
			 ACCM->actionTypeParameter1ValueID, ACCM->actionTypeParameter1ContentLength,
			 ACCM->actionTypeParameter1, ACCM->actionTypeParameter2ValueID,
			 ACCM->actionTypeParameter2ContentLength, ACCM->actionTypeParameter2,
			 ACCM->actionTypeParameter3ValueID, ACCM->actionTypeParameter3ContentLength,
			 ACCM->actionTypeParameter3);
	}

	return (I32) messageLen;
}

/*!
 * \brief ObjectControlBuildEXACMessage Fills an ISO EXAC struct with relevant data fields, and corresponding value IDs and content lengths
 * \param mqEXACData Data which is to fill EXAC struct
 * \param EXAC Output EXAC struct
 * \param debug Debug flag
 * \return Byte size of EXAC struct
 */
I32 ObjectControlBuildEXACMessage(EXACData * mqEXACData, EXACType * EXAC, U8 debug) {
	// TODO: Make system time follow GPS time (better) or pass the GSD pointer into here somehow
	struct timeval systemTime;

	TimeSetToCurrentSystemTime(&systemTime);

	// Header
	EXAC->header.SyncWordU16 = ISO_SYNC_WORD;
	EXAC->header.TransmitterIdU8 = 0;
	EXAC->header.MessageCounterU8 = 0;
	EXAC->header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	EXAC->header.MessageIdU16 = COMMAND_EXAC_CODE;

	// Data fields
	EXAC->actionID = mqEXACData->actionID;
	EXAC->executionTime_qmsoW = mqEXACData->executionTime_qmsoW;

	// Value ID fields
	EXAC->actionIDValueID = VALUE_ID_ACTION_ID;
	EXAC->executionTime_qmsoWValueID = VALUE_ID_ACTION_EXECUTE_TIME;

	// Content length fields
	EXAC->actionIDContentLength = sizeof (EXAC->actionID);
	EXAC->executionTime_qmsoWContentLength = sizeof (EXAC->executionTime_qmsoW);


	// Header message length
	EXAC->header.MessageLengthU32 = sizeof (EXAC->actionID) + sizeof (EXAC->executionTime_qmsoW)
		+ sizeof (EXAC->actionIDValueID) + sizeof (EXAC->executionTime_qmsoWValueID)
		+ sizeof (EXAC->actionIDContentLength) + sizeof (EXAC->executionTime_qmsoWContentLength);

	// Footer (TODO)
	EXAC->footer.Crc = 0;

	U32 messageLen =
		EXAC->header.MessageLengthU32 + sizeof (EXAC->footer.Crc) + sizeof (EXAC->header.SyncWordU16) +
		sizeof (EXAC->header.MessageIdU16) + sizeof (EXAC->header.AckReqProtVerU8) +
		sizeof (EXAC->header.TransmitterIdU8) + sizeof (EXAC->header.MessageCounterU8) +
		sizeof (EXAC->header.MessageLengthU32);

	if (debug) {
		LogPrint("EXAC (%u bytes):\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x", messageLen,
				 EXAC->actionIDValueID, EXAC->actionIDContentLength, EXAC->actionID,
				 EXAC->executionTime_qmsoWValueID, EXAC->executionTime_qmsoWContentLength,
				 EXAC->executionTime_qmsoW);
	}

	return (I32) messageLen;
}

/*!
 * \brief ObjectControlBuildTRCMMessage Fills an ISO TRCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param mqTRCMData Data which is to fill TRCM struct
 * \param TRCM Output TRCM struct
 * \param debug Debug flag
 * \return Byte size of TRCM struct
 */
I32 ObjectControlBuildTRCMMessage(TRCMData * mqTRCMData, TRCMType * TRCM, U8 debug) {
	// Header
	TRCM->header.SyncWordU16 = ISO_SYNC_WORD;
	TRCM->header.TransmitterIdU8 = 0;
	TRCM->header.MessageCounterU8 = 0;
	TRCM->header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	TRCM->header.MessageIdU16 = COMMAND_TRCM_CODE;


	// Data fields
	TRCM->triggerID = mqTRCMData->triggerID;
	TRCM->triggerType = mqTRCMData->triggerType;
	TRCM->triggerTypeParameter1 = mqTRCMData->triggerTypeParameter1;
	TRCM->triggerTypeParameter2 = mqTRCMData->triggerTypeParameter2;
	TRCM->triggerTypeParameter3 = mqTRCMData->triggerTypeParameter3;

	// Value ID fields
	TRCM->triggerIDValueID = VALUE_ID_TRIGGER_ID;
	TRCM->triggerIDValueID = VALUE_ID_TRIGGER_TYPE;
	TRCM->triggerTypeParameter1ValueID = VALUE_ID_TRIGGER_TYPE_PARAM1;
	TRCM->triggerTypeParameter2ValueID = VALUE_ID_TRIGGER_TYPE_PARAM2;
	TRCM->triggerTypeParameter3ValueID = VALUE_ID_TRIGGER_TYPE_PARAM3;

	// Content length fields
	TRCM->triggerIDContentLength = sizeof (TRCM->triggerID);
	TRCM->triggerTypeContentLength = sizeof (TRCM->triggerType);
	TRCM->triggerTypeParameter1ContentLength = sizeof (TRCM->triggerTypeParameter1);
	TRCM->triggerTypeParameter2ContentLength = sizeof (TRCM->triggerTypeParameter2);
	TRCM->triggerTypeParameter3ContentLength = sizeof (TRCM->triggerTypeParameter3);


	// Message length in header
	TRCM->header.MessageLengthU32 = sizeof (TRCM->triggerID) + sizeof (TRCM->triggerType)
		+ sizeof (TRCM->triggerTypeParameter1) + sizeof (TRCM->triggerTypeParameter2) +
		sizeof (TRCM->triggerTypeParameter3)
		+ sizeof (TRCM->triggerIDValueID) + sizeof (TRCM->triggerTypeValueID)
		+ sizeof (TRCM->triggerTypeParameter1ValueID) + sizeof (TRCM->triggerTypeParameter1ValueID) +
		sizeof (TRCM->triggerTypeParameter3ValueID)
		+ sizeof (TRCM->triggerIDContentLength) + sizeof (TRCM->triggerTypeContentLength)
		+ sizeof (TRCM->triggerTypeParameter1ContentLength) +
		sizeof (TRCM->triggerTypeParameter1ContentLength) + sizeof (TRCM->triggerTypeParameter3ContentLength);


	// Footer (TODO)
	TRCM->footer.Crc = 0;

	U32 messageLen =
		TRCM->header.MessageLengthU32 + sizeof (TRCM->footer.Crc) + sizeof (TRCM->header.SyncWordU16) +
		sizeof (TRCM->header.MessageIdU16) + sizeof (TRCM->header.AckReqProtVerU8) +
		sizeof (TRCM->header.TransmitterIdU8) + sizeof (TRCM->header.MessageCounterU8) +
		sizeof (TRCM->header.MessageLengthU32);
	if (debug) {
		LogPrint
			("TRCM (%u bytes):\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x\n\t%#x-%#x-%#x",
			 messageLen, TRCM->triggerIDValueID, TRCM->triggerIDContentLength, TRCM->triggerID,
			 TRCM->triggerTypeValueID, TRCM->triggerTypeContentLength, TRCM->triggerType,
			 TRCM->triggerTypeParameter1ValueID, TRCM->triggerTypeParameter1ContentLength,
			 TRCM->triggerTypeParameter1, TRCM->triggerTypeParameter2ValueID,
			 TRCM->triggerTypeParameter2ContentLength, TRCM->triggerTypeParameter2,
			 TRCM->triggerTypeParameter3ValueID, TRCM->triggerTypeParameter3ContentLength,
			 TRCM->triggerTypeParameter3);
	}

	return (I32) messageLen;
}


I32 ObjectControlSendDTMMessage(C8 * DTMData, I32 * Socket, I32 RowCount, C8 * IP, U32 Port,
								DOTMType * DOTMData, U8 debug) {

	U32 Rest = 0, i = 0, MessageLength = 0, SumMessageLength = 0, Modulo = 0, Transmissions = 0;
	U16 CrcU16 = 0;

	MessageLength =
		ObjectControlBuildDTMMessage(TrajBuffer, DTMData, COMMAND_DOTM_ROWS_IN_TRANSMISSION, DOTMData, 0);

	if (debug)
		LogMessage(LOG_LEVEL_DEBUG, "Transmission %d: %d bytes sent", i, MessageLength);

	LogMessage(LOG_LEVEL_INFO, "%d DTM bytes sent to %s:%d", SumMessageLength, IP, Port);

	return 0;
}


I32 ObjectControlBuildDTMMessage(C8 * MessageBuffer, C8 * DTMData, I32 RowCount, DOTMType * DOTMData,
								 U8 debug) {
	I32 MessageIndex = 0;
	U32 Data;
	C8 *src, *p;
	U16 Crc = 0;

	bzero(MessageBuffer, COMMAND_DOTM_ROW_MESSAGE_LENGTH * RowCount);

	I32 i = 0, j = 0, n = 0;

	for (i = 0; i < RowCount; i++) {
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "DOTM row:");
		//Time
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 3);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 2);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 1);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 0);
		DOTMData->RelativeTimeValueIdU16 = VALUE_ID_RELATIVE_TIME;
		DOTMData->RelativeTimeContentLengthU16 = 4;
		DOTMData->RelativeTimeU32 = SwapU32((U32) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Time=%d", DOTMData->RelativeTimeU32);

		//x
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 7);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 6);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 5);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 4);
		DOTMData->XPositionValueIdU16 = VALUE_ID_X_POSITION;
		DOTMData->XPositionContentLengthU16 = 4;
		DOTMData->XPositionI32 = SwapI32((I32) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "X=%d", DOTMData->XPositionI32);

		//y
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 11);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 10);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 9);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 8);
		DOTMData->YPositionValueIdU16 = VALUE_ID_Y_POSITION;
		DOTMData->YPositionContentLengthU16 = 4;
		DOTMData->YPositionI32 = SwapI32((I32) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Y=%d", DOTMData->YPositionI32);

		//z
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 15);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 14);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 13);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 12);
		DOTMData->ZPositionValueIdU16 = VALUE_ID_Z_POSITION;
		DOTMData->ZPositionContentLengthU16 = 4;
		DOTMData->ZPositionI32 = SwapI32((I32) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Z=%d", DOTMData->ZPositionI32);

		//Heading
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 17);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 16);
		//Data = UtilRadToDeg(Data);
		//Data = 4500 - Data; //Turn heading back pi/2
		//while(Data<0) Data+=360.0;
		//while(Data>3600) Data-=360.0;
		DOTMData->HeadingValueIdU16 = VALUE_ID_HEADING;
		DOTMData->HeadingContentLengthU16 = 2;
		DOTMData->HeadingU16 = SwapU16((U16) (Data));
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "Heading=%d", DOTMData->HeadingU16);

		//Longitudinal speed
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 19);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 18);
		DOTMData->LongitudinalSpeedValueIdU16 = VALUE_ID_LONGITUDINAL_SPEED;
		DOTMData->LongitudinalSpeedContentLengthU16 = 2;
		DOTMData->LongitudinalSpeedI16 = SwapI16((I16) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "LongitudinalSpeedI16=%d", DOTMData->LongitudinalSpeedI16);

		//Lateral speed
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 21);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 20);
		DOTMData->LateralSpeedValueIdU16 = VALUE_ID_LATERAL_SPEED;
		DOTMData->LateralSpeedContentLengthU16 = 2;
		DOTMData->LateralSpeedI16 = SwapI16((I16) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "LateralSpeedI16=%d", DOTMData->LateralSpeedI16);

		//Longitudinal acceleration
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 23);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 22);
		DOTMData->LongitudinalAccValueIdU16 = VALUE_ID_LONGITUDINAL_ACCELERATION;
		DOTMData->LongitudinalAccContentLengthU16 = 2;
		DOTMData->LongitudinalAccI16 = SwapI16((I16) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "LongitudinalAccI16=%d", DOTMData->LongitudinalAccI16);

		//Lateral acceleration
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 25);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 24);
		DOTMData->LateralAccValueIdU16 = VALUE_ID_LATERAL_ACCELERATION;
		DOTMData->LateralAccContentLengthU16 = 2;
		DOTMData->LateralAccI16 = SwapI16((I16) Data);
		if (debug)
			LogMessage(LOG_LEVEL_DEBUG, "LateralAccI16=%d", DOTMData->LateralAccI16);

		//Curvature
		Data = 0;
		Data = *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 29);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 28);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 27);
		Data = (Data << 8) | *(DTMData + COMMAND_DTM_BYTES_IN_ROW * i + 26);
		DOTMData->CurvatureValueIdU16 = VALUE_ID_CURVATURE;
		DOTMData->CurvatureContentLengthU16 = 4;
		DOTMData->CurvatureI32 = SwapI32((I32) Data);
		if (debug)
			printf("CurvatureI32=%d \n", DOTMData->CurvatureI32);

		p = (C8 *) DOTMData;
		for (j = 0; j < sizeof (DOTMType); j++, n++)
			*(MessageBuffer + n) = *p++;
		MessageIndex = n;
	}


	Crc = crc_16((const C8 *)MessageBuffer, sizeof (DOTMType));
	Crc = 0;
	*(MessageBuffer + MessageIndex++) = (U8) (Crc);
	*(MessageBuffer + MessageIndex++) = (U8) (Crc >> 8);


	if (debug) {
		int i = 0;

		for (i = 0; i < MessageIndex; i++) {
			// TODO: Write to log when bytes thingy has been implemented
			if ((unsigned char)MessageBuffer[i] >= 0 && (unsigned char)MessageBuffer[i] <= 15)
				printf("0");
			printf("%x-", (unsigned char)MessageBuffer[i]);
		}
		printf("\n");
	}

	return MessageIndex;		//Total number of bytes

}


static int iGetObjectIndexFromObjectIP(in_addr_t ipAddr, in_addr_t objectIPs[], unsigned int numberOfObjects) {
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		if (objectIPs[i] == ipAddr)
			return (int)i;
	}
	return -1;
}

static I32 vConnectObject(int *sockfd, const char *name, const uint32_t port, U8 * Disconnect) {
	struct sockaddr_in serv_addr;
	struct hostent *server;

	char buffer[256];
	int iResult;

	*sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (*sockfd < 0) {
		util_error("[ObjectControl] ERR: Failed to open control socket");
	}

	server = gethostbyname(name);
	if (server == NULL) {
		util_error("[ObjectControl] ERR: Unknown host ");
	}

	bzero((char *)&serv_addr, sizeof (serv_addr));
	serv_addr.sin_family = AF_INET;

	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(port);

	LogMessage(LOG_LEVEL_INFO, "Attempting to connect to socket: %s %i", name, port);

	// do
	{
		iResult = connect(*sockfd, (struct sockaddr *)&serv_addr, sizeof (serv_addr));

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
		   } */
	}
	//} while(iResult < 0 && *Disconnect == 0);

	LogMessage(LOG_LEVEL_INFO, "Connected to command socket: %s %i", name, port);
	// Enable polling of status to detect remote disconnect
	fcntl(*sockfd, F_SETFL, O_NONBLOCK);


	return iResult;
}

static void vDisconnectObject(int *sockfd) {
	close(*sockfd);
}


static void vCreateSafetyChannel(const char *name, const uint32_t port, int *sockfd, struct sockaddr_in *addr) {
	int result;
	struct hostent *object;

	/* Connect to object safety socket */
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");

	*sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (*sockfd < 0) {
		util_error("ERR: Failed to connect to monitor socket");
	}

	/* Set address to object */
	object = gethostbyname(name);

	if (object == 0) {
		util_error("ERR: Unknown host");
	}

	bcopy((char *)object->h_addr, (char *)&addr->sin_addr.s_addr, object->h_length);
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);

	/* set socket to non-blocking */
	result = fcntl(*sockfd, F_SETFL, fcntl(*sockfd, F_GETFL, 0) | O_NONBLOCK);
	if (result < 0) {
		util_error("ERR: calling fcntl");
	}

	LogMessage(LOG_LEVEL_INFO, "Created socket and safety address: %s:%d", name, port);
}

static void vCloseSafetyChannel(int *sockfd) {
	close(*sockfd);
}

static I32 vCheckRemoteDisconnected(int *sockfd) {
	char dummy;
	ssize_t x = recv(*sockfd, &dummy, 1, MSG_PEEK);

	// Remote has disconnected: EOF => x=0
	if (x == 0) {
		return 1;
	}

	if (x == -1) {
		// Everything is normal - no communication has been received
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return 0;

		// Other error occurred
		LogMessage(LOG_LEVEL_WARNING, "Error when checking connection status");
		return 1;
	}

	// Something has been received on socket
	if (x > 0) {
		LogMessage(LOG_LEVEL_INFO, "Received unexpected communication from object on command channel");
		return 0;
	}

	return 1;
}

int ObjectControlSendUDPData(int *sockfd, struct sockaddr_in *addr, char *SendData, int Length, char debug) {
	ssize_t result;

	// TODO: Change to log write when bytes thingy has been implemented
	if (debug) {
		printf("Bytes sent: ");
		int i = 0;

		for (i = 0; i < Length; i++)
			printf("%x-", (unsigned char)*(SendData + i));
		printf("\n");
	}

	result = sendto(*sockfd, SendData, Length, 0, (const struct sockaddr *)addr, sizeof (struct sockaddr_in));

	if (result < 0) {
		util_error("ERR: Failed to send on monitor socket");
	}

	return 0;
}


static size_t uiRecvMonitor(int *sockfd, char *buffer, size_t length) {
	ssize_t result = 0;
	size_t recvDataSize = 0;

	// Read until receive buffer is empty, return last read message
	do {
		result = recv(*sockfd, buffer, length, 0);

		if (result < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				util_error("Failed to receive from monitor socket");
			}
		}
		else {
			recvDataSize = (size_t) (result);
			LogMessage(LOG_LEVEL_DEBUG, "Received: <%s>", buffer);
		}
	} while (result > 0);

	return recvDataSize;
}

int iFindObjectsInfo(C8 object_traj_file[MAX_OBJECTS][MAX_FILE_PATH],
					 C8 object_address_name[MAX_OBJECTS][MAX_FILE_PATH], in_addr_t objectIPs[MAX_OBJECTS],
					 I32 * nbr_objects) {
	DIR *traj_directory;
	struct dirent *directory_entry;
	int iForceObjectToLocalhost;
	struct sockaddr_in sockaddr;
	int result;
	char trajPathDir[MAX_FILE_PATH];
	int retval = 0;

	UtilGetTrajDirectoryPath(trajPathDir, sizeof (trajPathDir));

	iForceObjectToLocalhost = 0;

	traj_directory = opendir(trajPathDir);
	if (traj_directory == NULL) {
		util_error("ERR: Failed to open trajectory directory");
	}

	(void)iUtilGetIntParaConfFile("ForceObjectToLocalhost", &iForceObjectToLocalhost);

	while ((directory_entry = readdir(traj_directory)) && ((*nbr_objects) < MAX_OBJECTS)) {

		/* Check so it's not . or .. */
		if (strncmp(directory_entry->d_name, ".", 1) && (strstr(directory_entry->d_name, "sync") == NULL)) {
			bzero(object_address_name[(*nbr_objects)], MAX_FILE_PATH);

			bzero(object_traj_file[(*nbr_objects)], MAX_FILE_PATH);
			(void)strcat(object_traj_file[(*nbr_objects)], trajPathDir);
			(void)strcat(object_traj_file[(*nbr_objects)], directory_entry->d_name);

			if (UtilCheckTrajectoryFileFormat
				(object_traj_file[*nbr_objects], sizeof (object_traj_file[*nbr_objects]))) {
				LogMessage(LOG_LEVEL_ERROR, "Trajectory file <%s> is not valid",
						   object_traj_file[*nbr_objects]);
				retval = -1;
			}

			if (0 == iForceObjectToLocalhost) {
				(void)strncat(object_address_name[(*nbr_objects)], directory_entry->d_name,
							  strlen(directory_entry->d_name));
				result = inet_pton(AF_INET, object_address_name[*nbr_objects], &sockaddr.sin_addr);
				if (result == -1) {
					LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
					retval = -1;
					continue;
				}
				else if (result == 0) {
					LogMessage(LOG_LEVEL_WARNING, "Address <%s> is not a valid IPv4 address",
							   object_address_name[*nbr_objects]);
					retval = -1;
					continue;
				}
				else
					objectIPs[*nbr_objects] = sockaddr.sin_addr.s_addr;
			}
			else {
				if (USE_TEST_HOST == 0)
					(void)strcat(object_address_name[(*nbr_objects)], LOCALHOST);
				else if (USE_TEST_HOST == 1)
					(void)strcat(object_address_name[(*nbr_objects)], TESTHOST_IP);

			}

			++(*nbr_objects);
		}
	}
	(void)closedir(traj_directory);
	return retval;
}



OBCState_t vGetState(GSDType * GSD) {
	return DataDictionaryGetOBCStateU8(GSD);
}

StateTransitionResult vSetState(OBCState_t requestedState, GSDType * GSD) {
	StateTransition transitionFunction;
	StateTransitionResult retval = TRANSITION_RESULT_UNDEFINED;
	OBCState_t currentState = DataDictionaryGetOBCStateU8(GSD);

	// Always allow transitions to these two states
	if (requestedState == OBC_STATE_ERROR || requestedState == OBC_STATE_UNDEFINED) {
		if (DataDictionarySetOBCStateU8(GSD, requestedState) == WRITE_OK) {
			LogMessage(LOG_LEVEL_WARNING, "Transitioning to state %u", (unsigned char)requestedState);
			retval = TRANSITION_OK;
		}
		else
			retval = TRANSITION_MEMORY_ERROR;
	}
	else if (requestedState == currentState) {
		retval = TRANSITION_OK;
	}
	else {
		transitionFunction = tGetTransition(currentState);
		retval = transitionFunction(&currentState, requestedState);
		if (retval != TRANSITION_INVALID) {
			if (DataDictionarySetOBCStateU8(GSD, currentState) == WRITE_OK) {
				LogMessage(LOG_LEVEL_INFO, "Transitioning to state %u", (unsigned char)requestedState);
				retval = TRANSITION_OK;
			}
			else
				retval = TRANSITION_MEMORY_ERROR;
		}
	}

	if (retval == TRANSITION_INVALID) {
		LogMessage(LOG_LEVEL_WARNING, "Invalid transition requested: from %d to %d", currentState,
				   requestedState);
	}
	else if (retval == TRANSITION_MEMORY_ERROR) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to set state to %u in shared memory!!", requestedState);
	}
	return retval;
}


StateTransition tGetTransition(OBCState_t fromState) {
	switch (fromState) {
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

StateTransitionResult tFromIdle(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_INITIALIZED) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromInitialized(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_IDLE) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromConnected(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_ARMED || requestedState == OBC_STATE_IDLE) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromArmed(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_RUNNING
		|| requestedState == OBC_STATE_IDLE) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromRunning(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_CONNECTED || requestedState == OBC_STATE_IDLE) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromError(OBCState_t * currentState, OBCState_t requestedState) {
	if (requestedState == OBC_STATE_IDLE) {
		*currentState = requestedState;
		return TRANSITION_OK;
	}
	return TRANSITION_INVALID;
}

StateTransitionResult tFromUndefined(OBCState_t * currentState, OBCState_t requestedState) {
	return TRANSITION_INVALID;
}


OBCState_t vInitializeState(OBCState_t firstState, GSDType * GSD) {
	static int8_t isInitialized = 0;

	if (!isInitialized) {
		isInitialized = 1;
		if (DataDictionarySetOBCStateU8(GSD, firstState) != WRITE_OK)
			util_error("Unable to write object control state to shared memory");
	}
	else {
		LogMessage(LOG_LEVEL_WARNING, "Object control state already initialized");
	}
	return DataDictionaryGetOBCStateU8(GSD);
}
