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

#define OC_SLEEP_TIME_EMPTY_MQ_S 0
#define OC_SLEEP_TIME_EMPTY_MQ_NS 1000000
#define OC_SLEEP_TIME_NONEMPTY_MQ_S 0
#define OC_SLEEP_TIME_NONEMPTY_MQ_NS 0

#define OBJECT_CONTROL_CONTROL_MODE 0
#define OBJECT_CONTROL_REPLAY_MODE 1
#define OBJECT_CONTROL_ABORT_MODE 1

#define OC_STATE_REPORT_PERIOD_S 1
#define OC_STATE_REPORT_PERIOD_US 0

#define TRAJECTORY_TX_BUFFER_SIZE 2048

#define SMALL_BUFFER_SIZE_0 20
#define SMALL_BUFFER_SIZE_1 2
#define SMALL_BUFFER_SIZE_2 1
#define SMALL_BUFFER_SIZE_254 254

#define TRAJECTORY_FILE_MAX_ROWS  4096

#define LOG_BUFFER_LENGTH 128

#define USE_TEMP_LOGFILE 0
#define TEMP_LOG_FILE "log/temp.log"


typedef enum {
	TRANSITION_RESULT_UNDEFINED,
	TRANSITION_OK,
	TRANSITION_INVALID,
	TRANSITION_MEMORY_ERROR
} StateTransitionResult;

typedef struct {
	uint16_t actionID;
	ActionTypeParameter_t command;
	in_addr_t ip;
} TestScenarioCommandAction;	//!< Struct describing a command to be sent as action, e.g. delayed start


/* Small note: syntax for declaring a function pointer is (example for a function taking an int and a float,
   returning nothing) where the function foo(int a, float b) is declared elsewhere:
      void (*fooptr)(int,float) = foo;
      fooptr(10,1.5);

   Consequently, the below typedef defines a StateTransition type as a function pointer to a function taking
   (OBCState_t, OBCState_t) as input, and returning a StateTransitionResult
*/
typedef StateTransitionResult(*StateTransition) (OBCState_t * currentState, OBCState_t requestedState);


/*------------------------------------------------------------
-- Function declarations.
------------------------------------------------------------*/
static I32 vConnectObject(int *sockfd, const char *name, const uint32_t port, U8 * Disconnect);
static void vDisconnectObject(int *sockfd);
static I32 vCheckRemoteDisconnected(int *sockfd);

static void vCreateSafetyChannel(const char *name, const uint32_t port, int *sockfd,
								 struct sockaddr_in *addr);
static void vCloseSafetyChannel(int *sockfd);
static size_t uiRecvMonitor(int *sockfd, char *buffer, size_t length);
static int iGetObjectIndexFromObjectIP(in_addr_t ipAddr, in_addr_t objectIPs[], unsigned int numberOfObjects);
static void signalHandler(int signo);
static void resetCommandActionList(TestScenarioCommandAction commandActions[],
								   const int numberOfElementsInList);
static int addCommandToActionList(const TestScenarioCommandAction command,
								  TestScenarioCommandAction commandActions[],
								  const int numberOfElementsInList);

static int hasDelayedStart(const in_addr_t objectIP, const TestScenarioCommandAction commandActions[],
						   const int numberOfElementsInList);
static int findCommandAction(const uint16_t actionID, const TestScenarioCommandAction commandActions[],
							 const int numberOfElementsInList);
static ssize_t ObjectControlSendTRAJMessage(const char *Filename, int *Socket, const char debug);

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
StateTransitionResult tFromRemoteControl(OBCState_t * currentState, OBCState_t requestedState);
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
	TestScenarioCommandAction commandActions[MAX_OBJECTS];
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
	struct timeval currentTime, nextStateReportTime, nextHeartbeatTime, nextAdaptiveSyncMessageTime;

	const struct timeval heartbeatPeriod = { 1 / HEAB_FREQUENCY_HZ,
		(1000000 / HEAB_FREQUENCY_HZ) % 1000000
	};
	const struct timeval adaptiveSyncMessagePeriod = heartbeatPeriod;

	U8 iForceObjectToLocalhostU8 = DEFAULT_FORCE_OBJECT_TO_LOCALHOST;

	FILE *fd;
	C8 Timestamp[SMALL_BUFFER_SIZE_0];
	C8 GPSWeek[SMALL_BUFFER_SIZE_0];
	ssize_t MessageLength;
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
	dbl OriginLatitudeDbl = DEFAULT_ORIGIN_LAT;
	dbl OriginLongitudeDbl = DEFAULT_ORIGIN_LOG;
	dbl OriginAltitudeDbl = DEFAULT_ORIGIN_ALT;
	dbl OriginHeadingDbl = DEFAULT_ORIGIN_HEADING;
	C8 pcSendBuffer[MBUS_MAX_DATALEN];
	C8 ObjectPort[SMALL_BUFFER_SIZE_0];
	MonitorDataType monitorData;
	ACCMData mqACCMData;
	EXACData mqEXACData;
	TRCMData mqTRCMData;
	GeoPosition OriginPosition;
	ASPType ASPData;

	ASPData.MTSPU32 = 0;
	ASPData.TimeToSyncPointDbl = 0;
	ASPData.PrevTimeToSyncPointDbl = 0;
	ASPData.CurrentTimeDbl = 0;
	AdaptiveSyncPoint ASP[MAX_ADAPTIVE_SYNC_POINTS];
	I32 SyncPointCount = 0;
	I32 SearchStartIndex = 0;
	dbl ASPMaxTimeDiffDbl = DEFAULT_ASP_MAX_TIME_DIFF;
	dbl ASPMaxTrajDiffDbl = DEFAULT_ASP_MAX_TRAJ_DIFF;
	dbl ASPFilterLevelDbl = DEFAULT_ASP_FILTER_LEVEL;
	dbl ASPMaxDeltaTimeDbl = DEFAULT_ASP_MAX_DELTA_TIME;
	I32 ASPDebugRate = 1;
	I32 ASPStepBackCount = DEFAULT_ASP_STEP_BACK_COUNT;
	char confDirectoryPath[MAX_FILE_PATH];

	ControlCenterStatusType objectControlServerStatus = CONTROL_CENTER_STATUS_INIT;

	vInitializeState(OBC_STATE_IDLE, GSD);
	U8 uiTimeCycle = 0;
	I32 ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;

	C8 Buffer2[SMALL_BUFFER_SIZE_1];
	C8 LogBuffer[LOG_BUFFER_LENGTH];
	C8 VOILReceivers[SMALL_BUFFER_SIZE_254];
	C8 DTMReceivers[SMALL_BUFFER_SIZE_254];
	U32 RowCount;

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
	nextHeartbeatTime = currentTime;
	nextAdaptiveSyncMessageTime = currentTime;

	while (!iExit) {

		if (vGetState(GSD) == OBC_STATE_ERROR) {
			objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
			MessageLength =
				encodeHEABMessage(objectControlServerStatus, MessageBuffer, sizeof (MessageBuffer), 0);
			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex],
								MessageBuffer, MessageLength, 0);
			}
		}

		// Heartbeat
		if ((vGetState(GSD) == OBC_STATE_RUNNING || vGetState(GSD) == OBC_STATE_ARMED
			 || vGetState(GSD) == OBC_STATE_CONNECTED || vGetState(GSD) == OBC_STATE_REMOTECTRL)
			&& timercmp(&currentTime, &nextHeartbeatTime, >)) {

			timeradd(&nextHeartbeatTime, &heartbeatPeriod, &nextHeartbeatTime);
			MessageLength =
				encodeHEABMessage(objectControlServerStatus, MessageBuffer, sizeof (MessageBuffer), 0);

			// Transmit heartbeat to all objects
			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				UtilSendUDPData("Object Control", &safety_socket_fd[iIndex], &safety_object_addr[iIndex],
								MessageBuffer, MessageLength, 0);
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
			|| vGetState(GSD) == OBC_STATE_ARMED || vGetState(GSD) == OBC_STATE_REMOTECTRL) {
			char buffer[RECV_MESSAGE_BUFFER];
			size_t receivedMONRData = 0;

			CurrentTimeU32 =
				((GPSTime->GPSSecondsOfWeekU32 * 1000 + (U32) TimeControlGetMillisecond(GPSTime)) << 2) +
				GPSTime->MicroSecondU16;

			 /*MTSP*/ if (timercmp(&currentTime, &nextAdaptiveSyncMessageTime, >)) {

				timeradd(&nextAdaptiveSyncMessageTime, &adaptiveSyncMessagePeriod,
						 &nextAdaptiveSyncMessageTime);

				struct timeval estSyncPointTime;

				TimeSetToGPStime(&estSyncPointTime, TimeGetAsGPSweek(&currentTime), ASPData.MTSPU32);

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					for (i = 0; i < SyncPointCount; i++) {
						if (TEST_SYNC_POINTS == 0
							&& strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL
							&& ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1) {

							/*Send Master time to adaptive sync point */
							MessageLength =
								encodeMTSPMessage(&estSyncPointTime, MessageBuffer, sizeof (MessageBuffer),
												  0);
							UtilSendUDPData(MODULE_NAME, &safety_socket_fd[iIndex],
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

				if (receivedMONRData > 0 && getISOMessageType(buffer, receivedMONRData, 0) == MESSAGE_ID_MONR) {
					LogMessage(LOG_LEVEL_DEBUG, "Recieved %d bytes of new data from %s %d: %s",
							   receivedMONRData, object_address_name[iIndex], object_udp_port[iIndex],
							   buffer);

					monitorData.ClientIP = safety_object_addr[iIndex].sin_addr.s_addr;
					if (decodeMONRMessage
						(buffer, receivedMONRData, &monitorData.ClientID, &monitorData.data,
						 0) != MESSAGE_OK) {
						LogMessage(LOG_LEVEL_INFO, "Error decoding MONR from %s: disconnecting object",
								   object_address_name[iIndex]);
						vDisconnectObject(&safety_socket_fd[iIndex]);
						// TODO smarter way of handling?
						continue;
					}

					if (ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) {
						// Place struct in buffer
						memcpy(&buffer, &monitorData, sizeof (monitorData));
						// Send MONR message as bytes
						if (iCommSend(COMM_MONR, buffer, sizeof (monitorData)) < 0) {
							LogMessage(LOG_LEVEL_ERROR,
									   "Fatal communication fault when sending MONR command - entering error state");
							vSetState(OBC_STATE_ERROR, GSD);
							objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
						}
					}


					// Store monitor data in shared memory
					if (DataDictionarySetMonitorData(&monitorData) == UNDEFINED) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to write monitor data to shared memory");
						// TODO: handle the error
					}

					memset(buffer, 0, sizeof (buffer));
					UtilMonitorDataToString(monitorData, buffer, sizeof (buffer));

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

							OP[iIndex].x = monitorData.data.position.xCoord_m;	//Set x and y on OP (ObjectPosition)
							OP[iIndex].y = monitorData.data.position.yCoord_m;

							//OP[iIndex].OrigoDistance = sqrt(pow(OP[iIndex].x,2) + pow(OP[iIndex].y,2)); //Calculate hypotenuse

							// TODO: check use of this function since it should take two lat/long points but is here used with x/y
							UtilCalcPositionDelta(OriginLatitudeDbl, OriginLongitudeDbl,
												  monitorData.data.position.xCoord_m,
												  monitorData.data.position.yCoord_m, &OP[iIndex]);

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

								if (TimeGetAsGPSqmsOfWeek(&monitorData.data.timestamp) % ASPDebugRate == 0) {
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
						(float)sqrt(pow(monitorData.data.speed.lateral_m_s, 2) +
									pow(monitorData.data.speed.longitudinal_m_s, 2));
				}
				else if (receivedMONRData > 0)
					LogMessage(LOG_LEVEL_WARNING, "Received unhandled message on monitoring socket");
			}
		}


		bzero(pcRecvBuffer, RECV_MESSAGE_BUFFER);
		//Have we recieved a command?
		if (iCommRecv(&iCommand, pcRecvBuffer, RECV_MESSAGE_BUFFER, NULL)) {
			LogMessage(LOG_LEVEL_DEBUG, "Received command %d", iCommand);


			if (iCommand == COMM_ARM && vGetState(GSD) == OBC_STATE_CONNECTED) {

				LogMessage(LOG_LEVEL_INFO, "Sending ARM");
				LOG_SEND(LogBuffer, "[ObjectControl] Sending ARM");
				vSetState(OBC_STATE_ARMED, GSD);
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_ARM, MessageBuffer, sizeof (MessageBuffer), 0);

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					UtilSendTCPData("[Object Control]", MessageBuffer, MessageLength, &socket_fds[iIndex], 0);
				}

				objectControlServerStatus = CONTROL_CENTER_STATUS_READY;
			}
			else if (iCommand == COMM_DISARM && vGetState(GSD) == OBC_STATE_ARMED) {

				LogMessage(LOG_LEVEL_INFO, "Sending DISARM");
				LOG_SEND(LogBuffer, "[ObjectControl] Sending DISARM");
				vSetState(OBC_STATE_CONNECTED, GSD);

				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_DISARM, MessageBuffer, sizeof (MessageBuffer), 0);

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					UtilSendTCPData("[" MODULE_NAME "]", MessageBuffer, MessageLength, &socket_fds[iIndex],
									0);
				}

				objectControlServerStatus = CONTROL_CENTER_STATUS_READY;
			}
			else if (iCommand == COMM_STRT && (vGetState(GSD) == OBC_STATE_ARMED) /*|| OBC_STATE_INITIALIZED) */ )	//OBC_STATE_INITIALIZED is temporary!
			{
				struct timeval startTime, startDelay;

				MiscPtr = pcRecvBuffer;
				TimeSetToUTCms(&startTime, (int64_t) strtoul(MiscPtr, &MiscPtr, 10));
				TimeSetToUTCms(&startDelay, (int64_t) strtoul(MiscPtr + 1, NULL, 10));
				timeradd(&startTime, &startDelay, &startTime);
				MessageLength = encodeSTRTMessage(&startTime, MessageBuffer, sizeof (MessageBuffer), 0);

				ASPData.MTSPU32 = 0;
				ASPData.TimeToSyncPointDbl = 0;
				SearchStartIndex = -1;
				ASPData.PrevTimeToSyncPointDbl = 0;
				OldTimeU32 = CurrentTimeU32;
				objectControlServerStatus = CONTROL_CENTER_STATUS_READY;

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					if (!hasDelayedStart
						(objectIPs[iIndex], commandActions,
						 sizeof (commandActions) / sizeof (commandActions[0])))
						UtilSendTCPData("Object Control", MessageBuffer, MessageLength, &socket_fds[iIndex],
										0);
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
				objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
				LogMessage(LOG_LEVEL_WARNING, "ABORT received");
				LOG_SEND(LogBuffer, "[ObjectControl] ABORT received.");
			}
			else if (iCommand == COMM_CONTROL) {
				ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
				printf("[ObjectControl] Object control in CONTROL mode\n");
			}
			else if (iCommand == COMM_REMOTECTRL_ENABLE) {
				vSetState(OBC_STATE_REMOTECTRL, GSD);
				// TODO: objectControlServerStatus = something
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_REMOTE_CONTROL, MessageBuffer, sizeof (MessageBuffer),
									  0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					LogMessage(LOG_LEVEL_INFO, "Setting object with IP %s to remote control mode",
							   object_address_name[iIndex]);
					UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &socket_fds[iIndex], 0);
				}
				// TODO: check objects' states
				LogMessage(LOG_LEVEL_INFO, "Enabled remote control mode");
			}
			else if (iCommand == COMM_REMOTECTRL_DISABLE) {
				// TODO set objects' states to connected
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_DISARM, MessageBuffer, sizeof (MessageBuffer), 0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					LogMessage(LOG_LEVEL_INFO, "Setting object with IP %s to disarmed mode",
							   object_address_name[iIndex]);
					UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &socket_fds[iIndex], 0);
				}
				// TODO: check objects' states
				// TODO: objectControlServerStatus = something
				vSetState(OBC_STATE_CONNECTED, GSD);
				LogMessage(LOG_LEVEL_INFO, "Disabled remote control mode");
			}
			else if (iCommand == COMM_REMOTECTRL_MANOEUVRE) {
				RemoteControlCommandType rcCommand;
				char ipString[INET_ADDRSTRLEN];

				// TODO check size of received data
				memcpy(&rcCommand, pcRecvBuffer, sizeof (rcCommand));
				LogMessage(LOG_LEVEL_INFO, "Received remote control manoeuvre for object with IP %s", inet_ntop(AF_INET, &rcCommand.objectIP, ipString, sizeof (ipString)));	// TODO print command type
				if (vGetState(GSD) == OBC_STATE_REMOTECTRL) {
					switch (rcCommand.manoeuvre) {
					case MANOEUVRE_BACK_TO_START:
						iIndex =
							iGetObjectIndexFromObjectIP(rcCommand.objectIP, objectIPs,
														sizeof (objectIPs) / sizeof (objectIPs[0]));
						if (iIndex != -1) {
							LogMessage(LOG_LEVEL_INFO, "Sending back to start command to object with IP %s",
									   object_address_name[iIndex]);
							MessageLength =
								encodeRCMMMessage(rcCommand.manoeuvre, MessageBuffer, sizeof (MessageBuffer),
												  0);
							if (MessageLength > 0) {
								UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
												&socket_fds[iIndex], 0);
							}
							else {
								LogMessage(LOG_LEVEL_ERROR, "Error encoding RCMM message");
							}
						}
						else {
							char ipString[INET_ADDRSTRLEN];

							LogMessage(LOG_LEVEL_ERROR, "Back to start command for invalid IP %s received",
									   inet_ntop(AF_INET, &rcCommand.objectIP, ipString, sizeof (ipString)));
						}
						break;
					default:
						LogMessage(LOG_LEVEL_ERROR, "Unsupported remote control manoeuvre");
					}
				}
				else {
					LogMessage(LOG_LEVEL_WARNING,
							   "Remote control manoeuvring is not allowed outside of remote control mode");
				}
			}
			else if (iCommand == COMM_INIT) {
				LogMessage(LOG_LEVEL_INFO, "INIT received");
				LOG_SEND(LogBuffer, "[ObjectControl] INIT received.");
				nbr_objects = 0;
				if (iFindObjectsInfo(object_traj_file, object_address_name, objectIPs, &nbr_objects) == 0) {
					// Get objects; name and drive file
					DataDictionaryGetForceToLocalhostU8(GSD, &iForceObjectToLocalhostU8);

					resetCommandActionList(commandActions,
										   sizeof (commandActions) / sizeof (commandActions[0]));

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
				if (mqACCMData.actionType == ACTION_TEST_SCENARIO_COMMAND) {
					// Special handling is required from Maestro for test scenario command
					TestScenarioCommandAction newCommandAction;

					if (mqACCMData.actionTypeParameter1 == ACTION_PARAMETER_VS_SEND_START) {
						newCommandAction.ip = mqACCMData.ip;
						newCommandAction.command = mqACCMData.actionTypeParameter1;
						newCommandAction.actionID = mqACCMData.actionID;
						if (addCommandToActionList
							(newCommandAction, commandActions,
							 sizeof (commandActions) / sizeof (commandActions[0])) == -1) {
							LogMessage(LOG_LEVEL_ERROR, "Unable to handle command action configuration");
						}
					}
					else {
						LogMessage(LOG_LEVEL_WARNING, "Unimplemented test scenario command action");
					}
				}
				else {
					// Send ACCM to objects
					iIndex = iGetObjectIndexFromObjectIP(mqACCMData.ip, objectIPs,
														 sizeof (objectIPs) / sizeof (objectIPs[0]));
					if (iIndex != -1) {
						MessageLength =
							encodeACCMMessage(&mqACCMData.actionID, &mqACCMData.actionType,
											  &mqACCMData.actionTypeParameter1,
											  &mqACCMData.actionTypeParameter2,
											  &mqACCMData.actionTypeParameter3, MessageBuffer,
											  sizeof (MessageBuffer), 0);
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &(socket_fds[iIndex]), 0);
					}
					else if (mqACCMData.ip == 0) {
						LogMessage(LOG_LEVEL_DEBUG, "ACCM with no configured target IP: no message sent");
					}
					else {
						LogMessage(LOG_LEVEL_WARNING, "Unable to send ACCM: no valid socket found");
					}
				}
			}
			else if (iCommand == COMM_TRCM && vGetState(GSD) == OBC_STATE_CONNECTED) {
				UtilPopulateTRCMDataStructFromMQ(pcRecvBuffer, sizeof (pcRecvBuffer), &mqTRCMData);
				iIndex = iGetObjectIndexFromObjectIP(mqTRCMData.ip, objectIPs,
													 sizeof (objectIPs) / sizeof (objectIPs[0]));
				if (iIndex != -1) {
					MessageLength =
						encodeTRCMMessage(&mqTRCMData.triggerID, &mqTRCMData.triggerType,
										  &mqTRCMData.triggerTypeParameter1,
										  &mqTRCMData.triggerTypeParameter2,
										  &mqTRCMData.triggerTypeParameter3, MessageBuffer,
										  sizeof (MessageBuffer), 0);
					UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &(socket_fds[iIndex]), 0);
				}
				else
					LogMessage(LOG_LEVEL_WARNING, "Unable to send TRCM: no valid socket found");
			}
			else if (iCommand == COMM_EXAC && vGetState(GSD) == OBC_STATE_RUNNING) {
				UtilPopulateEXACDataStructFromMQ(pcRecvBuffer, sizeof (pcRecvBuffer), &mqEXACData);
				int commandIndex;

				if ((commandIndex =
					 findCommandAction(mqEXACData.actionID, commandActions,
									   sizeof (commandActions) / sizeof (commandActions[0]))) != -1) {
					switch (commandActions[commandIndex].command) {
					case ACTION_PARAMETER_VS_SEND_START:
						iIndex =
							iGetObjectIndexFromObjectIP(mqEXACData.ip, objectIPs,
														sizeof (objectIPs) / sizeof (objectIPs[0]));
						if (iIndex != -1) {
							struct timeval startTime;

							TimeSetToCurrentSystemTime(&currentTime);
							TimeSetToGPStime(&startTime, TimeGetAsGPSweek(&currentTime),
											 mqEXACData.executionTime_qmsoW);
							LogPrint("Current time: %ld, Start time: %ld, delay: %u",
									 TimeGetAsUTCms(&currentTime), TimeGetAsUTCms(&startTime),
									 mqEXACData.executionTime_qmsoW);
							MessageLength =
								encodeSTRTMessage(&startTime, MessageBuffer, sizeof (MessageBuffer), 0);
							UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &socket_fds[iIndex],
											0);
						}
						else if (mqEXACData.ip == 0) {
							LogMessage(LOG_LEVEL_DEBUG,
									   "Delayed STRT with no configured target IP: no message sent");
						}
						else {
							LogMessage(LOG_LEVEL_WARNING,
									   "Unable to send delayed STRT: no valid socket found");
						}
						break;
					default:
						LogMessage(LOG_LEVEL_WARNING, "Unimplemented EXAC test scenario command");
					}
				}
				else {
					iIndex =
						iGetObjectIndexFromObjectIP(mqEXACData.ip, objectIPs,
													sizeof (objectIPs) / sizeof (objectIPs[0]));
					if (iIndex != -1) {
						struct timeval executionTime;

						TimeSetToGPStime(&executionTime, TimeGetAsGPSweek(&currentTime),
										 mqEXACData.executionTime_qmsoW);
						MessageLength =
							encodeEXACMessage(&mqEXACData.actionID, &executionTime, MessageBuffer,
											  sizeof (MessageBuffer), 0);
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &(socket_fds[iIndex]), 0);
					}
					else if (mqEXACData.ip == 0) {
						LogMessage(LOG_LEVEL_DEBUG, "EXAC with no configured target IP: no message sent");
					}
					else {
						LogMessage(LOG_LEVEL_WARNING, "Unable to send EXAC: no valid socket found");
					}
				}
			}
			else if (iCommand == COMM_CONNECT && vGetState(GSD) == OBC_STATE_INITIALIZED) {
				LogMessage(LOG_LEVEL_INFO, "CONNECT received");
				LOG_SEND(LogBuffer, "[ObjectControl] CONNECT received.");

				/* Connect and send drive files */
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {

					UtilSetObjectPositionIP(&OP[iIndex], object_address_name[iIndex]);
					float altitude = (float)OriginPosition.Altitude;

					MessageLength =
						encodeOSEMMessage(&OriginPosition.Latitude, &OriginPosition.Longitude,
										  &altitude, NULL, NULL, NULL, MessageBuffer,
										  sizeof (MessageBuffer), 0);
					if (MessageLength < 0) {
						util_error("OSEM encoding error");
					}

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

					} while (iExit == 0 && iResult < 0 && DisconnectU8 == 0);

					if (iResult >= 0) {
						/* Send OSEM command in mq so that we get some information like GPSweek, origin (latitude,logitude,altitude in gps coordinates) */
						LogMessage(LOG_LEVEL_INFO, "Sending OSEM");
						LOG_SEND(LogBuffer, "[ObjectControl] Sending OSEM.");

						//Restore the buffers
						DataDictionaryGetOriginLatitudeC8(GSD, OriginLatitude, SMALL_BUFFER_SIZE_0);
						DataDictionaryGetOriginLongitudeC8(GSD, OriginLongitude, SMALL_BUFFER_SIZE_0);
						DataDictionaryGetOriginAltitudeC8(GSD, OriginAltitude, SMALL_BUFFER_SIZE_0);

						memset(pcSendBuffer, 0, sizeof (pcSendBuffer));
						snprintf(pcSendBuffer, sizeof (pcSendBuffer), "%u;", TimeGetAsGPSweek(&currentTime));
						snprintf(pcSendBuffer + strlen(pcSendBuffer),
								 sizeof (pcSendBuffer) - strlen(pcSendBuffer), "%s;%s;%s;%s;", OriginLatitude,
								 OriginLongitude, OriginAltitude, OriginHeading);

						if (iCommSend(COMM_OSEM, pcSendBuffer, strlen(pcSendBuffer) + 1) < 0) {
							LogMessage(LOG_LEVEL_ERROR,
									   "Fatal communication fault when sending OSEM command - entering error state");
							vSetState(OBC_STATE_ERROR, GSD);
							objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
						}
						LogPrint("OSEM msglen: %ld", MessageLength);
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength, &socket_fds[iIndex], 0);

						/* Here we send TRAJ, if the IP-address is not operating with a dynamic trajectory */
						if (strstr(DTMReceivers, object_address_name[iIndex]) == NULL) {
							LogMessage(LOG_LEVEL_INFO, "Sending TRAJ to %s", object_address_name[iIndex]);
							if (ObjectControlSendTRAJMessage(object_traj_file[iIndex], &socket_fds[iIndex], 0)
								== -1) {
								LogMessage(LOG_LEVEL_ERROR, "Failed to send TRAJ to %s",
										   object_address_name[iIndex]);
								continue;
							}
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
							struct timeval syncPointTime, syncStopTime;

							TimeSetToUTCms(&syncPointTime, (int64_t) (ASP[i].SlaveTrajSyncTime * 1000.0f));
							TimeSetToUTCms(&syncStopTime, (int64_t) (ASP[i].SlaveSyncStopTime * 1000.0f));
							if (TEST_SYNC_POINTS == 1 && iIndex == 1) {
								/*Send SYPM to slave */
								MessageLength =
									encodeSYPMMessage(syncPointTime, syncStopTime, MessageBuffer,
													  sizeof (MessageBuffer), 0);
								UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
												&socket_fds[iIndex], 0);
							}
							else if (TEST_SYNC_POINTS == 0
									 && strstr(object_address_name[iIndex], ASP[i].SlaveIP) != NULL) {
								/*Send SYPM to slave */
								MessageLength =
									encodeSYPMMessage(syncPointTime, syncStopTime, MessageBuffer,
													  sizeof (MessageBuffer), 0);
								UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
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
				objectControlServerStatus = CONTROL_CENTER_STATUS_READY;

				if (DisconnectU8 == 0) {
					nextHeartbeatTime = currentTime;
					nextAdaptiveSyncMessageTime = currentTime;
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
			else if (iCommand == COMM_GETSTATUS) {

				memset(pcSendBuffer, 0, sizeof (pcSendBuffer));
				sprintf(pcSendBuffer, "%s", MODULE_NAME);
				iCommSend(COMM_GETSTATUS_OK, pcSendBuffer, sizeof (pcSendBuffer));
			}
			else if (iCommand == COMM_GETSTATUS_OK) {
			}

			else {
				LogMessage(LOG_LEVEL_WARNING, "Unhandled command in object control: %d", iCommand);
			}
		}

		if (!iExit) {
			/* Make call periodic */
			sleep_time = iCommand == COMM_INV ? mqEmptyPollPeriod : mqNonEmptyPollPeriod;

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
					objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
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


/*!
 * \brief ObjectControlSendTRAJMessage Sends a trajectory message over a socket based on a trajectory file.
 * \param Filename Path and name of the file containing the trajectory
 * \param Socket TCP socket over which the trajectory is to be sent
 * \param debug Flag for enabling debugging
 * \return Number of bytes sent, or -1 in case of an error
 */
ssize_t ObjectControlSendTRAJMessage(const char *Filename, int *Socket, const char debug) {
	FILE *fd;
	char *line;
	size_t len = 0;
	ssize_t read;
	ssize_t printedBytes = 0, totalPrintedBytes = 0;
	int socketOptions;
	TrajectoryFileHeader fileHeader;
	TrajectoryFileLine fileLine;
	char messageBuffer[TRAJECTORY_TX_BUFFER_SIZE];
	size_t remainingBufferSpace = sizeof (messageBuffer);
	char *messageBufferPosition = messageBuffer;

	memset(&fileHeader, 0, sizeof (fileHeader));
	memset(&fileLine, 0, sizeof (fileLine));

	// Check file format before doing anything
	if (UtilCheckTrajectoryFileFormat(Filename, strlen(Filename)) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Incorrect trajectory file format - cannot proceed to send message");
		return -1;
	}

	// Save socket settings and set it to blocking
	int retval = fcntl(*Socket, F_GETFL);

	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error getting socket options with fcntl");
		return -1;
	}
	socketOptions = retval;

	retval = fcntl(*Socket, F_SETFL, socketOptions & ~O_NONBLOCK);
	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error setting socket options with fcntl");
		return -1;
	}

	// Open the file and parse header
	fd = fopen(Filename, "r");
	if (fd == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to open file <%s>", Filename);
		return -1;
	}

	read = getline(&line, &len, fd);
	if (read == -1 || (retval = UtilParseTrajectoryFileHeader(line, &fileHeader)) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to parse header of file <%s>", Filename);
		fclose(fd);
		return -1;
	}

	// Generate ISO trajectory message header
	if ((printedBytes = encodeTRAJMessageHeader(fileHeader.ID > UINT16_MAX ? 0 : (uint16_t) fileHeader.ID,
												fileHeader.majorVersion, fileHeader.name,
												strlen(fileHeader.name), fileHeader.numberOfLines,
												messageBufferPosition, remainingBufferSpace, debug)) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Unable to encode trajectory message");
		fclose(fd);
		return -1;
	}

	totalPrintedBytes += printedBytes;
	messageBufferPosition += printedBytes;
	remainingBufferSpace -= (size_t) printedBytes;

	read = getline(&line, &len, fd);
	for (unsigned int i = 0; i < fileHeader.numberOfLines && read != -1; ++i, read = getline(&line, &len, fd)) {

		// Parse file line
		struct timeval relTime;
		CartesianPosition position;
		SpeedType speed;
		AccelerationType acceleration;

		if (UtilParseTrajectoryFileLine(line, &fileLine) == -1) {
			// TODO: how to terminate an ISO message when an error has occurred?
			LogMessage(LOG_LEVEL_ERROR, "Unable to parse line %u of trajectory file <%s>", i + 1, Filename);
			fclose(fd);
			return -1;
		}

		relTime.tv_sec = (time_t) fileLine.time;
		relTime.tv_usec = (time_t) ((fileLine.time - relTime.tv_sec) * 1000000);
		position.xCoord_m = fileLine.xCoord;
		position.yCoord_m = fileLine.yCoord;
		position.isPositionValid = fileLine.zCoord != NULL;
		position.zCoord_m = position.isPositionValid ? *fileLine.zCoord : 0;
		position.heading_rad = fileLine.heading;
		position.isHeadingValid = true;
		speed.isLongitudinalValid = fileLine.longitudinalVelocity != NULL;
		speed.isLateralValid = fileLine.lateralVelocity != NULL;
		speed.longitudinal_m_s = fileLine.longitudinalVelocity != NULL ? *fileLine.longitudinalVelocity : 0;
		speed.lateral_m_s = fileLine.lateralVelocity != NULL ? *fileLine.lateralVelocity : 0;
		acceleration.isLongitudinalValid = fileLine.longitudinalAcceleration != NULL;
		acceleration.isLateralValid = fileLine.lateralAcceleration != NULL;
		acceleration.longitudinal_m_s2 =
			fileLine.longitudinalAcceleration != NULL ? *fileLine.longitudinalAcceleration : 0;

		acceleration.lateral_m_s2 = fileLine.lateralAcceleration != NULL ? *fileLine.lateralAcceleration : 0;

		// Print to buffer
		if ((printedBytes = encodeTRAJMessagePoint(&relTime, position, speed, acceleration,
												   (float)fileLine.curvature, messageBufferPosition,
												   remainingBufferSpace, debug)) == -1) {

			if (errno == ENOBUFS) {
				// Reached the end of buffer, send buffered data and
				// try again
				UtilSendTCPData(MODULE_NAME, messageBuffer, messageBufferPosition - messageBuffer, Socket,
								debug);

				messageBufferPosition = messageBuffer;
				remainingBufferSpace = sizeof (messageBuffer);
				if ((printedBytes =
					 encodeTRAJMessagePoint(&relTime, position, speed, acceleration, fileLine.curvature,
											messageBufferPosition, remainingBufferSpace, debug)) == -1) {
					// TODO how to terminate an ISO message when an error has occurred?
					LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message point");
					fclose(fd);
					return -1;
				}
				messageBufferPosition += printedBytes;
				totalPrintedBytes += printedBytes;
				remainingBufferSpace -= (size_t) printedBytes;
			}
			else {
				// TODO how to terminate an ISO message when an error has occurred?
				LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message point");
				fclose(fd);
				return -1;
			}
		}
		else {
			totalPrintedBytes += printedBytes;
			messageBufferPosition += printedBytes;
			remainingBufferSpace -= (size_t) printedBytes;
		}
	}

	// Trajectory message footer
	if ((printedBytes = encodeTRAJMessageFooter(messageBufferPosition, remainingBufferSpace, debug)) == -1) {
		if (errno == ENOBUFS) {
			// Buffer was filled: send the data and try to encode it again
			UtilSendTCPData(MODULE_NAME, messageBuffer, messageBufferPosition - messageBuffer, Socket, debug);
			messageBufferPosition = messageBuffer;
			remainingBufferSpace = sizeof (messageBuffer);
			if ((printedBytes =
				 encodeTRAJMessageFooter(messageBufferPosition, remainingBufferSpace, debug)) == -1) {
				// TODO how to terminate an ISO message when an error has occurred?
				LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message footer");
				fclose(fd);
				return -1;
			}
			messageBufferPosition += printedBytes;
			totalPrintedBytes += printedBytes;
			remainingBufferSpace -= (size_t) printedBytes;
		}
		else {
			// TODO how to terminate an ISO message when an error has occurred?
			LogMessage(LOG_LEVEL_ERROR, "Error encoding trajectory message footer");
			fclose(fd);
			return -1;
		}
	}
	else {
		totalPrintedBytes += printedBytes;
		messageBufferPosition += printedBytes;
		remainingBufferSpace -= (size_t) printedBytes;
	}

	UtilSendTCPData(MODULE_NAME, messageBuffer, messageBufferPosition - messageBuffer, Socket, debug);

	fclose(fd);

	// Reset socket settings
	retval = fcntl(*Socket, F_SETFL, socketOptions);
	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error setting socket options with fcntl");
		return -1;
	}

	return totalPrintedBytes;
}


/*!
 * \brief resetCommandActionList Resets the list back to unset values: 0 for the IP and ID, and
 *			::ACTION_PARAMETER_UNAVAILABLE for the command.
 * \param commandActions List of command actions
 * \param numberOfElementsInList Number of elements in the entire list
 */
void resetCommandActionList(TestScenarioCommandAction commandActions[], const int numberOfElementsInList) {
	for (int i = 0; i < numberOfElementsInList; ++i) {
		commandActions[i].ip = 0;
		commandActions[i].command = ACTION_PARAMETER_UNAVAILABLE;
		commandActions[i].actionID = 0;
	}
}


/*!
 * \brief addCommandToActionList Adds a command to the command action list by searching for the
 *			first occurrence of ::ACTION_PARAMETER_UNAVAILABLE and replacing it with the chosen
 *			command. An error is returned if no such occurrence is found.
 * \param command Command to be added to list
 * \param commandActions List of all command actions
 * \param numberOfElementsInList Number of elements in the entire list
 * \return 0 on success, -1 otherwise
 */
int addCommandToActionList(const TestScenarioCommandAction command,
						   TestScenarioCommandAction commandActions[], const int numberOfElementsInList) {
	for (int i = 0; i < numberOfElementsInList; ++i) {
		if (commandActions[i].command == ACTION_PARAMETER_UNAVAILABLE) {
			commandActions[i] = command;
			return 0;
		}
	}
	errno = ENOBUFS;
	return -1;
}


/*!
 * \brief hasDelayedStart Checks the command action list for any delayed start configurations
 *			for the chosen object IP.
 * \param objectIP IP of the object to be checked for delayed start
 * \param commandActions List of all configured command actions
 * \param numberOfElementsInList Number of elements in the entire list
 * \return Boolean value indicating if the object has a delayed start configuration
 */
int hasDelayedStart(const in_addr_t objectIP, const TestScenarioCommandAction commandActions[],
					const int numberOfElementsInList) {
	for (int i = 0; i < numberOfElementsInList; ++i) {
		if (commandActions[i].ip == objectIP && commandActions[i].command == ACTION_PARAMETER_VS_SEND_START) {
			return 1;
		}
	}
	return 0;
}

/*!
 * \brief findCommandAction Finds the command action with specified action ID.
 * \param actionID ID of the action to be found
 * \param commandActions List of all configured command actions
 * \param numberOfElementsInList Number of elements in the entire list
 * \return Index of the command action, or -1 if not found
 */
int findCommandAction(const uint16_t actionID, const TestScenarioCommandAction commandActions[],
					  const int numberOfElementsInList) {
	for (int i = 0; i < numberOfElementsInList; ++i) {
		if (commandActions[i].actionID == actionID
			&& commandActions[i].command != ACTION_PARAMETER_UNAVAILABLE) {
			return i;
		}
	}
	return -1;
}


int iGetObjectIndexFromObjectIP(in_addr_t ipAddr, in_addr_t objectIPs[], unsigned int numberOfObjects) {
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


size_t uiRecvMonitor(int *sockfd, char *buffer, size_t length) {
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
	int iForceObjectToLocalhost = DEFAULT_FORCE_OBJECT_TO_LOCALHOST;
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
	case OBC_STATE_REMOTECTRL:
		return &tFromRemoteControl;
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
	if (requestedState == OBC_STATE_ARMED || requestedState == OBC_STATE_IDLE
		|| requestedState == OBC_STATE_REMOTECTRL) {
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

StateTransitionResult tFromRemoteControl(OBCState_t * currentState, OBCState_t requestedState) {
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
