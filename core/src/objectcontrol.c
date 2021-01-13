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
#include "journal.h"

#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
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

#define RECV_MESSAGE_BUFFER 6200
#define BUFFER_SIZE_3100 3100
#define TRAJ_FILE_HEADER_ROW 256

#define OC_SLEEP_TIME_EMPTY_MQ_S 0
#define OC_SLEEP_TIME_EMPTY_MQ_NS 1000000
#define OC_SLEEP_TIME_NONEMPTY_MQ_S 0
#define OC_SLEEP_TIME_NONEMPTY_MQ_NS 0

#define OBJECT_CONNECTION_TIMEOUT_S 10
#define OBJECT_CONNECTION_RETRY_PERIOD_MS 1000

#define MAX_NETWORK_DELAY_USEC 100000

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


typedef struct {
	int commandSocket;
	int monitorSocket;
	struct sockaddr_in objectCommandAddress;
	struct sockaddr_in objectMonitorAddress;
} ObjectConnection;


typedef struct {
	uint32_t sourceID;
	unsigned int numberOfTargets;
	uint32_t *targetIDs;
	int isActive;
} DataInjectionMap;

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
static void disconnectObject(ObjectConnection * objectConnection);
static void disconnectAllObjects(ObjectConnection objectConnections[], const unsigned int numberOfObjects);
static int connectAllObjects(ObjectConnection objectConnections[], const unsigned int numberOfObjects);
static int configureAllObjects(ObjectConnection objectConnections[],
							   const GeoPosition originPosition,
							   const struct timeval currentTime,
							   const char trajectoryFiles[MAX_OBJECTS][MAX_FILE_PATH],
							   const unsigned int numberOfObjects);
static int configureAdaptiveSynchronizationPoints(const char trajectoryFiles[MAX_OBJECTS][MAX_FILE_PATH],
												  const ObjectConnection objectConnections[],
												  ObjectPosition objectPositions[],
												  const unsigned int numberOfObjects,
												  const AdaptiveSyncPoint ASP[],
												  const unsigned int adaptiveSyncPointCount);
static int configureObjectDataInjection(DataInjectionMap injectionMaps[], const uint32_t transmitterIDs[],
										const unsigned int numberOfObjects);
static int parseDataInjectionSetting(const char objectFilePath[MAX_FILE_PATH],
									 DataInjectionMap injectionMaps[], const unsigned int numberOfMaps);
static int sendDataInjectionMessages(const ObjectDataType * monitorData,
									 const DataInjectionMap dataInjectionMaps[],
									 const ObjectConnection objectConnections[],
									 const uint32_t transmitterIDs[], const unsigned int numberOfObjects);
static void *objectConnectorThread(void *args);
static int checkObjectConnections(ObjectConnection objectConnections[], const struct timeval monitorTimeout,
								  const unsigned int numberOfObjects);
static int hasRemoteDisconnected(int *sockfd);
static int handleObjectPropertiesData(const ObjectPropertiesType * properties,
									  const DataInjectionMap injectionMaps[],
									  const ObjectConnection objectConnections[],
									  const uint32_t transmitterIDs[], const unsigned int numberOfObjects);
static ssize_t uiRecvMonitor(int *sockfd, char *buffer, size_t length);
static int getObjectIndexFromIP(const in_addr_t ipAddr, const ObjectConnection objectConnections[],
								unsigned int numberOfObjects);
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
							ObjectConnection objectConnections[],
							uint32_t objectIDs[MAX_OBJECTS], unsigned int *nbr_objects);
static int readMonitorDataTimeoutSetting(struct timeval *timeout);

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

	C8 object_traj_file[MAX_OBJECTS][MAX_FILE_PATH];
	char ipString[INET_ADDRSTRLEN];
	U32 object_transmitter_ids[MAX_OBJECTS];
	ObjectConnection objectConnections[MAX_OBJECTS];
	TestScenarioCommandAction commandActions[MAX_OBJECTS];
	DataInjectionMap dataInjectionMaps[MAX_OBJECTS];

	memset(dataInjectionMaps, 0, sizeof (dataInjectionMaps));
	unsigned int nbr_objects = 0;
	enum COMMAND iCommand;
	U8 pcRecvBuffer[RECV_MESSAGE_BUFFER];
	C8 pcTempBuffer[512];
	C8 MessageBuffer[BUFFER_SIZE_3100];
	I32 iIndex = 0;
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
	struct timeval monitorDataTimeout;	//!< Timeout after N missing monitor messages i.e. after this long

	const struct timeval adaptiveSyncMessagePeriod = heartbeatPeriod;

	FILE *fd;
	ssize_t MessageLength;
	C8 *MiscPtr;
	C8 MiscText[SMALL_BUFFER_SIZE_0];
	U32 StartTimeU32 = 0;
	U32 CurrentTimeU32 = 0;
	U32 OldTimeU32 = 0;
	U64 TimeCap1, TimeCap2;
	struct timeval CurrentTimeStruct;

	ObjectPosition OP[MAX_OBJECTS];

	memset(OP, 0, sizeof (OP));
	C8 OriginLatitude[SMALL_BUFFER_SIZE_0], OriginLongitude[SMALL_BUFFER_SIZE_0],
		OriginAltitude[SMALL_BUFFER_SIZE_0], OriginHeading[SMALL_BUFFER_SIZE_0];
	C8 TextBuffer[SMALL_BUFFER_SIZE_0];
	dbl OriginLatitudeDbl = DEFAULT_ORIGIN_LAT;
	dbl OriginLongitudeDbl = DEFAULT_ORIGIN_LOG;
	dbl OriginAltitudeDbl = DEFAULT_ORIGIN_ALT;
	dbl OriginHeadingDbl = DEFAULT_ORIGIN_HEADING;
	C8 pcSendBuffer[MBUS_MAX_DATALEN];
	ObjectDataType monitorData;
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
	C8 VOILReceivers[SMALL_BUFFER_SIZE_254];
	C8 DTMReceivers[SMALL_BUFFER_SIZE_254];


	FILE *TempFd;

	// Create log
	LogInit(MODULE_NAME, logLevel);
	LogMessage(LOG_LEVEL_INFO, "Object control task running with PID: %i", getpid());

	// Create test journal
	if (JournalInit(MODULE_NAME) == -1) {
		util_error("Unable to create test journal");
	}

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

	ObjectEnabledType objectEnabledStatus;
	uint32_t transmitterId;

	while (!iExit) {
		TimeSetToCurrentSystemTime(&currentTime);

		if (vGetState(GSD) == OBC_STATE_ERROR) {
			objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
			MessageLength =
				encodeHEABMessage(&currentTime, objectControlServerStatus, MessageBuffer,
								  sizeof (MessageBuffer), 0);

			DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);

			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex], &objectEnabledStatus);
				if (objectEnabledStatus == OBJECT_ENABLED)
					UtilSendUDPData(MODULE_NAME, &objectConnections[iIndex].monitorSocket,
									&objectConnections[iIndex].objectMonitorAddress,
									MessageBuffer, MessageLength, 0);
			}
		}

		// Heartbeat
		if ((vGetState(GSD) == OBC_STATE_RUNNING || vGetState(GSD) == OBC_STATE_ARMED
			 || vGetState(GSD) == OBC_STATE_CONNECTED || vGetState(GSD) == OBC_STATE_REMOTECTRL)
			&& timercmp(&currentTime, &nextHeartbeatTime, >)) {

			DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);

			timeradd(&nextHeartbeatTime, &heartbeatPeriod, &nextHeartbeatTime);
			MessageLength =
				encodeHEABMessage(&currentTime, objectControlServerStatus, MessageBuffer,
								  sizeof (MessageBuffer), 0);

			// Transmit heartbeat to all objects
			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex], &objectEnabledStatus);
				if (objectEnabledStatus == OBJECT_ENABLED)
					UtilSendUDPData(MODULE_NAME, &objectConnections[iIndex].monitorSocket,
									&objectConnections[iIndex].objectMonitorAddress,
									MessageBuffer, MessageLength, 0);
			}
		}

		// Every iteration while connected, do
		if (vGetState(GSD) == OBC_STATE_RUNNING || vGetState(GSD) == OBC_STATE_CONNECTED
			|| vGetState(GSD) == OBC_STATE_ARMED || vGetState(GSD) == OBC_STATE_REMOTECTRL) {

			// Check if any object has disconnected - if so, disconnect all objects and return to idle
			if (checkObjectConnections(objectConnections, monitorDataTimeout, nbr_objects)) {
				disconnectAllObjects(objectConnections, nbr_objects);
				iCommSend(COMM_DISCONNECT, NULL, 0);
				vSetState(OBC_STATE_IDLE, GSD);
			}

			char buffer[RECV_MESSAGE_BUFFER];
			size_t receivedMONRData = 0;
			size_t receivedTCPData = 0;

			CurrentTimeU32 =
				((GPSTime->GPSSecondsOfWeekU32 * 1000 + (U32) TimeControlGetMillisecond(GPSTime)) << 2) +
				GPSTime->MicroSecondU16;

			 /*MTSP*/ if (timercmp(&currentTime, &nextAdaptiveSyncMessageTime, >)) {

				timeradd(&nextAdaptiveSyncMessageTime, &adaptiveSyncMessagePeriod,
						 &nextAdaptiveSyncMessageTime);

				struct timeval estSyncPointTime;

				TimeSetToGPStime(&estSyncPointTime, TimeGetAsGPSweek(&currentTime), ASPData.MTSPU32);
				DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
							  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
							  ipString, sizeof (ipString));
					for (int i = 0; i < SyncPointCount; i++) {
						if (TEST_SYNC_POINTS == 0
							&& strstr(ipString, ASP[i].SlaveIP) != NULL
							&& ASPData.MTSPU32 > 0 && ASPData.TimeToSyncPointDbl > -1) {

							/*Send Master time to adaptive sync point */
							MessageLength =
								encodeMTSPMessage(&estSyncPointTime, MessageBuffer, sizeof (MessageBuffer),
												  0);
							DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
																	&objectEnabledStatus);
							if (objectEnabledStatus == OBJECT_ENABLED)
								UtilSendUDPData(MODULE_NAME, &objectConnections[iIndex].monitorSocket,
												&objectConnections[iIndex].objectMonitorAddress,
												MessageBuffer, MessageLength, 0);
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


			DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);
			for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
				inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
						  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
						  ipString, sizeof (ipString));
				DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex], &objectEnabledStatus);
				if (objectEnabledStatus == OBJECT_ENABLED) {
					memset(buffer, 0, sizeof (buffer));
					receivedMONRData =
						uiRecvMonitor(&objectConnections[iIndex].monitorSocket, buffer, sizeof (buffer));
				}

				if (receivedMONRData > 0 && getISOMessageType(buffer, receivedMONRData, 0) == MESSAGE_ID_MONR) {
					LogMessage(LOG_LEVEL_DEBUG, "Recieved %d bytes of new data from %s %u",
							   receivedMONRData, ipString,
							   objectConnections[iIndex].objectMonitorAddress.sin_port);

					monitorData.ClientIP = objectConnections[iIndex].objectMonitorAddress.sin_addr.s_addr;
					TimeSetToCurrentSystemTime(&currentTime);
					if (decodeMONRMessage
						(buffer, receivedMONRData, currentTime, &monitorData.ClientID, &monitorData.MonrData,
						 0) < 0) {
						LogMessage(LOG_LEVEL_INFO, "Error decoding MONR from %s: disconnecting object",
								   ipString);

						disconnectObject(&objectConnections[iIndex]);
						// TODO smarter way of handling?
						continue;
					}
					else {
						TimeSetToCurrentSystemTime(&monitorData.lastDataUpdate);
						struct timeval monitorDataAge;

						timersub(&monitorData.lastDataUpdate, &monitorData.MonrData.timestamp,
								 &monitorDataAge);
						if (monitorDataAge.tv_sec || labs(monitorDataAge.tv_usec) > MAX_NETWORK_DELAY_USEC) {
							LogMessage(LOG_LEVEL_WARNING,
									   "Network delay from object %u exceeds 100 ms (%ld ms delay)",
									   object_transmitter_ids[iIndex], TimeGetAsUTCms(&monitorDataAge));
						}
					}

					// TEMPORARY FIX: if an object sends with transmitter ID not matching the local, modify locally
					//      stored transmitter ID
					uint32_t localTransmitterID;

					if (DataDictionaryGetObjectTransmitterIDByIP(monitorData.ClientIP, &localTransmitterID) ==
						READ_OK) {
						if (monitorData.ClientID != localTransmitterID) {
							LogMessage(LOG_LEVEL_WARNING,
									   "Modifying local transmitter ID %u to %u in lieu of ability to set remote",
									   localTransmitterID, monitorData.ClientID);
							DataDictionaryModifyTransmitterIDByIP(monitorData.ClientIP, monitorData.ClientID);
						}
					}
					else {
						LogMessage(LOG_LEVEL_ERROR, "Unable to read monitor data from shared memory");
						// TODO: Handle the error
					}
					// Store monitor data in shared memory
					if (DataDictionarySetMonitorData
						(monitorData.ClientID, &monitorData.MonrData, &currentTime) == UNDEFINED) {
						LogMessage(LOG_LEVEL_ERROR, "Unable to write monitor data to shared memory");
						// TODO: handle the error
					}

					// Log the data
					if (ObjectcontrolExecutionMode == OBJECT_CONTROL_CONTROL_MODE) {
						JournalRecordMonitorData(&monitorData);
					}

					memset(buffer, 0, sizeof (buffer));
					UtilObjectDataToString(monitorData, buffer, sizeof (buffer));

					if (ASPData.MTSPU32 != 0) {
						//Add MTSP to MONR if not 0
						sprintf(buffer + strlen(buffer), "%" PRIu32 ";", ASPData.MTSPU32);
					}

					//Ok, let's do the ASP
					for (int i = 0; i < SyncPointCount; i++) {
						if (TEST_SYNC_POINTS == 0
							&& strstr(ipString, ASP[i].MasterIP) != NULL
							&& CurrentTimeU32 > StartTimeU32 && StartTimeU32 > 0
							&& ASPData.TimeToSyncPointDbl > -1
							/*|| TEST_SYNC_POINTS == 1 && ASP[0].TestPort == object_udp_port[iIndex] && StartTimeU32 > 0 && iIndex == 0 && TimeToSyncPoint > -1 */
							) {
							// Use the util.c function for time here but it soent mather
							gettimeofday(&CurrentTimeStruct, NULL);	//Capture time

							TimeCap1 = (uint64_t) CurrentTimeStruct.tv_sec * 1000 + (uint64_t) CurrentTimeStruct.tv_usec / 1000;	//Calculate initial timestamp

							OP[iIndex].x = monitorData.MonrData.position.xCoord_m;	//Set x and y on OP (ObjectPosition)
							OP[iIndex].y = monitorData.MonrData.position.yCoord_m;

							//OP[iIndex].OrigoDistance = sqrt(pow(OP[iIndex].x,2) + pow(OP[iIndex].y,2)); //Calculate hypotenuse

							// TODO: check use of this function since it should take two lat/long points but is here used with x/y
							UtilCalcPositionDelta(OriginLatitudeDbl, OriginLongitudeDbl,
												  monitorData.MonrData.position.xCoord_m,
												  monitorData.MonrData.position.yCoord_m, &OP[iIndex]);

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

								if (TimeGetAsGPSqmsOfWeek(&monitorData.MonrData.timestamp) % ASPDebugRate ==
									0) {
									printf("%d, %d, %3.3f, %s, %s\n", CurrentTimeU32, StartTimeU32,
										   ASPData.TimeToSyncPointDbl, ipString, ASP[i].MasterIP);
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
						(float)sqrt(pow(monitorData.MonrData.speed.lateral_m_s, 2) +
									pow(monitorData.MonrData.speed.longitudinal_m_s, 2));

					// Pass on the information to the configured injection targets
					sendDataInjectionMessages(&monitorData, dataInjectionMaps, objectConnections,
											  object_transmitter_ids, nbr_objects);
				}
				else if (receivedMONRData > 0) {
					LogMessage(LOG_LEVEL_WARNING, "Received unhandled message on monitoring socket");
				}

				if (objectEnabledStatus == OBJECT_ENABLED) {
					memset(buffer, 0, sizeof (buffer));
					receivedTCPData =
						UtilReceiveTCPData(MODULE_NAME, &objectConnections[iIndex].commandSocket, buffer,
										   sizeof (buffer), 0);

					if (receivedTCPData > 0 && getISOMessageType(buffer, receivedTCPData, 0)
						== MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO) {
						// Received OPRO message
						ObjectPropertiesType properties;

						if (decodeOPROMessage(&properties, buffer, sizeof (buffer), 0) >= 0) {
							DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);
							if (handleObjectPropertiesData(&properties, dataInjectionMaps,
														   objectConnections, object_transmitter_ids,
														   nbr_objects) < 0) {
								LogMessage(LOG_LEVEL_ERROR, "Error handling object property data");
							}
						}
						else {
							LogMessage(LOG_LEVEL_ERROR, "Failed to decode OPRO message");
						}
					}
					else if (receivedTCPData > 0) {
						LogMessage(LOG_LEVEL_WARNING, "Unhandled TCP data received from IP %s",
								   inet_ntop(AF_INET, &objectConnections->objectCommandAddress,
											 ipString, sizeof (ipString)));
					}
				}
			}

		}


		bzero(pcRecvBuffer, RECV_MESSAGE_BUFFER);
		//Have we recieved a command?
		if (iCommRecv(&iCommand, pcRecvBuffer, RECV_MESSAGE_BUFFER, NULL)) {
			LogMessage(LOG_LEVEL_DEBUG, "Received command %d", iCommand);

			DataDictionaryGetObjectTransmitterIDs(object_transmitter_ids, nbr_objects);
			if (iCommand == COMM_ARM && vGetState(GSD) == OBC_STATE_CONNECTED) {

				LogMessage(LOG_LEVEL_INFO, "Sending ARM");
				JournalRecordData(JOURNAL_RECORD_EVENT, "Sending ARM");
				vSetState(OBC_STATE_ARMED, GSD);
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_ARM, MessageBuffer, sizeof (MessageBuffer), 0);

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
															&objectEnabledStatus);
					if (objectEnabledStatus == OBJECT_ENABLED)
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&objectConnections[iIndex].commandSocket, 0);
				}

				objectControlServerStatus = CONTROL_CENTER_STATUS_READY;
			}
			else if (iCommand == COMM_DISARM && vGetState(GSD) == OBC_STATE_ARMED) {

				LogMessage(LOG_LEVEL_INFO, "Sending DISARM");
				JournalRecordData(JOURNAL_RECORD_EVENT, "Sending DISARM");
				vSetState(OBC_STATE_CONNECTED, GSD);

				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_DISARM, MessageBuffer, sizeof (MessageBuffer), 0);

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					/*Send OSTM message */
					DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
															&objectEnabledStatus);
					if (objectEnabledStatus == OBJECT_ENABLED)
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&objectConnections[iIndex].commandSocket, 0);
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

				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					if (!hasDelayedStart
						(objectConnections[iIndex].objectCommandAddress.sin_addr.s_addr, commandActions,
						 sizeof (commandActions) / sizeof (commandActions[0]))) {
						DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
																&objectEnabledStatus);
						if (objectEnabledStatus == OBJECT_ENABLED) {
							UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
											&objectConnections[iIndex].commandSocket, 0);
						}
					}
				}
				vSetState(OBC_STATE_RUNNING, GSD);
				objectControlServerStatus = CONTROL_CENTER_STATUS_RUNNING;

				//printf("OutgoingStartTimeU32 = %d\n", OutgoingStartTimeU32);
				GSD->ScenarioStartTimeU32 = TimeGetAsGPSqmsOfWeek(&startTime) >> 2;
				bzero(MiscText, SMALL_BUFFER_SIZE_0);
				sprintf(MiscText, "%" PRIu32, GSD->ScenarioStartTimeU32 << 2);
				JournalRecordData(JOURNAL_RECORD_EVENT, "Sending START with time %u [second of week]",
								  TimeGetAsGPSSecondOfWeek(&startTime));
			}
			else if (iCommand == COMM_REPLAY) {
				//ObjectcontrolExecutionMode = OBJECT_CONTROL_REPLAY_MODE;
				//LogMessage(LOG_LEVEL_INFO, "Entering REPLAY mode <%s>", pcRecvBuffer);
				LogMessage(LOG_LEVEL_WARNING, "REPLAY mode support deprecated");
			}
			else if (iCommand == COMM_ABORT && vGetState(GSD) == OBC_STATE_RUNNING) {
				vSetState(OBC_STATE_CONNECTED, GSD);
				objectControlServerStatus = CONTROL_CENTER_STATUS_ABORT;
				LogMessage(LOG_LEVEL_WARNING, "ABORT received");
				JournalRecordData(JOURNAL_RECORD_EVENT, "ABORT received");
			}
			else if (iCommand == COMM_REMOTECTRL_ENABLE) {
				vSetState(OBC_STATE_REMOTECTRL, GSD);
				// TODO: objectControlServerStatus = something
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_REMOTE_CONTROL, MessageBuffer, sizeof (MessageBuffer),
									  0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
							  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
							  ipString, sizeof (ipString));
					LogMessage(LOG_LEVEL_INFO, "Setting object with IP %s to remote control mode", ipString);
					DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
															&objectEnabledStatus);
					if (objectEnabledStatus == OBJECT_ENABLED)
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&objectConnections[iIndex].commandSocket, 0);
				}
				// TODO: check objects' states
				LogMessage(LOG_LEVEL_INFO, "Enabled remote control mode");
			}
			else if (iCommand == COMM_REMOTECTRL_DISABLE) {
				// TODO set objects' states to connected
				MessageLength =
					encodeOSTMMessage(OBJECT_COMMAND_DISARM, MessageBuffer, sizeof (MessageBuffer), 0);
				for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
					inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
							  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
							  ipString, sizeof (ipString));
					LogMessage(LOG_LEVEL_INFO, "Setting object with IP %s to disarmed mode", ipString);
					DataDictionaryGetObjectEnableStatusById(object_transmitter_ids[iIndex],
															&objectEnabledStatus);
					if (objectEnabledStatus == OBJECT_ENABLED)
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&objectConnections[iIndex].commandSocket, 0);
				}
				// TODO: check objects' states
				// TODO: objectControlServerStatus = something
				vSetState(OBC_STATE_CONNECTED, GSD);
				LogMessage(LOG_LEVEL_INFO, "Disabled remote control mode");
			}
			else if (iCommand == COMM_REMOTECTRL_MANOEUVRE) {
				RemoteControlCommandType rcCommand;

				// TODO check size of received data
				memcpy(&rcCommand, pcRecvBuffer, sizeof (rcCommand));
				LogMessage(LOG_LEVEL_INFO, "Received remote control manoeuvre for object with IP %s", inet_ntop(AF_INET, &rcCommand.objectIP, ipString, sizeof (ipString)));	// TODO print command type
				if (vGetState(GSD) == OBC_STATE_REMOTECTRL) {
					switch (rcCommand.manoeuvre) {
					case MANOEUVRE_BACK_TO_START:
						iIndex = getObjectIndexFromIP(rcCommand.objectIP, objectConnections, nbr_objects);
						DataDictionaryGetObjectEnableStatusByIp(rcCommand.objectIP, &objectEnabledStatus);
						if (iIndex != -1 && objectEnabledStatus == OBJECT_ENABLED) {
							inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
									  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
									  ipString, sizeof (ipString));
							LogMessage(LOG_LEVEL_INFO, "Sending back to start command to object with IP %s",
									   ipString);
							MessageLength =
								encodeRCMMMessage(rcCommand.manoeuvre, MessageBuffer, sizeof (MessageBuffer),
												  0);
							if (MessageLength > 0) {
								UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
												&objectConnections[iIndex].commandSocket, 0);
							}
							else {
								LogMessage(LOG_LEVEL_ERROR, "Error encoding RCMM message");
							}
						}
						else {
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
			else if (iCommand == COMM_ENABLE_OBJECT) {
				ObjectEnabledCommandType enableCommand;

				memcpy(&enableCommand, pcRecvBuffer, sizeof (enableCommand));
				if (vGetState(GSD) != OBC_STATE_RUNNING && vGetState(GSD) != OBC_STATE_ARMED) {
					iIndex = getObjectIndexFromIP(enableCommand.objectIP, objectConnections, nbr_objects);
					DataDictionaryGetObjectTransmitterIDByIP(enableCommand.objectIP, &transmitterId);
					if (transmitterId > 0) {
						inet_ntop(objectConnections[iIndex].objectMonitorAddress.sin_family,
								  &objectConnections[iIndex].objectMonitorAddress.sin_addr,
								  ipString, sizeof (ipString));
						DataDictionarySetObjectEnableStatus(transmitterId, enableCommand.Enabled);
						if (enableCommand.Enabled == OBJECT_ENABLED)
							LogMessage(LOG_LEVEL_INFO, "Enable object %s.", ipString);
						else if (enableCommand.Enabled == OBJECT_DISABLED)
							LogMessage(LOG_LEVEL_INFO, "Disable object %s.", ipString);
					}
					else
						LogMessage(LOG_LEVEL_ERROR, "Disable/Enabled of object IP %x not found!",
								   enableCommand.objectIP);
				}
				else {
					LogMessage(LOG_LEVEL_WARNING, "Not allowed to enable/disable objects in RUNNING state!");
				}
			}
			else if (iCommand == COMM_INIT) {
				JournalRecordData(JOURNAL_RECORD_EVENT, "INIT received");
				nbr_objects = 0;
				int initSuccessful = true;

				// Get objects; name and drive file
				if (iFindObjectsInfo
					(object_traj_file, objectConnections, object_transmitter_ids, &nbr_objects)
					== 0) {

					// Reset preexisting stored monitor data
					if (DataDictionarySetNumberOfObjects(0) != WRITE_OK) {
						LogMessage(LOG_LEVEL_ERROR, "Error clearing old object data");
						initSuccessful = false;
					}
					else {
						LogMessage(LOG_LEVEL_INFO, "Cleared previous object data");
					}

					// Get number of allowed missing monitor messages before abort
					if (readMonitorDataTimeoutSetting(&monitorDataTimeout) == -1) {
						LogMessage(LOG_LEVEL_ERROR, "Error reading monitor data timeout setting");
						initSuccessful = false;
					}

					// Enable all objects at INIT
					ObjectDataType objectData;

					for (iIndex = 0; iIndex < nbr_objects; iIndex++) {
						objectData.Enabled = OBJECT_ENABLED;
						objectData.ClientIP = objectConnections[iIndex].objectCommandAddress.sin_addr.s_addr;
						objectData.ClientID = object_transmitter_ids[iIndex];
						objectData.lastDataUpdate.tv_sec = 0;
						objectData.lastDataUpdate.tv_usec = 0;
						objectData.propertiesReceived = 0;
						if (DataDictionarySetObjectData(&objectData) != WRITE_OK) {
							LogMessage(LOG_LEVEL_ERROR, "Error setting object data");
							initSuccessful = false;
						}
					}

					resetCommandActionList(commandActions,
										   sizeof (commandActions) / sizeof (commandActions[0]));

					for (iIndex = 0; iIndex < nbr_objects; ++iIndex) {
						objectConnections[iIndex].objectMonitorAddress.sin_port = htons(SAFETY_CHANNEL_PORT);
						objectConnections[iIndex].objectCommandAddress.sin_port = htons(CONTROL_CHANNEL_PORT);
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

						for (int i = 0; i < SyncPointCount; i++) {
							UtilSetAdaptiveSyncPoint(&ASP[i], fd, 0);
							if (TEST_SYNC_POINTS == 1)
								ASP[i].TestPort = SAFETY_CHANNEL_PORT;
						}
						fclose(fd);
					}

					if (configureObjectDataInjection(dataInjectionMaps, object_transmitter_ids, nbr_objects)
						== -1) {
						LogMessage(LOG_LEVEL_ERROR, "Error when configuring object data injection.");
						initSuccessful = false;
					}

					//Remove temporary file
					remove(TEMP_LOG_FILE);
					if (USE_TEMP_LOGFILE) {
						//Create temporary file
						TempFd = fopen(TEMP_LOG_FILE, "w+");
					}
				}
				else {
					initSuccessful = false;
					LogMessage(LOG_LEVEL_INFO, "Object info was not processed successfully");
				}

				if (initSuccessful) {
					vSetState(OBC_STATE_INITIALIZED, GSD);
					LogMessage(LOG_LEVEL_INFO, "Successfully initialized");
				}
				else {
					LogMessage(LOG_LEVEL_INFO, "Initialization failed");
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
					iIndex = getObjectIndexFromIP(mqACCMData.ip, objectConnections, nbr_objects);
					DataDictionaryGetObjectEnableStatusByIp(mqACCMData.ip, &objectEnabledStatus);
					if (iIndex != -1 && objectEnabledStatus == OBJECT_ENABLED) {
						MessageLength =
							encodeACCMMessage(&mqACCMData.actionID, &mqACCMData.actionType,
											  &mqACCMData.actionTypeParameter1,
											  &mqACCMData.actionTypeParameter2,
											  &mqACCMData.actionTypeParameter3, MessageBuffer,
											  sizeof (MessageBuffer), 0);
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&(objectConnections[iIndex].commandSocket), 0);
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
				iIndex = getObjectIndexFromIP(mqTRCMData.ip, objectConnections, nbr_objects);
				DataDictionaryGetObjectEnableStatusByIp(mqTRCMData.ip, &objectEnabledStatus);
				if (iIndex != -1 && objectEnabledStatus == OBJECT_ENABLED) {
					MessageLength =
						encodeTRCMMessage(&mqTRCMData.triggerID, &mqTRCMData.triggerType,
										  &mqTRCMData.triggerTypeParameter1,
										  &mqTRCMData.triggerTypeParameter2,
										  &mqTRCMData.triggerTypeParameter3, MessageBuffer,
										  sizeof (MessageBuffer), 0);
					UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
									&(objectConnections[iIndex].commandSocket), 0);
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
						iIndex = getObjectIndexFromIP(mqEXACData.ip, objectConnections, nbr_objects);
						DataDictionaryGetObjectEnableStatusByIp(mqEXACData.ip, &objectEnabledStatus);
						if (iIndex != -1 && objectEnabledStatus == OBJECT_ENABLED) {
							struct timeval startTime;

							TimeSetToCurrentSystemTime(&currentTime);
							TimeSetToGPStime(&startTime, TimeGetAsGPSweek(&currentTime),
											 mqEXACData.executionTime_qmsoW);
							LogMessage(LOG_LEVEL_INFO, "Current time: %ld, Start time: %ld, delay: %u",
									   TimeGetAsUTCms(&currentTime), TimeGetAsUTCms(&startTime),
									   mqEXACData.executionTime_qmsoW);
							MessageLength =
								encodeSTRTMessage(&startTime, MessageBuffer, sizeof (MessageBuffer), 0);
							UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
											&objectConnections[iIndex].commandSocket, 0);
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
					iIndex = getObjectIndexFromIP(mqEXACData.ip, objectConnections, nbr_objects);
					DataDictionaryGetObjectEnableStatusById(mqEXACData.ip, &objectEnabledStatus);
					if (iIndex != -1 && objectEnabledStatus == OBJECT_ENABLED) {
						struct timeval executionTime;

						TimeSetToGPStime(&executionTime, TimeGetAsGPSweek(&currentTime),
										 mqEXACData.executionTime_qmsoW);
						MessageLength =
							encodeEXACMessage(&mqEXACData.actionID, &executionTime, MessageBuffer,
											  sizeof (MessageBuffer), 0);
						UtilSendTCPData(MODULE_NAME, MessageBuffer, MessageLength,
										&(objectConnections[iIndex].commandSocket), 0);
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
				JournalRecordData(JOURNAL_RECORD_EVENT, "CONNECT received.");

				if (connectAllObjects(objectConnections, nbr_objects) < 0) {
					LogMessage(LOG_LEVEL_INFO, "Unable to connect all objects");
					vSetState(OBC_STATE_IDLE, GSD);
				}
				else {
					JournalRecordData(JOURNAL_RECORD_STRING, "Configuring connected objects.");
					if (configureAllObjects
						(objectConnections, OriginPosition, currentTime, object_traj_file, nbr_objects) < 0) {
						LogMessage(LOG_LEVEL_INFO, "Unable to configure connected objects");
						// TODO disconnect all objects
						vSetState(OBC_STATE_IDLE, GSD);
					}
					else {
						// Set up adaptive sync points
						LogMessage(LOG_LEVEL_INFO, "Configuring adaptive synchronization points");
						if (configureAdaptiveSynchronizationPoints(object_traj_file, objectConnections,
																   OP, nbr_objects, ASP,
																   SyncPointCount) < 0) {
							LogMessage(LOG_LEVEL_ERROR, "Unable to set up adaptive synchronization points");
						}

						// Set latest monitor data receive time to now, so the timeout doesn't trigger instantly
						uiTimeCycle = 0;
						ObjectcontrolExecutionMode = OBJECT_CONTROL_CONTROL_MODE;
						objectControlServerStatus = CONTROL_CENTER_STATUS_READY;
						nextHeartbeatTime = currentTime;
						nextAdaptiveSyncMessageTime = currentTime;

						vSetState(OBC_STATE_CONNECTED, GSD);
						iCommSend(COMM_OBJECTS_CONNECTED, NULL, 0);
					}
				}
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
				LogMessage(LOG_LEVEL_INFO, "DISCONNECT received");
				JournalRecordData(JOURNAL_RECORD_EVENT, "DISCONNECT received.");
				disconnectAllObjects(objectConnections, nbr_objects);
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
 * \brief handleObjectPropertiesData Retransmits object properties data
 *			according to the configured injection maps. Assumes all
 *			objects are connected.
 * \param properties Properties of the object which are to be shared
 * \param injectionMaps Maps showing which objects' data should be
 *			retransmitted to which objects
 * \param objectConnections Object connection structs
 * \param transmitterIDs Object transmitter ID arrays
 * \param numberOfObjects Number of objects in scenario
 * \return 0 if successful, -1 otherwise
 */
int handleObjectPropertiesData(const ObjectPropertiesType * properties,
							   const DataInjectionMap injectionMaps[],
							   const ObjectConnection objectConnections[],
							   const uint32_t transmitterIDs[], const unsigned int numberOfObjects) {

	const DataInjectionMap *relevantMap = NULL;
	ForeignObjectPropertiesType propertyMessage;
	ssize_t messageSize = 0;
	char transmissionBuffer[1024];

	if (DataDictionarySetObjectProperties(properties->objectID, properties)
		!= WRITE_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to write object property data to data dictionary");
		return -1;
	}

	// Find the map for source of monitor data
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		if (injectionMaps[i].sourceID == properties->objectID && injectionMaps[i].isActive) {
			relevantMap = &injectionMaps[i];
		}
	}

	if (relevantMap == NULL) {
		return 0;
	}
	if (relevantMap->numberOfTargets == 0) {
		return 0;
	}

	// Create the message
	propertyMessage.foreignTransmitterID = properties->objectID;
	propertyMessage.mass_kg = properties->mass_kg;
	propertyMessage.actorType = properties->actorType;
	propertyMessage.objectType = properties->objectType;
	propertyMessage.operationMode = properties->operationMode;
	propertyMessage.objectXDimension_m = properties->objectXDimension_m;
	propertyMessage.objectYDimension_m = properties->objectYDimension_m;
	propertyMessage.objectZDimension_m = properties->objectZDimension_m;

	propertyMessage.isMassValid = properties->isMassValid;
	propertyMessage.isObjectXDimensionValid = properties->isObjectXDimensionValid;
	propertyMessage.isObjectYDimensionValid = properties->isObjectYDimensionValid;
	propertyMessage.isObjectZDimensionValid = properties->isObjectZDimensionValid;
	propertyMessage.isObjectXDisplacementValid = properties->isObjectXDisplacementValid;
	propertyMessage.isObjectYDisplacementValid = properties->isObjectYDisplacementValid;
	propertyMessage.isObjectZDisplacementValid = properties->isObjectZDisplacementValid;

	messageSize = encodeFOPRMessage(&propertyMessage, transmissionBuffer, sizeof (transmissionBuffer), 0);
	if (messageSize == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to encode FOPR message");
		return -1;
	}


	// Send message to all configured receivers
	for (unsigned int i = 0; i < relevantMap->numberOfTargets; ++i) {
		for (unsigned int j = 0; j < numberOfObjects; j++) {
			if (transmitterIDs[j] == relevantMap->targetIDs[i]) {
				LogMessage(LOG_LEVEL_INFO, "Configuring object %u to receive injection data from object %u",
						   relevantMap->targetIDs[i], relevantMap->sourceID);
				UtilSendTCPData(MODULE_NAME, transmissionBuffer, messageSize,
								&objectConnections[j].commandSocket, 0);
			}
		}
	}
	return 0;
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


int getObjectIndexFromIP(const in_addr_t ipAddr,
						 const ObjectConnection objectConnections[], unsigned int numberOfObjects) {
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		if (objectConnections[i].objectCommandAddress.sin_addr.s_addr == ipAddr)
			return (int)i;
	}
	return -1;
}

void *objectConnectorThread(void *inputArgs) {
	ObjectConnection *args = (ObjectConnection *) inputArgs;
	int retval;
	char ipString[INET_ADDRSTRLEN];

	inet_ntop(AF_INET, &args->objectCommandAddress.sin_addr, ipString, sizeof (ipString));

	if (args->objectCommandAddress.sin_addr.s_addr == 0) {
		LogMessage(LOG_LEVEL_ERROR, "Invalid connection IP specified: %s", ipString);
		pthread_exit((void *)-1);
	}

	args->commandSocket = socket(AF_INET, SOCK_STREAM, 0);

	if (args->commandSocket < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to open control socket");
		args->monitorSocket = -1;
		pthread_exit((void *)-1);
	}

	// Begin connection attempt
	struct timeval currentTime, timeoutTime;
	struct timespec retryPeriod;

	retryPeriod.tv_sec = OBJECT_CONNECTION_RETRY_PERIOD_MS / 1000;
	retryPeriod.tv_nsec = (OBJECT_CONNECTION_RETRY_PERIOD_MS * 1000000) % 1000000000;

	LogMessage(LOG_LEVEL_INFO, "Attempting to connect to socket: %s:%u", ipString,
			   args->objectCommandAddress.sin_port);
	TimeSetToCurrentSystemTime(&timeoutTime);
	timeoutTime.tv_sec += OBJECT_CONNECTION_TIMEOUT_S;
	do {
		retval =
			connect(args->commandSocket, (struct sockaddr *)&args->objectCommandAddress,
					sizeof (args->objectCommandAddress));
		if (retval == -1 /* && errno == ECONNREFUSED */ ) {	// Retry for all errors for now
			LogMessage(LOG_LEVEL_ERROR, "Failed connection attempt to %s, retrying in %.3f s ...",
					   ipString, (double)retryPeriod.tv_sec + (double)retryPeriod.tv_nsec / 1000000000.0);
			nanosleep(&retryPeriod, NULL);
		}
		TimeSetToCurrentSystemTime(&currentTime);
	} while (timercmp(&currentTime, &timeoutTime, <) && retval != 0);

	if (retval == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Failed connection to socket: %s:%u", ipString,
				   args->objectCommandAddress.sin_port);
		close(args->commandSocket);
		args->commandSocket = -1;
		args->monitorSocket = -1;
		pthread_exit((void *)-1);
	}
	LogMessage(LOG_LEVEL_INFO, "Connected to socket: %s:%u", ipString, args->objectCommandAddress.sin_port);

	// Set command socket to nonblocking
	retval = fcntl(args->commandSocket, F_SETFL, fcntl(args->commandSocket, F_GETFL, 0) | O_NONBLOCK);
	if (retval == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Error calling fcntl");
		close(args->commandSocket);
		args->commandSocket = -1;
		args->monitorSocket = -1;
		pthread_exit((void *)-1);
	}

	// Create monitor socket
	LogMessage(LOG_LEVEL_DEBUG, "Creating safety socket");
	args->monitorSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (args->monitorSocket < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to create monitor socket");
		close(args->commandSocket);
		args->commandSocket = -1;
		args->monitorSocket = -1;
		pthread_exit((void *)-1);
	}

	retval = fcntl(args->monitorSocket, F_SETFL, fcntl(args->monitorSocket, F_GETFL, 0) | O_NONBLOCK);
	if (retval < 0) {
		LogMessage(LOG_LEVEL_ERROR, "Error calling fcntl");
		close(args->commandSocket);
		close(args->monitorSocket);
		args->commandSocket = -1;
		args->monitorSocket = -1;
		pthread_exit((void *)-1);
	}
	else {
		LogMessage(LOG_LEVEL_INFO, "Created monitor socket: %s:%u", ipString,
				   args->objectMonitorAddress.sin_port);
	}
	pthread_exit((void *)0);
}

int connectAllObjects(ObjectConnection objectConnections[], const unsigned int numberOfObjects) {

	uint32_t *transmitterIDs = (uint32_t *) malloc(sizeof (uint32_t) * numberOfObjects);
	pthread_t *connectors = (pthread_t *) malloc(sizeof (pthread_t) * numberOfObjects);

	ObjectEnabledType enabledStatus;
	int retval;

	DataDictionaryGetObjectTransmitterIDs(transmitterIDs, numberOfObjects);

	LogMessage(LOG_LEVEL_DEBUG, "Connecting to %u objects", numberOfObjects);
	for (unsigned int i = 0; i < numberOfObjects; ++i) {

		DataDictionaryGetObjectEnableStatusById(transmitterIDs[i], &enabledStatus);

		if (enabledStatus == OBJECT_ENABLED) {
			retval = pthread_create(&connectors[i], NULL, objectConnectorThread, objectConnections + i);
			if (retval == -1) {
				LogMessage(LOG_LEVEL_ERROR, "Failed to start connection thread");
			}
		}
	}

	// Await all threads to finish successfully
	retval = 0;
	while (!iExit) {
		char mqBuffer[MBUS_MAX_DATALEN];
		enum COMMAND command;

		// Check if any command arrived
		iCommRecv(&command, mqBuffer, sizeof (mqBuffer), NULL);
		switch (command) {
		case COMM_EXIT:
		case COMM_ABORT:
		case COMM_DISCONNECT:
			for (unsigned int i = 0; i < numberOfObjects; ++i) {
				pthread_cancel(connectors[i]);
			}
			retval = -1;
			break;
		case COMM_GETSTATUS:
			iCommSend(COMM_GETSTATUS_OK, MODULE_NAME, sizeof (MODULE_NAME));
			break;
		}

		int allConnectionsCompleted = 1;

		for (unsigned int i = 0; i < numberOfObjects; ++i) {
			allConnectionsCompleted = allConnectionsCompleted
				&& objectConnections[i].commandSocket != 0 && objectConnections[i].monitorSocket != 0;
		}
		if (allConnectionsCompleted || retval == -1) {
			break;
		}
	}

	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		long threadReturn;

		if (pthread_join(connectors[i], (void **)&threadReturn)) {
			LogMessage(LOG_LEVEL_ERROR, "Error joining connector thread");
			retval = -1;
		}
		else {
			retval = threadReturn == -1 ? -1 : retval;
		}
	}

	// If one connection failed, close all connections
	if (retval == -1) {
		disconnectAllObjects(objectConnections, numberOfObjects);
	}

	free(transmitterIDs);
	free(connectors);
	return retval;
}

void disconnectAllObjects(ObjectConnection objectConnections[], const unsigned int numberOfObjects) {
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		disconnectObject(&objectConnections[i]);
	}
}

int configureAllObjects(ObjectConnection objectConnections[],
						const GeoPosition originPosition,
						const struct timeval currentTime,
						const char trajectoryFiles[MAX_OBJECTS][MAX_FILE_PATH],
						const unsigned int numberOfObjects) {

	ObjectEnabledType enabledStatus;
	uint32_t *transmitterIDs = (uint32_t *) malloc(sizeof (uint32_t) * numberOfObjects);
	ssize_t messageLength;
	char messageBuffer[1024];
	char ipString[INET_ADDRSTRLEN];
	uint32_t serverTransmitterID;
	uint8_t isoTransmitterID;
	int retval = 0;

	DataDictionaryGetObjectTransmitterIDs(transmitterIDs, numberOfObjects);
	// Send configuration
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		DataDictionaryGetObjectEnableStatusById(transmitterIDs[i], &enabledStatus);
		inet_ntop(objectConnections[i].objectCommandAddress.sin_family,
				  &objectConnections[i].objectCommandAddress.sin_addr, ipString, sizeof (ipString));
		if (enabledStatus == OBJECT_ENABLED) {


			float altitude = (float)originPosition.Altitude;

			messageLength =
				encodeOSEMMessage(&currentTime, &transmitterIDs[i], &originPosition.Latitude,
								  &originPosition.Longitude, &altitude, NULL, NULL, NULL,
								  messageBuffer, sizeof (messageBuffer), 0);
			if (messageLength < 0) {
				LogMessage(LOG_LEVEL_ERROR, "OSEM encoding error");
				retval = -1;
				break;
			}

			LogMessage(LOG_LEVEL_INFO, "Sending OSEM to object ID %u with IP %s", transmitterIDs[i],
					   ipString);

			DataDictionaryGetTransmitterID(&serverTransmitterID);
			isoTransmitterID = (uint8_t) serverTransmitterID;
			setTransmitterID(isoTransmitterID);

			UtilSendTCPData(MODULE_NAME, messageBuffer, messageLength, &objectConnections[i].commandSocket,
							0);

			if (1 /* TODO Check that object is not among DTM receivers */ ) {
				LogMessage(LOG_LEVEL_INFO, "Sending TRAJ to %s", ipString);
				if (ObjectControlSendTRAJMessage(trajectoryFiles[i], &objectConnections[i].commandSocket, 0)
					== -1) {
					LogMessage(LOG_LEVEL_ERROR, "Failed to send TRAJ to %s", ipString);
					retval = -1;
					break;
				}
			}
			DataDictionarySetMonitorDataReceiveTime(transmitterIDs[i], &currentTime);
		}
	}
	free(transmitterIDs);
	return retval;
}


int checkObjectConnections(ObjectConnection objectConnections[], const struct timeval monitorTimeout,
						   const unsigned int numberOfObjects) {

	ObjectEnabledType enabledStatus;
	uint32_t *transmitterIDs = malloc(sizeof (uint32_t) * numberOfObjects);
	uint8_t disconnected = 0;
	struct timeval lastDataUpdate, timeSinceLastMonitorData, currentTime;
	char ipString[INET_ADDRSTRLEN];

	TimeSetToCurrentSystemTime(&currentTime);

	if (DataDictionaryGetObjectTransmitterIDs(transmitterIDs, numberOfObjects) != READ_OK) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to read object transmitter IDs from shared memory");
		free(transmitterIDs);
		return -1;
	}

	for (unsigned int i = 0; i < numberOfObjects; ++i) {

		if (DataDictionaryGetObjectEnableStatusById(transmitterIDs[i], &enabledStatus) != READ_OK) {
			LogMessage(LOG_LEVEL_ERROR, "Failed to read enabled status from shared memory");
			free(transmitterIDs);
			return -1;
		}

		if (transmitterIDs[i] == 0 || enabledStatus != OBJECT_ENABLED) {
			continue;
		}

		// Check broken TCP connection
		disconnected |= hasRemoteDisconnected(&objectConnections[i].commandSocket);

		// Check time since last position update
		DataDictionaryGetMonitorDataReceiveTime(transmitterIDs[i], &lastDataUpdate);

		timersub(&currentTime, &lastDataUpdate, &timeSinceLastMonitorData);
		if (timercmp(&timeSinceLastMonitorData, &monitorTimeout, >)) {
			LogMessage(LOG_LEVEL_WARNING,
					   "MONR timeout (ID %u):\n\ttimeout time %d s %d µs\n\tlast message %d s %d µs\n\tcurrent time %d s %d µs",
					   transmitterIDs[i], monitorTimeout.tv_sec,
					   monitorTimeout.tv_usec, lastDataUpdate.tv_sec,
					   lastDataUpdate.tv_usec, currentTime.tv_sec, currentTime.tv_usec);
			disconnected = 1;
		}

		if (disconnected) {
			inet_ntop(objectConnections[i].objectCommandAddress.sin_family,
					  &objectConnections[i].objectCommandAddress.sin_addr, ipString, sizeof (ipString));
			LogMessage(LOG_LEVEL_WARNING, "Lost connection to IP %s", ipString);
		}
	}
	free(transmitterIDs);
	return disconnected;
}

void disconnectObject(ObjectConnection * objectConnection) {
	if (objectConnection->commandSocket > 0) {
		close(objectConnection->commandSocket);
		objectConnection->commandSocket = 0;
	}
	if (objectConnection->monitorSocket > 0) {
		close(objectConnection->monitorSocket);
		objectConnection->monitorSocket = 0;
	}
}

int hasRemoteDisconnected(int *sockfd) {
	char dummy;

	if (*sockfd == 0) {
		return 1;
	}

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
		return 0;
	}

	return 1;
}

int configureAdaptiveSynchronizationPoints(const char trajectoryFiles[MAX_OBJECTS][MAX_FILE_PATH],
										   const ObjectConnection objectConnections[],
										   ObjectPosition objectPositions[],
										   const unsigned int numberOfObjects,
										   const AdaptiveSyncPoint ASP[],
										   const unsigned int adaptiveSyncPointCount) {

	char ipString[INET_ADDRSTRLEN];
	char messageBuffer[1024];
	ssize_t messageLength = 0;
	unsigned int rowCount;

	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		inet_ntop(objectConnections[i].objectCommandAddress.sin_family,
				  &objectConnections[i].objectCommandAddress.sin_addr.s_addr, ipString, sizeof (ipString));

		rowCount = 0;
		FILE *trajectory_fd = fopen(trajectoryFiles[i], "r");

		if (trajectory_fd) {
			rowCount = UtilCountFileRows(trajectory_fd) - 2;
			fclose(trajectory_fd);
		}

		if (objectPositions[i].SpaceArr != NULL) {
			free(objectPositions[i].SpaceArr);
			objectPositions[i].SpaceArr = NULL;
		}
		if (objectPositions[i].TimeArr != NULL) {
			free(objectPositions[i].TimeArr);
			objectPositions[i].TimeArr = NULL;
		}
		if (objectPositions[i].SpaceTimeArr != NULL) {
			free(objectPositions[i].SpaceTimeArr);
			objectPositions[i].SpaceTimeArr = NULL;
		}

		UtilSetObjectPositionIP(&objectPositions[i], ipString);
		objectPositions[i].TrajectoryPositionCount = rowCount;
		if (rowCount > 0) {
			objectPositions[i].SpaceArr = malloc(sizeof (*objectPositions[i].SpaceArr) * rowCount);
			objectPositions[i].TimeArr = malloc(sizeof (*objectPositions[i].TimeArr) * rowCount);
			objectPositions[i].SpaceTimeArr = malloc(sizeof (*objectPositions[i].SpaceTimeArr) * rowCount);

			UtilPopulateSpaceTimeArr(&objectPositions[i], trajectoryFiles[i]);

		}

		LogMessage(LOG_LEVEL_INFO, "Sync point counts: %d", adaptiveSyncPointCount);
		for (unsigned int j = 0; j < adaptiveSyncPointCount; j++) {
			struct timeval syncPointTime, syncStopTime;

			TimeSetToUTCms(&syncPointTime, (int64_t) (ASP[i].SlaveTrajSyncTime * 1000.0f));
			TimeSetToUTCms(&syncStopTime, (int64_t) (ASP[i].SlaveSyncStopTime * 1000.0f));
			if (TEST_SYNC_POINTS == 1 && i == 1) {
				/*Send SYPM to slave */
				messageLength =
					encodeSYPMMessage(syncPointTime, syncStopTime, messageBuffer, sizeof (messageBuffer), 0);
				UtilSendTCPData(MODULE_NAME, messageBuffer, messageLength,
								&objectConnections[i].commandSocket, 0);
			}
			else if (TEST_SYNC_POINTS == 0 && strstr(ipString, ASP[i].SlaveIP) != NULL) {
				/*Send SYPM to slave */
				messageLength =
					encodeSYPMMessage(syncPointTime, syncStopTime, messageBuffer, sizeof (messageBuffer), 0);
				UtilSendTCPData(MODULE_NAME, messageBuffer, messageLength,
								&objectConnections[i].commandSocket, 0);
			}
		}

		/*Set Sync point in OP */
		for (unsigned int j = 0; j < adaptiveSyncPointCount; j++) {
			if (TEST_SYNC_POINTS == 1 && i == 0)
				UtilSetSyncPoint(&objectPositions[i], 0, 0, 0, ASP[i].MasterTrajSyncTime);
			else if (TEST_SYNC_POINTS == 0 && strstr(ipString, ASP[i].MasterIP) != NULL)
				UtilSetSyncPoint(&objectPositions[i], 0, 0, 0, ASP[i].MasterTrajSyncTime);
		}
	}
}

ssize_t uiRecvMonitor(int *sockfd, char *buffer, size_t length) {
	ssize_t result = 0;
	ssize_t recvDataSize = 0;

	// Read until receive buffer is empty, return last read message
	do {
		result = recv(*sockfd, buffer, length, 0);

		if (result < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				LogMessage(LOG_LEVEL_ERROR, "Failed to receive from monitor socket");
				return -1;
			}
		}
		else {
			recvDataSize = result;
			LogMessage(LOG_LEVEL_DEBUG, "Received: <%s>", buffer);
		}
	} while (result > 0);

	return recvDataSize;
}

int iFindObjectsInfo(C8 object_traj_file[MAX_OBJECTS][MAX_FILE_PATH],
					 ObjectConnection objectConnections[],
					 uint32_t objectIDs[MAX_OBJECTS], unsigned int *nbr_objects) {
	DIR *traj_directory;
	DIR *object_directory;
	struct dirent *directory_entry;
	int result;
	char trajPathDir[MAX_FILE_PATH];
	char objectPathDir[MAX_FILE_PATH];
	char objectFilePath[MAX_FILE_PATH];
	int retval = 0;

	*nbr_objects = 0;

	UtilGetTrajDirectoryPath(trajPathDir, sizeof (trajPathDir));
	UtilGetObjectDirectoryPath(objectPathDir, sizeof (objectPathDir));
	object_directory = opendir(objectPathDir);

	traj_directory = opendir(trajPathDir);
	if (traj_directory == NULL) {
		util_error("Failed to open trajectory directory");
	}
	else if (object_directory == NULL) {
		util_error("Failed to open object directory");
	}

	// Check all entries in the object directory
	while ((directory_entry = readdir(object_directory)) && ((*nbr_objects) < MAX_OBJECTS)) {
		if (!strncmp(directory_entry->d_name, ".", 1)) {
			continue;
		}
		char objectSetting[100];

		strcpy(objectFilePath, objectPathDir);
		strcat(objectFilePath, directory_entry->d_name);
		memset(object_traj_file[*nbr_objects], 0, MAX_FILE_PATH);

		// Get IP setting
		if (UtilGetObjectFileSetting(OBJECT_SETTING_IP, objectFilePath,
									 sizeof (objectFilePath), objectSetting, sizeof (objectSetting)) == -1) {
			errno = EINVAL;
			LogMessage(LOG_LEVEL_ERROR, "Cannot find IP setting in file <%s>", objectFilePath);
			retval = -1;
			continue;
		}
		else {
			result = inet_pton(AF_INET, objectSetting,
							   &objectConnections[*nbr_objects].objectCommandAddress.sin_addr);
			if (result == -1) {
				LogMessage(LOG_LEVEL_ERROR, "Invalid address family");
				retval = -1;
				continue;
			}
			else if (result == 0) {
				errno = EINVAL;
				LogMessage(LOG_LEVEL_ERROR, "Address %s in object file %s is not a valid IPv4 address",
						   objectSetting, objectFilePath);
				retval = -1;
				continue;
			}
			else {
				objectConnections[*nbr_objects].objectCommandAddress.sin_family = AF_INET;
				objectConnections[*nbr_objects].objectMonitorAddress =
					objectConnections[*nbr_objects].objectCommandAddress;
			}
		}

		// Get trajectory file setting
		if (UtilGetObjectFileSetting(OBJECT_SETTING_TRAJ, objectFilePath, sizeof (objectFilePath),
									 objectSetting, sizeof (objectSetting)) == -1) {
			LogMessage(LOG_LEVEL_ERROR, "Cannot find trajectory setting in file <%s>", objectFilePath);
			retval = -1;
			continue;
		}
		else {
			strcpy(object_traj_file[*nbr_objects], trajPathDir);
			strcat(object_traj_file[*nbr_objects], objectSetting);

			if (UtilCheckTrajectoryFileFormat
				(object_traj_file[*nbr_objects], sizeof (object_traj_file[*nbr_objects]))) {
				errno = EINVAL;
				LogMessage(LOG_LEVEL_ERROR, "Trajectory file <%s> is not valid",
						   object_traj_file[*nbr_objects]);
				retval = -1;
				continue;
			}
		}

		if (UtilGetObjectFileSetting(OBJECT_SETTING_ID, objectFilePath, sizeof (objectFilePath),
									 objectSetting, sizeof (objectSetting)) == -1) {
			errno = EINVAL;
			LogMessage(LOG_LEVEL_ERROR, "Cannot find ID setting in file <%s>", objectFilePath);
			retval = -1;
			continue;
		}
		else {
			char *endptr;
			uint32_t id = (uint32_t) strtoul(objectSetting, &endptr, 10);

			if (endptr == objectSetting) {
				errno = EINVAL;
				LogMessage(LOG_LEVEL_ERROR, "ID <%s> in file <%s> is invalid", objectSetting, objectFilePath);
				retval = -1;
				continue;
			}
			else {
				objectIDs[*nbr_objects] = id;
			}
		}

		LogMessage(LOG_LEVEL_INFO, "Loaded object with ID %u, IP %s and trajectory file <%s>",
				   objectIDs[*nbr_objects], inet_ntop(AF_INET,
													  &objectConnections[*nbr_objects].
													  objectCommandAddress.sin_addr, objectSetting,
													  sizeof (objectSetting)),
				   object_traj_file[*nbr_objects]);
		++(*nbr_objects);
	}

	(void)closedir(object_directory);
	(void)closedir(traj_directory);

	// Check so there are no duplicates
	for (unsigned int i = 0; i < *nbr_objects; ++i) {
		for (unsigned int j = i + 1; j < *nbr_objects; ++j) {
			if (objectIDs[i] == objectIDs[j]) {
				errno = EINVAL;
				LogMessage(LOG_LEVEL_ERROR, "Found two objects with configured ID %u", objectIDs[i]);
				retval = -1;
			}
		}
	}

	return retval;
}

/*!
 * \brief configureObjectDataInjection Parses all object files to fill an array of maps between
 *			objects according to the injectorIDs settings found in the files.
 * \param injectionMaps Array of maps to be filled. The pointers contained in its elements are
 *			assumed to either be NULL or returned from a previous call to this function. The
 *			array is assumed to have the same number of elements as transmitterIDs.
 * \param transmitterIDs Configured scenario transmitter IDs.
 * \param numberOfObjects Number of elements in the arrays.
 * \return 0 if successful, -1 otherwise.
 */
int configureObjectDataInjection(DataInjectionMap injectionMaps[],
								 const uint32_t transmitterIDs[], const unsigned int numberOfObjects) {

	char objectDirPath[MAX_FILE_PATH];
	char objectFilePath[MAX_FILE_PATH];
	DIR *objectDirectory;
	struct dirent *dirEntry;
	int retval = 0;

	if (injectionMaps == NULL || transmitterIDs == NULL) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Data injection configuration input pointer error");
		return -1;
	}

	// Reset maps
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		injectionMaps[i].sourceID = transmitterIDs[i];
		free(injectionMaps[i].targetIDs);
		injectionMaps[i].targetIDs = NULL;
		injectionMaps[i].numberOfTargets = 0;
		injectionMaps[i].isActive = 1;
	}

	UtilGetObjectDirectoryPath(objectDirPath, sizeof (objectDirPath));
	objectDirectory = opendir(objectDirPath);
	if (objectDirectory == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to open object directory");
		return -1;
	}

	while ((dirEntry = readdir(objectDirectory)) != NULL) {
		if (!strncmp(dirEntry->d_name, ".", 1)) {
			continue;
		}
		strcpy(objectFilePath, objectDirPath);
		strcat(objectFilePath, dirEntry->d_name);
		if (parseDataInjectionSetting(objectFilePath, injectionMaps, numberOfObjects) == -1) {
			retval = -1;
			LogMessage(LOG_LEVEL_ERROR, "Failed to parse injection settings of file %s", objectFilePath);
		}
	}

	return retval;
}

/*!
 * \brief parseDataInjectionSetting Parses the injectorIDs setting of a single object file and inputs the data into
 *			the corresponding injection maps.
 * \param objectFilePath File path of the object file to be parsed.
 * \param injectionMaps Array of maps between objects. The pointers contained in its elements are
 *			assumed to either be NULL or returned from a previous memory allocation call.
 * \param numberOfMaps Number of elements in injection map array.
 * \return 0 if successful, -1 otherwise.
 */
int parseDataInjectionSetting(const char objectFilePath[MAX_FILE_PATH],
							  DataInjectionMap injectionMaps[], const unsigned int numberOfMaps) {

	char objectSetting[100];
	char *token = NULL, *endptr = NULL;
	const char delimiter[] = ",";
	int retval = 0;
	uint32_t sourceID = 0, targetID = 0;

	if (UtilGetObjectFileSetting(OBJECT_SETTING_ID,
								 objectFilePath, MAX_FILE_PATH,
								 objectSetting, sizeof (objectSetting)) == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Object ID missing from file <%s>", objectFilePath);
		return -1;
	}

	targetID = (uint32_t) strtoul(objectSetting, &endptr, 10);
	if (endptr == objectSetting) {
		errno = EINVAL;
		LogMessage(LOG_LEVEL_ERROR, "Invalid ID setting <%s> in file %s", objectSetting, objectFilePath);
		return -1;
	}

	if (UtilGetObjectFileSetting(OBJECT_SETTING_INJECTOR_IDS,
								 objectFilePath, MAX_FILE_PATH,
								 objectSetting, sizeof (objectSetting)) == -1) {
		return 0;				// No setting found
	}

	token = strtok(objectSetting, delimiter);
	if (token == NULL) {
		return 0;				// Empty setting found
	}

	do {
		sourceID = (uint32_t) strtoul(token, &endptr, 10);
		if (endptr == token) {
			errno = EINVAL;
			LogMessage(LOG_LEVEL_ERROR, "Unparsable injector ID setting <%s>", token);
			retval = -1;
		}
		else {
			// Find the map matching source ID in configuration
			int found = false;

			for (unsigned int i = 0; i < numberOfMaps; ++i) {
				if (injectionMaps[i].sourceID == sourceID) {
					found = true;
					// Append object ID of open file to targets of ID in configuration
					injectionMaps[i].targetIDs =
						realloc(injectionMaps[i].targetIDs,
								++injectionMaps[i].numberOfTargets * sizeof (uint32_t));
					if (injectionMaps[i].targetIDs == NULL) {
						LogMessage(LOG_LEVEL_ERROR, "Memory allocation error");
						return -1;
					}
					injectionMaps[i].targetIDs[injectionMaps[i].numberOfTargets - 1] = targetID;
				}
			}
			if (!found) {
				LogMessage(LOG_LEVEL_ERROR,
						   "Data injection source object with ID %u not among configured transmitter IDs",
						   sourceID, objectFilePath);
				retval = -1;
			}
		}
	} while ((token = strtok(NULL, delimiter)) != NULL);

	return retval;
}

/*!
 * \brief sendDataInjectionMessages Translates the passed monitor data to an injection message
 *			and sends it to all configured receivers.
 * \param objectData Monitor data to be passed on.
 * \param dataInjectionMaps Map showing which objects are the receivers of injection data.
 * \param objectConnections Connections to objects. Assumed to be ordered in the same way
 *			as the transmitter ID array.
 * \param transmitterIDs Transmitter IDs of objects.
 * \param numberOfObjects Size of all three arrays.
 * \return 0 if successful, -1 otherwise.
 */
int sendDataInjectionMessages(const ObjectDataType * objectData,
							  const DataInjectionMap dataInjectionMaps[],
							  const ObjectConnection objectConnections[],
							  const uint32_t transmitterIDs[], const unsigned int numberOfObjects) {

	if (objectData == NULL || dataInjectionMaps == NULL || objectConnections == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Attempted to pass null pointer to %s", __FUNCTION__);
		return -1;
	}

	const ObjectMonitorType *monitorData = &objectData->MonrData;
	char transmissionBuffer[1024];
	ssize_t messageSize;
	const DataInjectionMap *relevantMap = NULL;
	PeerObjectInjectionType injectionMessage;

	// Find the map for source of monitor data
	for (unsigned int i = 0; i < numberOfObjects; ++i) {
		if (dataInjectionMaps[i].sourceID == objectData->ClientID) {
			relevantMap = &dataInjectionMaps[i];
		}
	}
	if (relevantMap == NULL) {
		LogMessage(LOG_LEVEL_ERROR, "Found no injection settings for sender ID %u", objectData->ClientID);
		return -1;
	}
	if (relevantMap->numberOfTargets == 0) {
		return 0;
	}

	// Create the message
	injectionMessage.dataTimestamp = monitorData->timestamp;
	injectionMessage.position = monitorData->position;
	injectionMessage.state = monitorData->state;
	injectionMessage.speed = monitorData->speed;
	injectionMessage.foreignTransmitterID = objectData->ClientID;
	injectionMessage.isRollValid = 0;
	injectionMessage.isPitchValid = 0;

	messageSize = encodePODIMessage(&injectionMessage, transmissionBuffer, sizeof (transmissionBuffer), 0);
	if (messageSize == -1) {
		LogMessage(LOG_LEVEL_ERROR, "Failed to encode PODI message");
		return -1;
	}


	// Send message to all configured receivers
	for (unsigned int i = 0; i < relevantMap->numberOfTargets; ++i) {
		for (unsigned int j = 0; j < numberOfObjects; j++) {
			if (transmitterIDs[j] == relevantMap->targetIDs[i]) {
				UtilSendUDPData(MODULE_NAME, &objectConnections[j].monitorSocket,
								&objectConnections[j].objectMonitorAddress,
								transmissionBuffer, messageSize, 0);
			}
		}
	}
	return 0;
}

/*!
 * \brief readMonitorDataTimeout Reads the maximum allowed missing monitor
 *			messages from settings and calculates a timeout period.
 * \param timeout Time since last monitor message which should have
 *			elapsed before abort should occur
 * \return 0 on success, -1 otherwise
 */
int readMonitorDataTimeoutSetting(struct timeval *timeout) {
	uint8_t maxMissingMonitorMessages = 0;

	const struct timeval monitorDataPeriod = { 1 / MONR_EXPECTED_FREQUENCY_HZ,
		(1000000 / MONR_EXPECTED_FREQUENCY_HZ) % 1000000
	};
	*timeout = monitorDataPeriod;

	DataDictionaryGetMaxPacketsLost(&maxMissingMonitorMessages);

	LogMessage(LOG_LEVEL_INFO, "Read max allowed missing monitor packets: %u", maxMissingMonitorMessages);
	for (uint8_t i = 0; i < maxMissingMonitorMessages; ++i) {
		timeradd(timeout, &monitorDataPeriod, timeout);
	}

	return 0;
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
