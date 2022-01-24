#include "module.hpp"

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <sys/sysinfo.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <netdb.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <map>
//#include "systemcontrol.h"
#include "maestroTime.h"
#include "datadictionary.h"
#include "util.h"
#include "logging.h"

#define MODULE_NAME "SystemControl"

#define SYSTEM_CONTROL_SERVICE_POLL_TIME_MS 5000
#define SYSTEM_CONTROL_TASK_PERIOD_MS 1
#define SYSTEM_CONTROL_RVSS_TIME_MS 10

#define SYSTEM_CONTROL_GETSTATUS_TIME_MS 5000
#define SYSTEM_CONTROL_GETSTATUS_TIMEOUT_MS 2000
#define SYSTEM_CONTROL_NO_OF_MODULES_IN_USE 2	//TODO Create a file containing a list of which modules should be used. Check with this list to see if each module has responded.

#define SYSTEM_CONTROL_CONTROL_PORT   54241	// Default port, control channel
#define SYSTEM_CONTROL_PROCESS_PORT   54242	// Default port, process channel
#define SYSTEM_CONTROL_RX_PACKET_SIZE 1280
#define SYSTEM_CONTROL_TX_PACKET_SIZE SYSTEM_CONTROL_RX_PACKET_SIZE
#define IPC_BUFFER_SIZE SYSTEM_CONTROL_RX_PACKET_SIZE
//#define IPC_BUFFER_SIZE   1024
#define SYSTEM_CONTROL_CONTROL_RESPONSE_SIZE 64
#define SYSTEM_CONTROL_RVSS_DATA_BUFFER	128

#define SYSTEM_CONTROL_ARG_CHAR_COUNT 		2
#define SYSTEM_CONTROL_COMMAND_MAX_LENGTH 	32
#define SYSTEM_CONTROL_ARG_MAX_COUNT	 	6
#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH	32
//#define SYSTEM_CONTROL_TOTAL_COMMAND_MAX_LENGTH SYSTEM_CONTROL_ARG_CHAR_COUNT + SYSTEM_CONTROL_COMMAND_MAX_LENGTH + SYSTEM_CONTROL_ARG_MAX_COUNT*SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH
//#define SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH    80
#define TCP_RECV_BUFFER_SIZE 2048

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

#define RVSS_TIME_CHANNEL 1
#define RVSS_MONITOR_CHANNEL 2
#define RVSS_MAESTRO_CHANNEL 4
#define RVSS_ASP_CHANNEL 8

#define ENABLE_COMMAND_STRING "ENABLE"
#define DISABLE_COMMAND_STRING "DISABLE"

// Time intervals for sleeping when no message bus message was received and for when one was received
#define SC_SLEEP_TIME_EMPTY_MQ_S 0
#define SC_SLEEP_TIME_EMPTY_MQ_NS 10000000
#define SC_SLEEP_TIME_NONEMPTY_MQ_S 0
#define SC_SLEEP_TIME_NONEMPTY_MQ_NS 0

#define MAESTRO_GENERIC_FILE_TYPE     1
#define MAESTRO_TRAJ_FILE_TYPE        2
#define MAESTRO_CONF_FILE_TYPE        3
#define MAESTRO_GEOFENCE_FILE_TYPE    4
#define MAESTRO_OBJECT_FILE_TYPE	  5
#define MSCP_RESPONSE_DATALENGTH_BYTES 4
#define MSCP_RESPONSE_STATUS_CODE_BYTES 2

#define MAESTRO_TRAJ_DIRECTORY_STRING "traj/"

using std_msgs::Empty;
using std_msgs::String;

class SystemControlModule : public Module
{
	public:
	SystemControlModule(std::string);
	static volatile int iExit;
		typedef enum {
	SERVER_STATE_UNDEFINED,
	SERVER_STATE_INITIALIZED,
	SERVER_STATE_IDLE,
	SERVER_STATE_READY,
	SERVER_STATE_RUNNING,
	SERVER_STATE_INWORK,
	SERVER_STATE_ERROR,
	} ServerState_t;
	ServerState_t SystemControlState = SERVER_STATE_UNDEFINED;
	typedef enum {
		Idle_0, GetServerStatus_0, ArmScenario_0, DisarmScenario_0, StartScenario_1, stop_0, AbortScenario_0,
		InitializeScenario_0, ConnectObject_0, DisconnectObject_0, GetServerParameterList_0,
		SetServerParameter_2, GetServerParameter_1, DownloadFile_1, UploadFile_4, CheckFileDirectoryExist_1,
		GetRootDirectoryContent_0, GetDirectoryContent_1, DeleteTrajectory_1, DeleteGeofence_1,
		DeleteFileDirectory_1,
		ClearTrajectories_0, ClearGeofences_0, ClearObjects_0, RemoteControl_1, RemoteControlManoeuvre_2,
		SetObjectEnableStatus_2,
		GetObjectEnableStatus_1,
		CreateDirectory_1, GetTestOrigin_0, replay_1, control_0, Exit_0,
		start_ext_trigg_1, ClearAllScenario_0 , DownloadDirectoryContent_1, DownloadTrajFiles_0, nocommand
	} SystemControlCommand_t;
	SystemControlCommand_t SystemControlCommand = Idle_0;
	SystemControlCommand_t PreviousSystemControlCommand = Idle_0;

	private:

	/* definitions and datatypes */
	const char *SystemControlStatesArr[7] =
		{ "UNDEFINED", "INITIALIZED", "IDLE", "READY", "RUNNING", "INWORK", "ERROR" };
	const char *SystemControlOBCStatesArr[8] =
		{ "UNDEFINED", "IDLE", "INITIALIZED", "CONNECTED", "ARMED", "RUNNING", "REMOTECONTROL", "ERROR" };
	const char *POSTRequestMandatoryContent[3] = { "POST", "HTTP/1.1", "\r\n\r\n" };
	const char *SystemControlCommandsArr[37] = {
		"Idle_0", "GetServerStatus_0", "ArmScenario_0", "DisarmScenario_0", "StartScenario_1", "stop_0",
		"AbortScenario_0", "InitializeScenario_0",
		"ConnectObject_0", "DisconnectObject_0", "GetServerParameterList_0", "SetServerParameter_2",
		"GetServerParameter_1", "DownloadFile_1", "UploadFile_4", "CheckFileDirectoryExist_1",
		"GetRootDirectoryContent_0", "GetDirectoryContent_1", "DeleteTrajectory_1", "DeleteGeofence_1",
		"DeleteFileDirectory_1",
		"ClearTrajectories_0", "ClearGeofences_0", "ClearObjects_0", "RemoteControl_1",
		"RemoteControlManoeuvre_2",
		"SetObjectEnableStatus_2",
		"GetObjectEnableStatus_1", "CreateDirectory_1", "GetTestOrigin_0", "replay_1",
		"control_0",
		"Exit_0", "start_ext_trigg_1", "ClearAllScenario_0", "DownloadDirectoryContent_1", "DownloadTrajFiles_0"
	};
	char SystemControlCommandArgCnt[SYSTEM_CONTROL_ARG_CHAR_COUNT];
	char SystemControlStrippedCommand[SYSTEM_CONTROL_COMMAND_MAX_LENGTH];
	char SystemControlArgument[SYSTEM_CONTROL_ARG_MAX_COUNT][SYSTEM_CONTROL_ARGUMENT_MAX_LENGTH];
	C8 *STR_SYSTEM_CONTROL_RX_PACKET_SIZE;
	C8 *STR_SYSTEM_CONTROL_TX_PACKET_SIZE;

	typedef struct {
		int exist;
		int fd;
		char *info_buffer;
		int size;
	} content_dir_info;

	typedef enum {
		MSCP_BACK_TO_START = 3
	} MSCPRemoteControlCommand;

	content_dir_info SystemControlDirectoryInfo;

	/* private functions */
	SystemControlCommand_t SystemControlFindCommand(const char *CommandBuffer,
													SystemControlCommand_t * CurrentCommand, int *ArgCount);
	I32 SystemControlInitServer(int *ClientSocket, int *ServerHandle, struct in_addr *ip_addr);
	I32 SystemControlConnectServer(int *sockfd, const char *name, const uint32_t port);
	void SystemControlSendBytes(const char *data, int length, int *sockfd, int debug);
	void SystemControlSendControlResponse(U16 ResponseStatus, C8 * ResponseString, C8 * ResponseData,
										I32 ResponseDataLength, I32 * Sockfd, U8 Debug);
	I32 SystemControlBuildControlResponse(U16 ResponseStatus, C8 * ResponseString, C8 * ResponseData,
										I32 ResponseDataLength, U8 Debug);
	void SystemControlFileDownloadResponse(U16 ResponseStatus, C8 * ResponseString,
										I32 ResponseDataLength, I32 * Sockfd, U8 Debug);
	void SystemControlSendLog(C8 * LogString, I32 * Sockfd, U8 Debug);
	void SystemControlSendMONR(C8 * LogString, I32 * Sockfd, U8 Debug);
	void SystemControlCreateProcessChannel(const C8 * name, const U32 port, I32 * sockfd,
												struct sockaddr_in *addr);
	//I32 SystemControlSendUDPData(I32 *sockfd, struct sockaddr_in* addr, C8 *SendData, I32 Length, U8 debug);
	I32 SystemControlReadServerParameterList(C8 * ParameterList, U8 debug);
	I32 SystemControlGetServerParameter(GSDType * GSD, C8 * ParameterName, C8 * ReturnValue, U32 BufferLength,
										U8 Debug);
	I32 SystemControlSetServerParameter(GSDType * GSD, C8 * ParameterName, C8 * NewValue, U8 Debug);
	I32 SystemControlCheckFileDirectoryExist(C8 * ParameterName, C8 * ReturnValue, U8 Debug);
	I32 SystemControlUploadFile(C8 * Filename, C8 * FileSize, C8 * PacketSize, C8 * FileType, C8 * ReturnValue,
								C8 * CompleteFilePath, U8 Debug);
	I32 SystemControlReceiveRxData(I32 * sockfd, C8 * Path, C8 * FileSize, C8 * PacketSize, C8 * ReturnValue,
								U8 Debug);
	C8 SystemControlDeleteTrajectory(const C8 * trajectoryName, const size_t nameLen);
	C8 SystemControlDeleteGeofence(const C8 * geofenceName, const size_t nameLen);
	C8 SystemControlDeleteGenericFile(const C8 * filePath, const size_t nameLen);
	C8 SystemControlClearTrajectories(void);
	C8 SystemControlClearGeofences(void);
	C8 SystemControlClearObjects(void);
	I32 SystemControlDeleteFileDirectory(C8 * Path, C8 * ReturnValue, U8 Debug);
	I32 SystemControlBuildFileContentInfo(C8 * Path, U8 Debug);
	I32 SystemControlDestroyFileContentInfo(C8 * Path, U8 RemoveFile);
	I32 SystemControlSendFileContent(I32 * sockfd, C8 * Path, C8 * PacketSize, C8 * ReturnValue, U8 Remove,
									U8 Debug);
	I32 SystemControlCreateDirectory(C8 * Path, C8 * ReturnValue, U8 Debug);
	I32 SystemControlBuildRVSSTimeChannelMessage(C8 * RVSSData, U32 * RVSSDataLengthU32, TimeType * GPSTime,
												U8 Debug);
	I32 SystemControlBuildRVSSMaestroChannelMessage(C8 * RVSSData, U32 * RVSSDataLengthU32, GSDType * GSD,
													U8 SysCtrlState, U8 Debug);
	I32 SystemControlBuildRVSSAspChannelMessage(C8 * RVSSData, U32 * RVSSDataLengthU32, U8 Debug);
	int32_t SystemControlSendRVSSMonitorChannelMessages(int *socket, struct sockaddr_in *addr);
	void SystemControlUpdateRVSSSendTime(struct timeval *currentRVSSSendTime, uint8_t RVSSRate_Hz);

	I32 SystemControlGetStatusMessage(char *respondingModule, size_t arrayLength, U8 debug);

	ssize_t SystemControlReceiveUserControlData(I32 socket, C8 * dataBuffer, size_t dataBufferLength);
	C8 SystemControlVerifyHostAddress(char *ip);
	static void signalHandler(int signo);

	void appendSysInfoString(char *ControlResponseBuffer, const size_t bufferSize);

	/* callbacks */
	virtual void initCB(const Empty::ConstPtr&) override;
	virtual void connectCB(const Empty::ConstPtr&) override;
	virtual void armCB(const Empty::ConstPtr&) override;
	virtual void startCB(const Empty::ConstPtr&) override;
	virtual void failureCB(const String::ConstPtr&) override;
	
	public:
	std::string name;
	void systemcontrol_task(TimeType * GPSTime, GSDType * GSD, LOG_LEVEL logLevel);
};
